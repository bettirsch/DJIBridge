package com.sok9hu.djibridge

import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager.CameraFrameListener
import dji.v5.manager.interfaces.ICameraStreamManager.FrameFormat
import org.json.JSONArray
import org.json.JSONObject
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * A low-rate CPU frame side channel for the DJI reference-board detector. It
 * receives decoded bytes from the MSDK frame listener and never reads Unity's
 * OES texture or Android ImageReader planes.
 */
object DjiBoardVisionBridge {
    private const val TAG = "DjiBoardVision"
    private const val HEADER_LENGTH = 17
    private const val MARKER_OUTPUT_LENGTH = 18
    private const val LAYOUT_STRIDE = 9
    private const val MAX_DETECTOR_FRAME_PIXELS = 16 * 1024 * 1024

    private val lock = Any()
    private val processing = AtomicBoolean(false)
    @Volatile private var detailedLogging = false

    @JvmStatic
    fun setDetailedLogging(enabled: Boolean) {
        detailedLogging = enabled
        DjiBoardVisionNative.setDetailedLogging(enabled)
    }

    @Volatile private var latestResultJson = ""
    @Volatile private var configuration: Configuration? = null

    private enum class ListenerState { RUNNING, STOPPING, STOPPED }

    /**
     * Only owned luma data crosses the DJI callback boundary. MSDK owns the
     * source byte array, so the luma payload is copied before worker dispatch.
     */
    private data class OwnedLumaFrame(
        val luma: ByteArray,
        val width: Int,
        val height: Int,
        // Callback arrival time from SystemClock.elapsedRealtimeNanos(). The
        // CameraFrameListener API exposes no DJI exposure/PTS timestamp.
        val visionFrameTimestampNs: Long,
        val rowStride: Int,
        val pixelStride: Int,
        val format: Int,
        val listenerGeneration: Long,
        val acceptedSequence: Long,
        val configuration: Configuration
    )

    private var detectorThread: HandlerThread? = null
    private var detectorHandler: Handler? = null
    private var frameListener: CameraFrameListener? = null
    private var listenerState = ListenerState.STOPPED
    private var nextListenerGeneration = 0L
    private var activeListenerGeneration = 0L
    @Volatile private var nextDetectionAtMs = 0L
    private var submittedFrames = 0L
    private val acceptedFrames = AtomicLong(0L)
    private val droppedRateLimitedFrames = AtomicLong(0L)
    private val droppedBusyFrames = AtomicLong(0L)
    @Volatile private var diagnosticCapture: ExactDetectorCapture? = null
    private var previousCapturedRotation: FloatArray? = null
    private val nativeMultiStartExperimentEnabled by lazy { DjiBoardVisionNative.isFourTagMultiStartExperimentEnabled() }
    private var frameDescriptorLogged = false
    private var verboseImageLogFramesRemaining = 12
    private var nextImageLifecycleLogAtMs = 0L
    private var timestampSourceLogged = false
    private val firstCpuFrameGeneration = AtomicLong(0L)

    private data class Configuration(
        val width: Int,
        val height: Int,
        val fx: Float,
        val fy: Float,
        val cx: Float,
        val cy: Float,
        val distortionCoefficients: FloatArray,
        val markerLayout: FloatArray,
        val detectionIntervalMs: Long
    ) {
        val hasUsableCalibration: Boolean
            get() = width > 0 && height > 0 &&
                fx.isFinite() && fy.isFinite() && fx > 0f && fy > 0f &&
                cx.isFinite() && cy.isFinite() &&
                distortionCoefficients.size == 5 && distortionCoefficients.all { it.isFinite() }

        fun validateForFrame(frameWidth: Int, frameHeight: Int, lumaLength: Int): String? {
            if (!hasUsableCalibration) return "CALIBRATION_INVALID"
            if (width != frameWidth || height != frameHeight) {
                return "FRAME_GEOMETRY_MISMATCH runtime=${frameWidth}x$frameHeight configured=${width}x$height"
            }
            val expectedLumaLength = frameWidth.toLong() * frameHeight.toLong()
            if (expectedLumaLength <= 0L || expectedLumaLength > MAX_DETECTOR_FRAME_PIXELS) {
                return "FRAME_PIXEL_COUNT_INVALID pixels=$expectedLumaLength"
            }
            if (lumaLength.toLong() != expectedLumaLength) {
                return "LUMA_LENGTH_MISMATCH actual=$lumaLength expected=$expectedLumaLength"
            }
            if (markerLayout.isEmpty() || markerLayout.size % LAYOUT_STRIDE != 0) {
                return "MARKER_LAYOUT_SIZE_INVALID length=${markerLayout.size}"
            }
            for (offset in markerLayout.indices step LAYOUT_STRIDE) {
                val id = markerLayout[offset]
                val sizeMeters = markerLayout[offset + 1]
                if (!id.isFinite() || id != id.toInt().toFloat() || !sizeMeters.isFinite() || sizeMeters <= 0f) {
                    return "MARKER_LAYOUT_ENTRY_INVALID offset=$offset"
                }
                for (index in offset + 2 until offset + LAYOUT_STRIDE) {
                    if (!markerLayout[index].isFinite()) return "MARKER_LAYOUT_NONFINITE offset=$offset"
                }
            }
            return null
        }
    }

    /**
     * markerLayout uses nine floats per marker: id, sizeM, boardPosition xyz,
     * boardRotation xyzw. Values belong to the physical board frame.
     */
    @JvmStatic
    fun configure(
        width: Int,
        height: Int,
        fx: Float,
        fy: Float,
        cx: Float,
        cy: Float,
        distortionCoefficients: FloatArray,
        markerLayout: FloatArray,
        detectionIntervalMs: Int
    ) {
        val next = Configuration(
            width = width,
            height = height,
            fx = fx,
            fy = fy,
            cx = cx,
            cy = cy,
            distortionCoefficients = if (distortionCoefficients.size == 5) distortionCoefficients.copyOf() else FloatArray(0),
            markerLayout = markerLayout.copyOf(),
            detectionIntervalMs = detectionIntervalMs.coerceAtLeast(50).toLong()
        )
        synchronized(lock) {
            configuration = next
            latestResultJson = ""
        }
        val distortionDescription = next.distortionCoefficients.joinToString(prefix = "[", postfix = "]")
        Log.i(TAG, "DJI_BOARD_VISION_CONFIGURED frame=${width}x$height markers=${markerLayout.size / LAYOUT_STRIDE} calibrated=${next.hasUsableCalibration} distortion=$distortionDescription")
        next.validateForFrame(width, height, width * height)?.let { reason ->
            Log.w(TAG, "DJI_BOARD_VISION_CONFIG_REJECTED reason=$reason")
        }
    }

    @JvmStatic
    fun start(cameraStreamManager: ICameraStreamManager?): Boolean {
        val config = configuration ?: run {
            Log.w(TAG, "DJI_BOARD_VISION_START_REJECTED reason=NOT_CONFIGURED")
            return false
        }
        val manager = cameraStreamManager ?: run {
            Log.w(TAG, "DJI_BOARD_VISION_START_REJECTED reason=CAMERA_STREAM_MANAGER_UNAVAILABLE")
            return false
        }
        synchronized(lock) {
            if (listenerState == ListenerState.RUNNING && frameListener != null) return true
            if (listenerState == ListenerState.STOPPING) {
                Log.w(TAG, "DJI_BOARD_VISION_START_REJECTED reason=FRAME_LISTENER_STOPPING")
                return false
            }

            val generation = ++nextListenerGeneration
            val listener = CameraFrameListener { frameData, offset, length, width, height, format ->
                onCameraFrame(frameData, offset, length, width, height, format, generation)
            }
            try {
                ensureDetectorHandlerLocked()
                frameListener = listener
                activeListenerGeneration = generation
                listenerState = ListenerState.RUNNING
                manager.addFrameListener(
                    ComponentIndexType.LEFT_OR_MAIN,
                    FrameFormat.NV21,
                    listener
                )
                nextDetectionAtMs = 0L
                submittedFrames = 0L
                acceptedFrames.set(0L)
                droppedRateLimitedFrames.set(0L)
                droppedBusyFrames.set(0L)
                frameDescriptorLogged = false
                firstCpuFrameGeneration.set(0L)
                verboseImageLogFramesRemaining = 12
                nextImageLifecycleLogAtMs = 0L
                Log.i(TAG, "DJI_FRAME_LISTENER_REGISTERED generation=$generation camera=LEFT_OR_MAIN requestedFormat=NV21")
                Log.i(TAG, "DJI_BOARD_CPU_FRAME_SOURCE_STARTED type=CameraFrameListener requestedFormat=NV21 calibrated=${config.hasUsableCalibration}")
                return true
            } catch (error: Throwable) {
                Log.e(TAG, "DJI_BOARD_CPU_FRAME_SOURCE_FAILED", error)
                try {
                    manager.removeFrameListener(listener)
                } catch (removeError: Throwable) {
                    Log.w(TAG, "DJI_FRAME_LISTENER_REMOVE_AFTER_START_FAILURE_FAILED", removeError)
                }
                frameListener = null
                activeListenerGeneration = 0L
                listenerState = ListenerState.STOPPED
                return false
            }
        }
    }

    @JvmStatic
    fun stop(cameraStreamManager: ICameraStreamManager?, reason: String = "EXPLICIT_STOP") {
        val listener: CameraFrameListener?
        val generation: Long
        synchronized(lock) {
            if (listenerState == ListenerState.STOPPED) return
            if (listenerState == ListenerState.STOPPING) {
                Log.i(TAG, "DJI_FRAME_LISTENER_STOPPING_ALREADY")
                return
            }
            listenerState = ListenerState.STOPPING
            listener = frameListener
            generation = activeListenerGeneration
            frameListener = null
            activeListenerGeneration = 0L
        }
        if (listener != null && cameraStreamManager != null) {
            try {
                cameraStreamManager.removeFrameListener(listener)
            } catch (error: Throwable) {
                Log.w(TAG, "DJI_FRAME_LISTENER_REMOVE_FAILED", error)
            }
        }
        enqueueDetectorCleanup(generation, reason)
    }

    /** Snapshot helpers used only by the video-lifecycle owner for idempotent registration. */
    @JvmStatic
    fun isListenerRegistered(): Boolean = synchronized(lock) {
        listenerState == ListenerState.RUNNING && frameListener != null
    }

    @JvmStatic
    fun activeListenerGeneration(): Long = synchronized(lock) { activeListenerGeneration }

    @JvmStatic
    fun listenerStateName(): String = synchronized(lock) { listenerState.name }

    @JvmStatic
    fun getLatestResultJson(): String = latestResultJson

    /** Called only by the separately compiled diagnostic APK, never normal startup. */
    @JvmStatic
    fun configureDiagnosticCapture(sessionId: String, enabled: Boolean) {
        diagnosticCapture?.close()
        diagnosticCapture = null
        previousCapturedRotation = null
        if (!enabled) return
        val application = DJIPlugin.applicationContextOrNull() ?: return
        val external = application.getExternalFilesDir("dji-calibration") ?: return
        diagnosticCapture = ExactDetectorCapture(external, sessionId, SystemClock::elapsedRealtimeNanos)
        Log.i(TAG, "DJI_EXACT_Y_CAPTURE_CAPABLE session=$sessionId armed=false root=$external")
    }

    private fun onCameraFrame(
        frameData: ByteArray,
        offset: Int,
        length: Int,
        width: Int,
        height: Int,
        format: FrameFormat,
        callbackGeneration: Long
    ) {
        // Capture this before validation/copying so it represents the same
        // physical callback arrival instant throughout the vision result.
        val frameArrivalTimestampNs = SystemClock.elapsedRealtimeNanos()
        val now = SystemClock.elapsedRealtime()
        if (!timestampSourceLogged) {
            timestampSourceLogged = true
            Log.i(
                TAG,
                "DJI_VIDEO_TIMESTAMP_SOURCE=SystemClock.elapsedRealtimeNanos " +
                    "unit=ns clockDomain=ANDROID_MONOTONIC provenance=CAMERA_FRAME_LISTENER_CALLBACK_ARRIVAL " +
                    "sdkFrameTimestamp=NOT_EXPOSED"
            )
        }
        val verbose = detailedLogging && shouldLogFrameLifecycle(now)
        if (verbose) {
            Log.i(TAG, "DJI_VISION_FRAME_CALLBACK generation=$callbackGeneration width=$width height=$height offset=$offset length=$length format=$format formatValue=${format.value} expectedLumaBytes=${width.toLong() * height.toLong()} thread=${Thread.currentThread().name}")
        }
        val config = synchronized(lock) {
            if (listenerState != ListenerState.RUNNING) {
                logFrameRejected("listener_stopping generation=$callbackGeneration state=$listenerState")
                return
            }
            if (callbackGeneration != activeListenerGeneration) {
                logFrameRejected("stale_listener_generation callback=$callbackGeneration active=$activeListenerGeneration")
                return
            }
            configuration
        } ?: run {
            logFrameRejected("not_configured")
            return
        }

        if (firstCpuFrameGeneration.compareAndSet(0L, callbackGeneration)) {
            Log.i(
                TAG,
                "DJI_BOARD_FIRST_CPU_FRAME width=$width height=$height format=$format " +
                    "generation=$callbackGeneration"
            )
        }

        val lumaBytes = width.toLong() * height.toLong()
        if (format != FrameFormat.NV21) {
            logFrameRejected("unexpected_format actual=$format requested=NV21")
            return
        }
        if (width <= 0 || height <= 0 || lumaBytes <= 0L || lumaBytes > MAX_DETECTOR_FRAME_PIXELS.toLong()) {
            logFrameRejected("invalid_geometry width=$width height=$height lumaBytes=$lumaBytes")
            return
        }
        if (offset < 0 || length < lumaBytes || offset.toLong() + lumaBytes > frameData.size.toLong()) {
            logFrameRejected("invalid_frame_range offset=$offset length=$length bufferBytes=${frameData.size} expectedLumaBytes=$lumaBytes")
            return
        }
        if (width != config.width || height != config.height) {
            publishRejectedFrame(width, height, frameArrivalTimestampNs, "CALIBRATION_GEOMETRY_MISMATCH runtime=${width}x$height configured=${config.width}x${config.height}")
            return
        }
        if (processing.get()) {
            val dropped = droppedBusyFrames.incrementAndGet()
            if (verbose) {
                Log.i(TAG, "DJI_VISION_FRAME_DROPPED reason=detector_busy generation=$callbackGeneration droppedBusyCount=$dropped")
            }
            return
        }
        if (now < nextDetectionAtMs) {
            val dropped = droppedRateLimitedFrames.incrementAndGet()
            if (verbose) {
                Log.i(TAG, "DJI_VISION_FRAME_DROPPED reason=rate_limit generation=$callbackGeneration nextAllowedAtMs=$nextDetectionAtMs droppedRateLimitCount=$dropped")
            }
            return
        }

        // The MSDK owns frameData; retain only an owned tightly packed Y image for the worker.
        var ownsProcessing = false
        try {
            val ownedLuma = frameData.copyOfRange(offset, offset + lumaBytes.toInt())
            if (!processing.compareAndSet(false, true)) {
                val dropped = droppedBusyFrames.incrementAndGet()
                if (verbose) {
                    Log.i(TAG, "DJI_VISION_FRAME_DROPPED reason=detector_busy generation=$callbackGeneration droppedBusyCount=$dropped")
                }
                return
            }
            ownsProcessing = true

            // The callback is the sole owner of rate-limit admission. The worker always
            // processes a frame after this point and clears processing in its finally block.
            nextDetectionAtMs = now + config.detectionIntervalMs
            val acceptedSequence = acceptedFrames.incrementAndGet()
            val frame = OwnedLumaFrame(
                luma = ownedLuma,
                width = width,
                height = height,
                visionFrameTimestampNs = frameArrivalTimestampNs,
                rowStride = width,
                pixelStride = 1,
                format = format.value,
                listenerGeneration = callbackGeneration,
                acceptedSequence = acceptedSequence,
                configuration = config
            )
            val handler = synchronized(lock) { detectorHandler }
            if (handler == null || !handler.post { processOwnedFrame(frame) }) {
                processing.set(false)
                ownsProcessing = false
                publishRejectedFrame(width, height, frameArrivalTimestampNs, "DETECTOR_WORKER_UNAVAILABLE")
                return
            }
            if (detailedLogging) Log.i(
                TAG,
                "DJI_VISION_FRAME_ACCEPTED sequence=$acceptedSequence generation=$callbackGeneration " +
                    "intervalMs=${config.detectionIntervalMs} acceptedCount=${acceptedFrames.get()} " +
                    "droppedRateLimitCount=${droppedRateLimitedFrames.get()} droppedBusyCount=${droppedBusyFrames.get()}"
            )
            if (verbose) {
                Log.i(TAG, "DJI_VISION_FRAME_RECEIVED generation=$callbackGeneration lumaBytes=${frame.luma.size} stride=${frame.rowStride} pixelStride=${frame.pixelStride}")
            }
        } catch (error: Throwable) {
            if (ownsProcessing) processing.set(false)
            Log.e(TAG, "DJI_VISION_FRAME_ACCEPTANCE_FAILED generation=$callbackGeneration", error)
            publishRejectedFrame(width, height, frameArrivalTimestampNs, "FRAME_ACCEPTANCE_FAILED_${error.javaClass.simpleName}")
        }
    }

    private fun processOwnedFrame(frame: OwnedLumaFrame) {
        var processedSequence = 0L
        var detectorStartTimestampNs = 0L
        try {
            val config = frame.configuration
            synchronized(lock) {
                if (listenerState != ListenerState.RUNNING || frame.listenerGeneration != activeListenerGeneration) {
                    Log.w(TAG, "DJI_VISION_FRAME_REJECTED reason=stale_listener_generation frameGeneration=${frame.listenerGeneration} activeGeneration=$activeListenerGeneration")
                    return
                }
            }
            submittedFrames++
            processedSequence = submittedFrames
            if (detailedLogging) Log.i(TAG, "DJI_VISION_WORKER_BEGIN acceptedSequence=${frame.acceptedSequence} sequence=$processedSequence generation=${frame.listenerGeneration}")
            logCalibrationFrameOnce(frame)
            if (detailedLogging) Log.i(TAG, "BOARD_STAGE_01_FRAME_RECEIVED sequence=$processedSequence width=${frame.width} height=${frame.height} lumaBytes=${frame.luma.size} generation=${frame.listenerGeneration}")
            val rejectionReason = config.validateForFrame(frame.width, frame.height, frame.luma.size)
            if (rejectionReason != null) {
                publishRejectedFrame(frame.width, frame.height, frame.visionFrameTimestampNs, rejectionReason)
                return
            }

            val capture = diagnosticCapture
            val captureThisFrame = capture?.wantsFrame(SystemClock.elapsedRealtimeNanos()) == true
            if (captureThisFrame) try { DjiBoardVisionNative.setCaptureEvidenceEnabled(true) }
                catch (_: Throwable) { capture?.errors?.incrementAndGet() }
            if (nativeMultiStartExperimentEnabled) {
                DjiBoardVisionNative.setPoseSelectionContext(frame.visionFrameTimestampNs, frame.listenerGeneration, processedSequence)
            }
            detectorStartTimestampNs = SystemClock.elapsedRealtimeNanos()
            var evidence = "null"
            var detectorEndTimestampNs = 0L
            val raw = try { DjiBoardVisionNative.detectBoardLuma(
                frame.luma,
                frame.width,
                frame.height,
                config.fx,
                config.fy,
                config.cx,
                config.cy,
                config.distortionCoefficients,
                config.markerLayout
            ).also { detectorEndTimestampNs = SystemClock.elapsedRealtimeNanos() } } finally {
                if (captureThisFrame) {
                    try { evidence = DjiBoardVisionNative.getCaptureEvidenceJson() ?: "null" }
                    catch (_: Throwable) { capture?.errors?.incrementAndGet() }
                    finally { try { DjiBoardVisionNative.setCaptureEvidenceEnabled(false) } catch (_: Throwable) { capture?.errors?.incrementAndGet() } }
                }
            }
            // AprilTag and board PnP run inside one native call. Its return is
            // therefore the first trustworthy timestamp for completed PnP.
            val pnpCompletedTimestampNs = detectorEndTimestampNs
            if (detailedLogging && (processedSequence <= 3L || processedSequence % 10L == 0L)) {
                val queueDelayMs = (detectorStartTimestampNs - frame.visionFrameTimestampNs) / 1_000_000.0
                val processingMs = (detectorEndTimestampNs - detectorStartTimestampNs) / 1_000_000.0
                Log.i(
                    TAG,
                    "DJI_VISION_PROCESSING_TIMESTAMPS frameTimestampNs=${frame.visionFrameTimestampNs} " +
                        "detectorStartTimestampNs=$detectorStartTimestampNs detectorEndTimestampNs=$detectorEndTimestampNs " +
                        "detectorQueueDelayMs=$queueDelayMs detectorProcessingMs=$processingMs " +
                        "clockDomain=ANDROID_MONOTONIC"
                )
            }
            val resultJson = serializeResult(
                raw,
                frame.width,
                frame.height,
                frame.visionFrameTimestampNs,
                detectorStartTimestampNs,
                detectorEndTimestampNs,
                pnpCompletedTimestampNs,
                config.hasUsableCalibration,
                processedSequence,
                frame.listenerGeneration
            )
            latestResultJson = resultJson
            if (captureThisFrame && capture != null) {
                // Capture failures must never replace a valid detector result or affect tracking.
                try {
                    val categories = linkedSetOf<String>()
                    val count = raw?.getOrNull(1)?.toInt() ?: 0
                    if (raw?.getOrNull(0)?.toInt() == -8) categories.add("NO_TAG")
                    if (count in 1..4) categories.add("TAGS_$count")
                    if (raw != null && raw.size >= HEADER_LENGTH && raw[0] == 2f) {
                        for (marker in 0 until count) {
                            for (corner in 0..3) {
                                val offset = HEADER_LENGTH + marker * MARKER_OUTPUT_LENGTH + 2 + corner * 2
                                val x = raw[offset]; val y = raw[offset + 1]
                                if (minOf(x, y, frame.width - x, frame.height - y) < 32f) categories.add("NEAR_BORDER")
                            }
                        }
                        if (kotlin.math.abs(raw[16]) < .7f) categories.add("OBLIQUE")
                        val rotation = raw.copyOfRange(8, 17)
                        previousCapturedRotation?.let { previous ->
                            val cosine = ((rotation.indices.sumOf { (rotation[it] * previous[it]).toDouble() } - 1) / 2).coerceIn(-1.0, 1.0)
                            if (Math.toDegrees(kotlin.math.acos(cosine)) > 15) categories.add("RAPID_POSE_ORIENTATION_CHANGE")
                        }
                        previousCapturedRotation = rotation
                    }
                    val evidenceJson = if (evidence == "null") JSONObject() else JSONObject(evidence)
                    for (field in arrayOf("selectedInitializerMarkerId", "initializerPose", "initializerTagPoseError", "finalRefinedPose", "initialPoseFinite", "initialMinDepth", "initialMaxDepth", "refinementFailureDetail")) {
                        if (!evidenceJson.has(field)) evidenceJson.put(field, JSONObject.NULL)
                    }
                    evidenceJson.put("calibration", JSONObject().put("fx", config.fx).put("fy", config.fy)
                        .put("cx", config.cx).put("cy", config.cy)
                        .put("distortion", JSONArray(config.distortionCoefficients.map { it.toDouble() }))
                        .put("markerLayoutStride", LAYOUT_STRIDE).put("markerLayout", JSONArray(config.markerLayout.map { it.toDouble() })))
                    capture.submit(DetectorCapturePacket(
                        DetectorCaptureIdentity(capture.sessionId, frame.listenerGeneration, processedSequence,
                            frame.acceptedSequence, frame.visionFrameTimestampNs, frame.width, frame.height, frame.rowStride),
                        frame.luma, detectorStartTimestampNs, detectorEndTimestampNs, resultJson, evidenceJson.toString(), categories))
                } catch (error: Exception) { capture.errors.incrementAndGet(); Log.w(TAG, "DJI_EXACT_Y_PACKET_FAILED", error) }
            }
            if (detailedLogging) Log.i(TAG, "BOARD_STAGE_09_RESULT_SENT_TO_UNITY sequence=$processedSequence nativeResultLength=${raw?.size ?: 0}")
        } catch (error: Throwable) {
            Log.e(TAG, "DJI_BOARD_OWNED_FRAME_PROCESSING_FAILED", error)
            publishRejectedFrame(
                frame.width,
                frame.height,
                frame.visionFrameTimestampNs,
                "KOTLIN_EXCEPTION_${error.javaClass.simpleName}",
                detectorStartTimestampNs,
                SystemClock.elapsedRealtimeNanos()
            )
        } finally {
            processing.set(false)
            if (detailedLogging) Log.i(TAG, "DJI_VISION_WORKER_END acceptedSequence=${frame.acceptedSequence} sequence=$processedSequence")
        }
    }

    private fun shouldLogFrameLifecycle(nowMs: Long): Boolean {
        if (verboseImageLogFramesRemaining > 0) {
            verboseImageLogFramesRemaining--
            return true
        }
        if (nowMs >= nextImageLifecycleLogAtMs) {
            nextImageLifecycleLogAtMs = nowMs + 5_000L
            return true
        }
        return false
    }

    private fun logFrameRejected(reason: String) {
        Log.w(TAG, "DJI_VISION_FRAME_REJECTED reason=$reason")
    }

    private fun logCalibrationFrameOnce(frame: OwnedLumaFrame) {
        if (frameDescriptorLogged) return
        frameDescriptorLogged = true
        val aspect = frame.width.toFloat() / frame.height.toFloat()
        Log.i(
            TAG,
            "DJI_CALIBRATION_FRAME width=${frame.width} height=${frame.height} " +
                "format=NV21_LUMA8 rotation=0 crop=NONE resize=NONE mirror=NONE yuvToRgb=NONE"
        )
        Log.i(
            TAG,
            "DJI_RUNTIME_FRAME width=${frame.width} height=${frame.height} aspect=$aspect rotation=0 " +
                "crop=NONE resize=NONE mirror=NONE pixelFormat=NV21_LUMA8 detectorInput=MSDK_FRAME_LISTENER_LUMA"
        )
    }

    private fun ensureDetectorHandlerLocked() {
        if (detectorHandler != null) return
        val thread = HandlerThread("DjiBoardVisionDetector").also { it.start() }
        detectorThread = thread
        detectorHandler = Handler(thread.looper)
    }

    private fun enqueueDetectorCleanup(generation: Long, reason: String) {
        val handler = synchronized(lock) { detectorHandler }
        val cleanup: () -> Unit = {
            try {
                DjiBoardVisionNative.releaseDetector()
            } catch (error: Throwable) {
                Log.w(TAG, "DJI_BOARD_NATIVE_RELEASE_FAILED", error)
            } finally {
                synchronized(lock) {
                    if (listenerState == ListenerState.STOPPING && activeListenerGeneration == 0L) {
                        listenerState = ListenerState.STOPPED
                    }
                }
                Log.i(TAG, "DJI_FRAME_LISTENER_REMOVED generation=$generation reason=$reason")
                DJIUnityVideoBridge.onBoardFrameListenerStopped(generation, reason)
            }
            Unit
        }
        if (handler == null || !handler.post(Runnable(cleanup))) cleanup()
    }

    private fun serializeResult(
        raw: FloatArray?,
        width: Int,
        height: Int,
        visionFrameTimestampNs: Long,
        detectorStartTimestampNs: Long,
        detectorEndTimestampNs: Long,
        pnpCompletedTimestampNs: Long,
        calibrated: Boolean,
        visionFrameId: Long,
        listenerGeneration: Long
    ): String {
        if (raw == null || raw.size < HEADER_LENGTH) {
            return serializeRejectedFrame(width, height, visionFrameTimestampNs, "JNI_RESULT_MISSING_OR_SHORT length=${raw?.size ?: 0}", detectorStartTimestampNs, detectorEndTimestampNs)
        }
        val values = raw
        val status = values.getOrElse(0) { 0f }.toInt()
        val markerCount = values.getOrElse(1) { 0f }.toInt()
        val correspondenceCount = values.getOrElse(2) { 0f }.toInt()
        val expectedLength = HEADER_LENGTH + markerCount * MARKER_OUTPUT_LENGTH
        if (markerCount < 0 || correspondenceCount < 0 || values.size != expectedLength) {
            return serializeRejectedFrame(
                width,
                height,
                visionFrameTimestampNs,
                "JNI_RESULT_LAYOUT_MISMATCH markers=$markerCount correspondences=$correspondenceCount expectedLength=$expectedLength actualLength=${values.size}",
                detectorStartTimestampNs,
                detectorEndTimestampNs
            )
        }

        // Transport contract: every recognized AprilTag contributes exactly four
        // correspondences. Corners cross JSON as flattened XY primitive arrays;
        // Unity JsonUtility does not reliably deserialize jagged float[][] arrays.
        val markerIds = JSONArray()
        val decisionMargins = JSONArray()
        val detectedCornersXY = JSONArray()
        val projectedCornersXY = JSONArray()
        val residuals = JSONArray()
        var offset = HEADER_LENGTH
        for (markerIndex in 0 until markerCount) {
            val markerId = values[offset].toInt()
            val decisionMargin = values[offset + 1]
            if (!decisionMargin.isFinite()) {
                return serializeRejectedFrame(width, height, visionFrameTimestampNs, "JNI_NONFINITE_DECISION_MARGIN markerIndex=$markerIndex", detectorStartTimestampNs, detectorEndTimestampNs)
            }
            markerIds.put(markerId)
            decisionMargins.put(decisionMargin)
            for (cornerIndex in 0 until 4) {
                val detectedX = values[offset + 2 + cornerIndex * 2]
                val detectedY = values[offset + 3 + cornerIndex * 2]
                val projectedX = values[offset + 10 + cornerIndex * 2]
                val projectedY = values[offset + 11 + cornerIndex * 2]
                if (!detectedX.isFinite() || !detectedY.isFinite() || !projectedX.isFinite() || !projectedY.isFinite()) {
                    return serializeRejectedFrame(
                        width,
                        height,
                        visionFrameTimestampNs,
                        "JNI_NONFINITE_CORNER markerIndex=$markerIndex cornerIndex=$cornerIndex",
                        detectorStartTimestampNs,
                        detectorEndTimestampNs
                    )
                }
                detectedCornersXY.put(detectedX)
                detectedCornersXY.put(detectedY)
                projectedCornersXY.put(projectedX)
                projectedCornersXY.put(projectedY)
                residuals.put(kotlin.math.hypot((detectedX - projectedX).toDouble(), (detectedY - projectedY).toDouble()).toFloat())
            }
            offset += MARKER_OUTPUT_LENGTH
        }
        if (correspondenceCount != markerCount * 4 ||
            detectedCornersXY.length() != correspondenceCount * 2 ||
            projectedCornersXY.length() != correspondenceCount * 2 ||
            residuals.length() != correspondenceCount) {
            return serializeRejectedFrame(
                width,
                height,
                visionFrameTimestampNs,
                "JNI_CORNER_CONTRACT_MISMATCH markers=$markerCount correspondences=$correspondenceCount detectedFloats=${detectedCornersXY.length()} projectedFloats=${projectedCornersXY.length()} residuals=${residuals.length()}",
                detectorStartTimestampNs,
                detectorEndTimestampNs
            )
        }

        val root = JSONObject()
        root.put("frameWidth", width)
        root.put("frameHeight", height)
        // timestampNs is retained for older Unity consumers. Both fields are
        // the exact same callback-arrival timestamp, never a worker timestamp.
        root.put("timestampNs", visionFrameTimestampNs)
        root.put("visionFrameTimestampNs", visionFrameTimestampNs)
        root.put("frameArrivalTimestampNs", visionFrameTimestampNs)
        root.put("detectorStartTimestampNs", detectorStartTimestampNs)
        root.put("detectorEndTimestampNs", detectorEndTimestampNs)
        root.put("pnpCompletedTimestampNs", pnpCompletedTimestampNs)
        root.put("frameSequence", visionFrameId)
        root.put("listenerGeneration", listenerGeneration)
        root.put("visionFrameId", visionFrameId)
        root.put("detectorFrameFormat", "NV21_LUMA8")
        root.put("pnpTimingGranularity", "COUPLED_NATIVE_APRILTAG_AND_PNP_CALL")
        // This is the exact detector-frame contract from the MSDK callback.
        // The Unity calibration gate must compare these values, not infer them
        // from the rendered OES video path.
        root.put("frameRotationDegrees", 0)
        root.put("frameCrop", "NONE")
        root.put("frameResize", "NONE")
        root.put("frameMirrorX", false)
        root.put("calibrationUsable", calibrated)
        root.put("status", status)
        root.put("rejectionReason", if (status < 0) "NATIVE_FAILURE_CODE_${-status}" else "")
        root.put("markerCount", markerCount)
        root.put("cornerCount", correspondenceCount)
        root.put("correspondenceCount", correspondenceCount)
        root.putFinite("reprojectionRms", values.getOrElse(3) { Float.NaN })
        root.putFinite("maxResidual", values.getOrElse(4) { Float.NaN })
        root.put("cameraFromBoardPosition", JSONArray().put(values.getOrElse(5) { 0f }).put(values.getOrElse(6) { 0f }).put(values.getOrElse(7) { 0f }))
        val rotation = JSONArray()
        for (index in 8..16) rotation.put(values.getOrElse(index) { 0f })
        root.put("cameraFromBoardRotationMatrix", rotation)
        root.put("markerIds", markerIds)
        root.put("markerDecisionMargins", decisionMargins)
        root.put("detectedCornersXY", detectedCornersXY)
        root.put("projectedCornersXY", projectedCornersXY)
        root.put("reprojectionResiduals", residuals)
        if (visionFrameId <= 3L || visionFrameId % 10L == 0L) {
            Log.i(
                TAG,
                "BOARD_RESULT_SERIALIZE frameId=$visionFrameId markerCount=$markerCount markerIdsLength=${markerIds.length()} " +
                    "correspondenceCount=$correspondenceCount detectedCornerFloatCount=${detectedCornersXY.length()} " +
                    "projectedCornerFloatCount=${projectedCornersXY.length()} residualCount=${residuals.length()} " +
                    "representation=FLATTENED_XY_PRIMITIVE_ARRAYS detected0to7=${previewArray(detectedCornersXY)} " +
                    "projected0to7=${previewArray(projectedCornersXY)}"
            )
        }
        return root.toString()
    }

    private fun publishRejectedFrame(
        width: Int,
        height: Int,
        visionFrameTimestampNs: Long,
        reason: String,
        detectorStartTimestampNs: Long = 0L,
        detectorEndTimestampNs: Long = 0L
    ) {
        Log.w(TAG, "DJI_BOARD_FRAME_REJECTED reason=$reason")
        latestResultJson = serializeRejectedFrame(width, height, visionFrameTimestampNs, reason, detectorStartTimestampNs, detectorEndTimestampNs)
    }

    private fun serializeRejectedFrame(
        width: Int,
        height: Int,
        visionFrameTimestampNs: Long,
        reason: String,
        detectorStartTimestampNs: Long = 0L,
        detectorEndTimestampNs: Long = 0L
    ): String {
        return JSONObject()
            .put("frameWidth", width)
            .put("frameHeight", height)
            .put("timestampNs", visionFrameTimestampNs)
            .put("visionFrameTimestampNs", visionFrameTimestampNs)
            .put("frameArrivalTimestampNs", visionFrameTimestampNs)
            .put("detectorStartTimestampNs", detectorStartTimestampNs)
            .put("detectorEndTimestampNs", detectorEndTimestampNs)
            .put("pnpCompletedTimestampNs", detectorEndTimestampNs)
            .put("frameSequence", submittedFrames)
            .put("listenerGeneration", activeListenerGeneration)
            .put("visionFrameId", submittedFrames)
            .put("detectorFrameFormat", "NV21_LUMA8")
            .put("pnpTimingGranularity", "COUPLED_NATIVE_APRILTAG_AND_PNP_CALL")
            .put("frameRotationDegrees", 0)
            .put("frameCrop", "NONE")
            .put("frameResize", "NONE")
            .put("frameMirrorX", false)
            .put("calibrationUsable", false)
            .put("status", -1)
            .put("rejectionReason", reason)
            .put("markerCount", 0)
            .put("cornerCount", 0)
            .put("correspondenceCount", 0)
            .put("reprojectionRms", JSONObject.NULL)
            .put("maxResidual", JSONObject.NULL)
            .put("cameraFromBoardPosition", JSONArray())
            .put("cameraFromBoardRotationMatrix", JSONArray())
            .put("markerIds", JSONArray())
            .put("markerDecisionMargins", JSONArray())
            .put("detectedCornersXY", JSONArray())
            .put("projectedCornersXY", JSONArray())
            .put("reprojectionResiduals", JSONArray())
            .toString()
    }

    private fun previewArray(values: JSONArray): String {
        val count = minOf(8, values.length())
        return buildString {
            append('[')
            for (index in 0 until count) {
                if (index > 0) append(',')
                append(values.optDouble(index))
            }
            append(']')
        }
    }

    private fun JSONObject.putFinite(name: String, value: Float) {
        put(name, if (value.isFinite()) value else JSONObject.NULL)
    }
}
