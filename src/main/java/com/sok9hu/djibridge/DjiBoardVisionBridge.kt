package com.sok9hu.djibridge

import android.graphics.ImageFormat
import android.media.Image
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager.ScaleType
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean

/**
 * A low-rate CPU frame side channel for the DJI reference-board detector. It
 * owns a separate ImageReader surface and never reads the Unity OES texture.
 */
object DjiBoardVisionBridge {
    private const val TAG = "DjiBoardVision"
    private const val HEADER_LENGTH = 17
    private const val MARKER_OUTPUT_LENGTH = 18
    private const val LAYOUT_STRIDE = 9

    private val lock = Any()
    private val processing = AtomicBoolean(false)

    @Volatile private var latestResultJson = ""
    @Volatile private var configuration: Configuration? = null

    private var imageReader: ImageReader? = null
    private var workerThread: HandlerThread? = null
    private var workerHandler: Handler? = null
    private var attachedSurface: Surface? = null
    private var nextDetectionAtMs = 0L
    private var submittedFrames = 0L
    private var nextCaptureAtMs = 0L
    private var captureRemaining = 0
    private var captureIndex = 0
    private var captureDirectory: File? = null
    private var frameDescriptorLogged = false

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
            get() = fx > 0f && fy > 0f && cx.isFinite() && cy.isFinite()
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
        require(markerLayout.size % LAYOUT_STRIDE == 0) { "Marker layout must use $LAYOUT_STRIDE floats per marker" }
        val next = Configuration(
            width = width,
            height = height,
            fx = fx,
            fy = fy,
            cx = cx,
            cy = cy,
            distortionCoefficients = distortionCoefficients.copyOf(5),
            markerLayout = markerLayout.copyOf(),
            detectionIntervalMs = detectionIntervalMs.coerceAtLeast(50).toLong()
        )
        synchronized(lock) {
            configuration = next
            latestResultJson = ""
        }
        val distortionDescription = next.distortionCoefficients.joinToString(prefix = "[", postfix = "]")
        Log.i(TAG, "DJI_BOARD_VISION_CONFIGURED frame=${width}x$height markers=${markerLayout.size / LAYOUT_STRIDE} calibrated=${next.hasUsableCalibration} distortion=$distortionDescription")
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
            if (imageReader != null) return true
            val thread = HandlerThread("DjiBoardVisionCpu").also { it.start() }
            val handler = Handler(thread.looper)
            val reader = ImageReader.newInstance(config.width, config.height, ImageFormat.YUV_420_888, 2)
            reader.setOnImageAvailableListener({ source -> onImageAvailable(source) }, handler)
            try {
                manager.putCameraStreamSurface(
                    ComponentIndexType.LEFT_OR_MAIN,
                    reader.surface,
                    config.width,
                    config.height,
                    ScaleType.FIX_XY
                )
                imageReader = reader
                workerThread = thread
                workerHandler = handler
                attachedSurface = reader.surface
                nextDetectionAtMs = 0L
                nextCaptureAtMs = 0L
                submittedFrames = 0L
                frameDescriptorLogged = false
                Log.i(TAG, "DJI_BOARD_CPU_FRAME_SOURCE_STARTED type=ImageReader format=YUV_420_888 frame=${config.width}x${config.height} calibrated=${config.hasUsableCalibration}")
                return true
            } catch (error: Throwable) {
                Log.e(TAG, "DJI_BOARD_CPU_FRAME_SOURCE_FAILED", error)
                reader.close()
                thread.quitSafely()
                return false
            }
        }
    }

    @JvmStatic
    fun stop(cameraStreamManager: ICameraStreamManager?) {
        val surface: Surface?
        val reader: ImageReader?
        val thread: HandlerThread?
        synchronized(lock) {
            surface = attachedSurface
            reader = imageReader
            thread = workerThread
            attachedSurface = null
            imageReader = null
            workerHandler = null
            workerThread = null
            processing.set(false)
        }
        if (surface != null && cameraStreamManager != null) {
            try {
                cameraStreamManager.removeCameraStreamSurface(surface)
            } catch (error: Throwable) {
                Log.w(TAG, "DJI_BOARD_CPU_FRAME_SOURCE_REMOVE_FAILED", error)
            }
        }
        reader?.close()
        thread?.quitSafely()
        try {
            DjiBoardVisionNative.releaseDetector()
        } catch (error: Throwable) {
            Log.w(TAG, "DJI_BOARD_NATIVE_RELEASE_FAILED", error)
        }
        Log.i(TAG, "DJI_BOARD_CPU_FRAME_SOURCE_STOPPED")
    }

    @JvmStatic
    fun getLatestResultJson(): String = latestResultJson

    /** Saves raw luma PGM frames from the exact input passed to native detection. */
    @JvmStatic
    fun requestCalibrationCapture(frameCount: Int): String {
        val application = DJIPlugin.applicationContextOrNull() ?: run {
            Log.w(TAG, "DJI_CALIBRATION_CAPTURE_REJECTED reason=APPLICATION_CONTEXT_UNAVAILABLE")
            return ""
        }
        val count = frameCount.coerceIn(1, 80)
        val root = File(application.getExternalFilesDir("dji-calibration"), "session_${System.currentTimeMillis()}")
        if (!root.mkdirs() && !root.isDirectory) {
            Log.w(TAG, "DJI_CALIBRATION_CAPTURE_REJECTED reason=CREATE_DIRECTORY_FAILED path=${root.absolutePath}")
            return ""
        }
        synchronized(lock) {
            captureDirectory = root
            captureRemaining = count
            captureIndex = 0
            nextCaptureAtMs = 0L
        }
        Log.i(TAG, "DJI_CALIBRATION_CAPTURE_REQUESTED frames=$count intervalMs=500 path=${root.absolutePath}")
        return root.absolutePath
    }

    private fun onImageAvailable(source: ImageReader) {
        val image = try {
            source.acquireLatestImage()
        } catch (error: Throwable) {
            Log.w(TAG, "DJI_BOARD_CPU_FRAME_ACQUIRE_FAILED", error)
            null
        } ?: return
        try {
            val config = configuration ?: return
            val now = SystemClock.elapsedRealtime()
            val luma = copyLumaPlane(image)
            logCalibrationFrameOnce(image)
            captureCalibrationFrameIfRequested(luma, image.width, image.height, image.timestamp, now)
            if (now < nextDetectionAtMs || !processing.compareAndSet(false, true)) return
            nextDetectionAtMs = now + config.detectionIntervalMs
            submittedFrames++
            val raw = DjiBoardVisionNative.detectBoardLuma(
                luma,
                image.width,
                image.height,
                config.fx,
                config.fy,
                config.cx,
                config.cy,
                config.distortionCoefficients,
                config.markerLayout
            )
            latestResultJson = serializeResult(raw, image.width, image.height, image.timestamp, config.hasUsableCalibration)
        } catch (error: Throwable) {
            Log.e(TAG, "DJI_BOARD_CPU_FRAME_PROCESSING_FAILED", error)
        } finally {
            processing.set(false)
            image.close()
        }
    }

    private fun copyLumaPlane(image: Image): ByteArray {
        val plane = image.planes[0]
        val output = ByteArray(image.width * image.height)
        val source = plane.buffer.duplicate()
        val rowStride = plane.rowStride
        val pixelStride = plane.pixelStride
        for (row in 0 until image.height) {
            val rowStart = row * rowStride
            for (column in 0 until image.width) {
                output[row * image.width + column] = source.get(rowStart + column * pixelStride)
            }
        }
        return output
    }

    private fun logCalibrationFrameOnce(image: Image) {
        if (frameDescriptorLogged) return
        frameDescriptorLogged = true
        val aspect = image.width.toFloat() / image.height.toFloat()
        Log.i(
            TAG,
            "DJI_CALIBRATION_FRAME width=${image.width} height=${image.height} " +
                "format=YUV_420_888_LUMA8 rotation=0 crop=NONE resize=NONE mirror=NONE yuvToRgb=NONE"
        )
        Log.i(
            TAG,
            "DJI_RUNTIME_FRAME width=${image.width} height=${image.height} aspect=$aspect rotation=0 " +
                "crop=NONE resize=NONE mirror=NONE pixelFormat=YUV_420_888_LUMA8 detectorInput=RAW_LUMA_PLANE"
        )
    }

    private fun captureCalibrationFrameIfRequested(
        luma: ByteArray,
        width: Int,
        height: Int,
        timestampNs: Long,
        nowMs: Long
    ) {
        val directory: File
        val index: Int
        synchronized(lock) {
            if (captureRemaining <= 0 || nowMs < nextCaptureAtMs) return
            directory = captureDirectory ?: return
            index = captureIndex++
            captureRemaining--
            nextCaptureAtMs = nowMs + 500L
        }

        try {
            val imageFile = File(directory, "frame_%03d_%d.pgm".format(index, timestampNs))
            FileOutputStream(imageFile).use { output ->
                output.write("P5\\n$width $height\\n255\\n".toByteArray(Charsets.US_ASCII))
                output.write(luma)
            }
            val manifest = JSONObject()
                .put("file", imageFile.name)
                .put("width", width)
                .put("height", height)
                .put("timestampNs", timestampNs)
                .put("pixelFormat", "YUV_420_888_LUMA8")
                .put("rotationDegrees", 0)
                .put("crop", "NONE")
                .put("resize", "NONE")
                .put("mirror", "NONE")
                .put("detectorInput", "RAW_LUMA_PLANE")
            File(directory, "capture_manifest.jsonl").appendText(manifest.toString() + "\\n")
            Log.i(TAG, "DJI_CALIBRATION_CAPTURE_SAVED index=$index remaining=$captureRemaining path=${imageFile.absolutePath}")
            if (captureRemaining == 0)
                Log.i(TAG, "DJI_CALIBRATION_CAPTURE_COMPLETE path=${directory.absolutePath}")
        } catch (error: Throwable) {
            Log.e(TAG, "DJI_CALIBRATION_CAPTURE_FAILED", error)
        }
    }

    private fun serializeResult(
        raw: FloatArray?,
        width: Int,
        height: Int,
        timestampNs: Long,
        calibrated: Boolean
    ): String {
        val values = raw ?: FloatArray(HEADER_LENGTH)
        val root = JSONObject()
        root.put("frameWidth", width)
        root.put("frameHeight", height)
        root.put("timestampNs", timestampNs)
        root.put("frameSequence", submittedFrames)
        root.put("detectorFrameFormat", "YUV_420_888_LUMA8")
        root.put("calibrationUsable", calibrated)
        root.put("status", values.getOrElse(0) { 0f }.toInt())
        root.put("markerCount", values.getOrElse(1) { 0f }.toInt())
        root.put("cornerCount", values.getOrElse(2) { 0f }.toInt())
        root.putFinite("reprojectionRms", values.getOrElse(3) { Float.NaN })
        root.putFinite("maxResidual", values.getOrElse(4) { Float.NaN })
        root.put("cameraFromBoardPosition", JSONArray().put(values.getOrElse(5) { 0f }).put(values.getOrElse(6) { 0f }).put(values.getOrElse(7) { 0f }))
        val rotation = JSONArray()
        for (index in 8..16) rotation.put(values.getOrElse(index) { 0f })
        root.put("cameraFromBoardRotationMatrix", rotation)
        val markers = JSONArray()
        var offset = HEADER_LENGTH
        while (offset + MARKER_OUTPUT_LENGTH <= values.size) {
            val marker = JSONObject()
            marker.put("id", values[offset].toInt())
            marker.put("decisionMargin", values[offset + 1])
            marker.put("detectedCorners", pointArray(values, offset + 2))
            marker.put("projectedCorners", pointArray(values, offset + 10))
            markers.put(marker)
            offset += MARKER_OUTPUT_LENGTH
        }
        root.put("markers", markers)
        return root.toString()
    }

    private fun pointArray(values: FloatArray, start: Int): JSONArray {
        val points = JSONArray()
        for (corner in 0 until 4) {
            points.put(JSONArray().put(values.getOrElse(start + corner * 2) { 0f }).put(values.getOrElse(start + corner * 2 + 1) { 0f }))
        }
        return points
    }

    private fun JSONObject.putFinite(name: String, value: Float) {
        put(name, if (value.isFinite()) value else JSONObject.NULL)
    }
}
