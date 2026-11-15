package com.sok9hu.djibridge

import android.util.Log
import android.view.Surface
import com.sok9hu.djibridge.video.UnityVideoBridgeController
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager

/**
 * Public Unity-facing API (kept as an object for easy JNI/Unity calls),
 * internally delegating responsibilities to smaller classes.
 */
object DJIUnityVideoBridge {

    private const val TAG = "DJIUnityVideoBridge"

    private val controller by lazy { UnityVideoBridgeController(TAG, ::cameraStreamManager) }

    private data class PendingSurface(
        val surface: Surface,
        val width: Int,
        val height: Int
    )

    @Volatile
    private var pendingSurface: PendingSurface? = null

    @Volatile
    private var boardVisionApiReadyLogged = false

    /**
     * A DroneView provider requests board vision once. The request survives an
     * OES/video restart, while the actual listener remains owned by this bridge.
     */
    @Volatile
    private var boardVisionRequested = false

    @Volatile
    private var videoPipelineStarted = false

    private var hasRegisteredBoardListener = false

    private fun cameraStreamManager(): ICameraStreamManager? =
        MediaDataCenter.getInstance().cameraStreamManager

    private fun updateDecoderSurface(surface: Surface, width: Int, height: Int, caller: String) {
        Log.i(TAG, " - $caller called")

        if (!DJIPlugin.isVideoBridgeReady()) {
            pendingSurface = PendingSurface(surface, width, height)
            Log.i(
                TAG,
                "Deferring $caller until DJI SDK is ready (${DJIPlugin.describeState()})"
            )
            logBoardFrameListenerState("VIDEO_NOT_READY:$caller")
            return
        }

        pendingSurface = null
        controller.startOrUpdate(surface, width, height)
        videoPipelineStarted = controller.isStarted()
        ensureBoardFrameListenerRegistered("VIDEO_PIPELINE_STARTED:$caller")
    }

    /**
     * Unity calls this when it has (or re-has) a valid Surface from its SurfaceTexture.
     * Call it again after permission dialog / resume.
     */
    @JvmStatic
    fun setDecoderSurface(surface: Surface, width: Int, height: Int) {
        updateDecoderSurface(surface, width, height, "setDecoderSurface")
    }

    @JvmStatic
    fun startOrUpdate(surface: Surface, width: Int, height: Int) {
        updateDecoderSurface(surface, width, height, "startOrUpdate")
    }

    /**
     * Records DroneView's request for board frames. Registration itself is tied
     * to the actual DJI video lifecycle so reconnects cannot lose the listener.
     */
    @JvmStatic
    fun startBoardVision(): Boolean {
        logBoardVisionApiReadyOnce()
        boardVisionRequested = true
        return ensureBoardFrameListenerRegistered("UNITY_PROVIDER_REQUEST")
    }

    @JvmStatic
    fun stopBoardVision() {
        boardVisionRequested = false
        hasRegisteredBoardListener = false
        DjiBoardVisionBridge.stop(cameraStreamManager(), "PROVIDER_DISABLED")
        logBoardFrameListenerState("PROVIDER_DISABLED")
    }

    /**
     * Exact JNI contract consumed by Unity's DjiBoardVisionBridge wrapper.
     * It returns the existing board-result JSON transport unchanged.
     */
    @JvmStatic
    fun getLatestBoardVisionJson(): String {
        logBoardVisionApiReadyOnce()
        return DjiBoardVisionBridge.getLatestResultJson()
    }

    /** Requests a bounded capture session of raw detector-input frames. */
    @JvmStatic
    fun requestBoardCalibrationCapture(frameCount: Int): String =
        DjiBoardVisionBridge.requestCalibrationCapture(frameCount)

    private fun logBoardVisionApiReadyOnce() {
        if (boardVisionApiReadyLogged) return

        synchronized(this) {
            if (boardVisionApiReadyLogged) return
            boardVisionApiReadyLogged = true
            Log.i(TAG, "DJI_BOARD_BRIDGE_API_READY getLatestBoardVisionJson=true")
        }
    }

    internal fun onSdkReadyChanged(source: String) {
        val pending = pendingSurface
        if (pending == null) {
            Log.i(TAG, "SDK ready via $source, but no deferred surface is waiting")
        } else {
            Log.i(TAG, "SDK ready via $source; retrying deferred decoder surface")
            updateDecoderSurface(pending.surface, pending.width, pending.height, "deferredStart")
        }

        ensureBoardFrameListenerRegistered("SDK_READY:$source")
    }

    internal fun onProductConnectionChanged(connected: Boolean, source: String) {
        if (!connected) {
            DjiBoardVisionBridge.stop(cameraStreamManager(), "PRODUCT_DISCONNECTED")
            logBoardFrameListenerState("PRODUCT_DISCONNECTED:$source")
            return
        }

        ensureBoardFrameListenerRegistered("PRODUCT_CONNECTED:$source")
    }

    /** Called after asynchronous listener cleanup, so a reconnect cannot miss re-registration. */
    internal fun onBoardFrameListenerStopped(generation: Long, reason: String) {
        Log.i(TAG, "DJI_BOARD_LISTENER_REMOVED generation=$generation reason=$reason")
        logBoardFrameListenerState("LISTENER_STOPPED:$reason generation=$generation")
        ensureBoardFrameListenerRegistered("LISTENER_STOPPED:$reason")
    }

    @JvmStatic
    fun stopVideo() {
        Log.i(TAG, " - stopVideo called")
        pendingSurface = null
        videoPipelineStarted = false
        DjiBoardVisionBridge.stop(cameraStreamManager(), "VIDEO_STOP")
        logBoardFrameListenerState("VIDEO_STOP")
        controller.stop()
    }

    private fun ensureBoardFrameListenerRegistered(reason: String): Boolean {
        val productConnected = DJIPlugin.isVideoBridgeReady()
        val videoReady = videoPipelineStarted && controller.isStarted()

        if (!boardVisionRequested) {
            logBoardFrameListenerState("NOT_REQUESTED:$reason")
            return false
        }
        if (!productConnected || !videoReady) {
            Log.i(TAG, "DJI_BOARD_LISTENER_WAITING reason=${if (!productConnected) "PRODUCT_OR_SDK_NOT_READY" else "VIDEO_NOT_READY"}")
            logBoardFrameListenerState("WAITING:$reason")
            return false
        }
        if (DjiBoardVisionBridge.isListenerRegistered()) {
            logBoardFrameListenerState("ALREADY_REGISTERED:$reason")
            return true
        }

        val registered = DjiBoardVisionBridge.start(cameraStreamManager())
        val generation = DjiBoardVisionBridge.activeListenerGeneration()
        if (registered) {
            val event = if (hasRegisteredBoardListener) "DJI_BOARD_LISTENER_REREGISTERED" else "DJI_BOARD_LISTENER_REGISTERED"
            hasRegisteredBoardListener = true
            Log.i(TAG, "$event generation=$generation reason=$reason")
        }
        logBoardFrameListenerState("${if (registered) "REGISTERED" else "REGISTRATION_DEFERRED"}:$reason")
        return registered
    }

    private fun logBoardFrameListenerState(reason: String) {
        Log.i(
            TAG,
            "DJI_BOARD_FRAME_LISTENER_STATE requested=$boardVisionRequested " +
                "videoReady=${videoPipelineStarted && controller.isStarted()} " +
                "productConnected=${DJIPlugin.isProductConnected()} " +
                "registered=${DjiBoardVisionBridge.isListenerRegistered()} " +
                "generation=${DjiBoardVisionBridge.activeListenerGeneration()} " +
                "reason=$reason"
        )
    }
}




