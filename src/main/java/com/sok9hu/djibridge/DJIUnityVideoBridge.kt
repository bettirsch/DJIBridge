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
            return
        }

        pendingSurface = null
        controller.startOrUpdate(surface, width, height)
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

    /** Starts the independent ImageReader CPU branch used by board vision. */
    @JvmStatic
    fun startBoardVision(): Boolean = DjiBoardVisionBridge.start(cameraStreamManager())

    @JvmStatic
    fun stopBoardVision() {
        DjiBoardVisionBridge.stop(cameraStreamManager())
    }

    @JvmStatic
    fun getLatestBoardVisionJson(): String = DjiBoardVisionBridge.getLatestResultJson()

    internal fun onSdkReadyChanged(source: String) {
        val pending = pendingSurface ?: run {
            Log.i(TAG, "SDK ready via $source, but no deferred surface is waiting")
            return
        }

        Log.i(TAG, "SDK ready via $source; retrying deferred decoder surface")
        updateDecoderSurface(pending.surface, pending.width, pending.height, "deferredStart")
    }

    @JvmStatic
    fun stopVideo() {
        Log.i(TAG, " - stopVideo called")
        pendingSurface = null
        DjiBoardVisionBridge.stop(cameraStreamManager())
        controller.stop()
    }
}




