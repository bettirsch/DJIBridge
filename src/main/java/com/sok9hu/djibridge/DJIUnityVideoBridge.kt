package com.sok9hu.djibridge

import android.util.Log
import android.view.Surface
import com.sok9hu.djibridge.video.UnityVideoBridgeController
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager

/**
 * Public Unity-facing API (kept as an object for easy JNI/Unity calls),
 * internally delegating responsibilities to smaller classes.
 *
 * You can split these classes into separate files later:
 * - DJIUnityVideoBridge (API)
 * - UnityVideoBridgeController (orchestrator)
 * - CameraStreamClient
 * - DecodeExecutor
 * - DecoderPipeline (codec + pts + csd)
 * - CsdStore + NalUnitParser
 * - PtsGenerator
 */
object DJIUnityVideoBridge {

    private const val TAG = "DJIUnityVideoBridge"

    private val controller by lazy { UnityVideoBridgeController(TAG, ::cameraStreamManager) }

    private fun cameraStreamManager(): ICameraStreamManager? =
        MediaDataCenter.getInstance().cameraStreamManager

    /**
     * Unity calls this when it has (or re-has) a valid Surface from its SurfaceTexture.
     * Call it again after permission dialog / resume.
     */
    @JvmStatic
    fun setDecoderSurface(surface: Surface, width: Int, height: Int) {
        Log.i(TAG, " - SetDecoderSurface called")
        controller.startOrUpdate(surface, width, height)
    }

    @JvmStatic
    fun startOrUpdate(surface: Surface, width: Int, height: Int) {
        Log.i(TAG, " - StartOrUpdate called")
        controller.startOrUpdate(surface, width, height)
    }

    @JvmStatic
    fun stopVideo() {
        Log.i(TAG, " - stopVideo called")
        controller.stop()
    }
}




