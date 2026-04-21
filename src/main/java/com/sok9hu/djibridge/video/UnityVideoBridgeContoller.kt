package com.sok9hu.djibridge.video

import android.util.Log
import android.view.Surface
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager.ScaleType

/**
 * High-level orchestrator that connects the stream client, decode thread,
 * and decoding pipeline.
 */
class UnityVideoBridgeController(
    private val tag: String,
    private val streamManagerProvider: () -> ICameraStreamManager?
) {
    private companion object {
        // Prefer DJI's direct Surface renderer. The byte-stream listener path can spin up an
        // internal H.264 encoder on some devices, flooding logcat with MediaCodec -38 noise.
        private const val USE_DIRECT_SURFACE_RENDERING = true
    }

    private val lock = Any()

    private var started = false
    private var directSurface: Surface? = null
    private var cameraClient: CameraStreamClient? = null
    private var decodeExecutor: DecodeExecutor? = null
    private var pipeline: DecoderPipeline? = null

    private val surfaceState = SurfaceState()

    fun startOrUpdate(surface: Surface, width: Int, height: Int) {
        val mgr = streamManagerProvider() ?: run {
            Log.e(tag, "CameraStreamManager is null (DJI SDK not ready?)")
            return
        }

        synchronized(lock) {
            surfaceState.update(surface, width, height)

            if (USE_DIRECT_SURFACE_RENDERING) {
                startOrUpdateDirectLocked(mgr, surface, width, height)
                return
            }

            if (!started) {
                started = true

                decodeExecutor = DecodeExecutor(threadName = "DJIUnityVideoDecode")
                pipeline = DecoderPipeline(tag, surfaceState)

                cameraClient = CameraStreamClient(
                    tag = tag,
                    manager = mgr,
                    component = ComponentIndexType.LEFT_OR_MAIN,
                    onPacket = { pkt ->
                        // Ensure all decoding happens on one thread.
                        decodeExecutor?.post {
                            pipeline?.onPacket(pkt)
                        }
                    }
                ).also { it.start() }

                Log.i(tag, "Started; waiting for stream...")
                return
            }

            // Already started: try to switch codec output surface safely on decode thread.
            decodeExecutor?.post {
                pipeline?.onSurfacePossiblyChanged()
            }
        }
    }

    private fun startOrUpdateDirectLocked(
        mgr: ICameraStreamManager,
        surface: Surface,
        width: Int,
        height: Int
    ) {
        val previousSurface = directSurface
        val surfaceChanged = previousSurface != null && previousSurface != surface

        if (!started) {
            started = true
            directSurface = surface
            putDirectSurface(mgr, surface, width, height, "start")
            return
        }

        if (surfaceChanged) {
            removeDirectSurface(mgr, previousSurface, "surfaceChanged")
        }

        directSurface = surface
        putDirectSurface(mgr, surface, width, height, "update")
    }

    fun stop() {
        val localClient: CameraStreamClient?
        val localExec: DecodeExecutor?
        val localPipe: DecoderPipeline?
        val localSurface: Surface?

        synchronized(lock) {
            if (!started) return
            started = false

            localSurface = directSurface
            localClient = cameraClient
            localExec = decodeExecutor
            localPipe = pipeline

            directSurface = null
            cameraClient = null
            decodeExecutor = null
            pipeline = null
            surfaceState.clear()
        }

        if (USE_DIRECT_SURFACE_RENDERING) {
            removeDirectSurface(streamManagerProvider(), localSurface, "stop")
            Log.i(tag, "Stopped")
            return
        }

        // Stop receiving packets first (no more enqueues).
        localClient?.stop()

        // Drain/cleanup codec on decode thread then stop the thread.
        localExec?.shutdownSafely {
            localPipe?.release()
        }

        Log.i(tag, "Stopped")
    }

    private fun putDirectSurface(
        mgr: ICameraStreamManager,
        surface: Surface,
        width: Int,
        height: Int,
        reason: String
    ) {
        try {
            mgr.putCameraStreamSurface(
                ComponentIndexType.LEFT_OR_MAIN,
                surface,
                width,
                height,
                ScaleType.FIX_XY
            )
            Log.i(tag, "Direct camera stream surface $reason ${width}x$height")
        } catch (t: Throwable) {
            Log.e(tag, "Direct camera stream surface $reason failed", t)
        }
    }

    private fun removeDirectSurface(
        mgr: ICameraStreamManager?,
        surface: Surface?,
        reason: String
    ) {
        if (mgr == null || surface == null) return

        try {
            mgr.removeCameraStreamSurface(surface)
            Log.i(tag, "Direct camera stream surface removed ($reason)")
        } catch (t: Throwable) {
            Log.w(tag, "Direct camera stream surface remove failed ($reason)", t)
        }
    }
}
