package com.sok9hu.djibridge.video

import android.util.Log
import android.view.Surface
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.interfaces.ICameraStreamManager

/* =========================
 * Orchestrator (high-level)
 * ========================= */
class UnityVideoBridgeController(
    private val tag: String,
    private val streamManagerProvider: () -> ICameraStreamManager?
) {
    private val lock = Any()

    private var started = false
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

    fun stop() {
        val localClient: CameraStreamClient?
        val localExec: DecodeExecutor?
        val localPipe: DecoderPipeline?

        synchronized(lock) {
            if (!started) return
            started = false

            localClient = cameraClient
            localExec = decodeExecutor
            localPipe = pipeline

            cameraClient = null
            decodeExecutor = null
            pipeline = null
            surfaceState.clear()
        }

        // Stop receiving packets first (no more enqueues).
        localClient?.stop()

        // Drain/cleanup codec on decode thread then stop the thread.
        localExec?.shutdownSafely {
            localPipe?.release()
        }

        Log.i(tag, "Stopped")
    }
}