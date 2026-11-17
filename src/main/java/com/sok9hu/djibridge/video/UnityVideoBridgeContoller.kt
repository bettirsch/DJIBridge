package com.sok9hu.djibridge.video

import android.util.Log
import android.view.Surface
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager.ScaleType

/** Manages the DJI SDK's direct camera stream Surface. */
class UnityVideoBridgeController(
    private val tag: String,
    private val streamManagerProvider: () -> ICameraStreamManager?
) {
    private val lock = Any()

    private var started = false
    private var directSurface: Surface? = null

    fun isStarted(): Boolean = synchronized(lock) { started }

    fun startOrUpdate(surface: Surface, width: Int, height: Int) {
        val mgr = streamManagerProvider() ?: run {
            Log.e(tag, "CameraStreamManager is null (DJI SDK not ready?)")
            return
        }

        synchronized(lock) {
            startOrUpdateDirectLocked(mgr, surface, width, height)
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
        val localSurface: Surface?

        synchronized(lock) {
            if (!started) return
            started = false

            localSurface = directSurface
            directSurface = null
        }

        removeDirectSurface(streamManagerProvider(), localSurface, "stop")
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
