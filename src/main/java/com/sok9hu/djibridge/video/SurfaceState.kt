package com.sok9hu.djibridge.video

import android.view.Surface

/* =========================
 * Surface state
 * ========================= */
class SurfaceState {
    @Volatile private var surface: Surface? = null
    @Volatile private var width: Int = 0
    @Volatile private var height: Int = 0

    fun update(surface: Surface, width: Int, height: Int) {
        this.surface = surface
        if (width > 0) this.width = width
        if (height > 0) this.height = height
    }

    fun snapshot(): SurfaceSnapshot = SurfaceSnapshot(surface, width, height)

    fun clear() {
        surface = null
        width = 0
        height = 0
    }
}

data class SurfaceSnapshot(
    val surface: Surface?,
    val width: Int,
    val height: Int
) {
    fun isValid(): Boolean = surface?.isValid == true
}