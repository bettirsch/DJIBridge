package com.sok9hu.djibridge

/** JNI boundary for the DJI-only AprilTag board localizer. */
internal object DjiBoardVisionNative {
    init {
        System.loadLibrary("djiunity")
    }

    external fun detectBoardLuma(
        luma: ByteArray,
        width: Int,
        height: Int,
        fx: Float,
        fy: Float,
        cx: Float,
        cy: Float,
        markerLayout: FloatArray
    ): FloatArray?

    external fun releaseDetector()
}
