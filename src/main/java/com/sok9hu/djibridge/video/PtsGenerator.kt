package com.sok9hu.djibridge.video

import kotlin.math.max

/**
 * Helper class that generates monotonically increasing PTS values.
 */
class PtsGenerator {
    var fpsHint: Int = 30
    private var lastPtsUs: Long = 0L

    fun reset() {
        lastPtsUs = 0L
    }

    fun monotonicUs(inPtsUs: Long): Long {
        val step = 1_000_000L / max(1, fpsHint)
        val pts = if (inPtsUs <= 0L) (lastPtsUs + step) else inPtsUs
        val fixed = if (pts <= lastPtsUs) lastPtsUs + step else pts
        lastPtsUs = fixed
        return fixed
    }
}