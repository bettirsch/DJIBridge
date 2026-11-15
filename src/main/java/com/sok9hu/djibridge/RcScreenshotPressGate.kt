package com.sok9hu.djibridge

internal class RcScreenshotPressGate {
    enum class Key { SHUTTER, RECORD }
    enum class Decision { INITIAL, DUPLICATE, RELEASE, OTHER_KEY_HELD, COALESCED, PRESS }

    private val states = arrayOfNulls<Boolean>(Key.entries.size)
    private val held = BooleanArray(Key.entries.size)
    private var lastPressNs: Long? = null
    private var lastReleaseNs: Long? = null
    private var lastObservationNs = Long.MIN_VALUE

    fun observe(key: Key, down: Boolean, nowNs: Long): Decision {
        if (nowNs < lastObservationNs) return Decision.DUPLICATE
        lastObservationNs = nowNs
        val previous = states[key.ordinal]
        states[key.ordinal] = down
        if (previous == null) return Decision.INITIAL
        if (previous == down) return Decision.DUPLICATE
        if (!down) {
            val wasHeld = held[key.ordinal]
            held[key.ordinal] = false
            if (wasHeld && held.none { it }) lastReleaseNs = nowNs
            return Decision.RELEASE
        }
        val otherHeld = held.any { it }
        held[key.ordinal] = true
        if (otherHeld) return Decision.OTHER_KEY_HELD
        // Two DJI keys can report the same physical actuation at slightly different times.
        if (lastPressNs?.let { nowNs - it < COALESCE_NS } == true ||
            lastReleaseNs?.let { nowNs - it < RELEASE_COALESCE_NS } == true
        ) return Decision.COALESCED
        lastPressNs = nowNs
        return Decision.PRESS
    }

    companion object {
        private const val COALESCE_NS = 400_000_000L
        private const val RELEASE_COALESCE_NS = 180_000_000L
    }
}
