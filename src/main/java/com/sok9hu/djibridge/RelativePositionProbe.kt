package com.sok9hu.djibridge

/** Diagnostic scheduling only. No position estimate, axis convention, or flight command. */
internal class RelativePositionProbe {
    data class Action(val reason: String? = null, val rebind: Boolean = false)
    private var wasConnected = false
    private var connectedAt = 0L
    private var nextRequest = 0L
    private var airborneAt: Long? = null
    private var connectedRequest = false
    private var airborneRequest = false
    private var rebound = false
    var callbacks = 0; private set
    var connectionGeneration = 0; private set

    fun callback() { callbacks++ }

    fun tick(now: Long, connected: Boolean, flying: Boolean?, requestPending: Boolean): Action {
        if (!connected) { wasConnected = false; return Action() }
        if (!wasConnected) {
            wasConnected = true; connectionGeneration++; connectedAt = now
            nextRequest = now + 2_000_000_000L
            airborneAt = null; connectedRequest = false; airborneRequest = false
            rebound = false
        }
        if (flying == false) { airborneAt = null; airborneRequest = false }
        if (flying == true && airborneAt == null) airborneAt = now
        val rebind = flying == true && !rebound && callbacks == 0 && airborneAt != null &&
            now - airborneAt!! >= 8_000_000_000L && now - connectedAt >= 2_000_000_000L
        if (rebind) rebound = true
        if (requestPending || now < nextRequest) return Action(rebind = rebind)
        nextRequest = now + 1_000_000_000L
        val reason = when {
            !connectedRequest -> { connectedRequest = true; "CONNECTED_DELAYED_GET" }
            flying == true && !airborneRequest -> { airborneRequest = true; "AIRBORNE_GET" }
            rebind -> "POST_REBIND_GET"
            else -> "PERIODIC_GET"
        }
        return Action(reason, rebind)
    }
}

/** JSON encoding is shared by Android acquisition and host-side serialization tests. */
internal object RelativePositionDiagnosticJson {
    fun encode(value: Any?): String = when (value) {
        null -> "null"
        is String -> buildString {
            append('"')
            value.forEach { c -> when (c) {
                '"' -> append("\\\"")
                '\\' -> append("\\\\")
                '\n' -> append("\\n")
                '\r' -> append("\\r")
                '\t' -> append("\\t")
                else -> if (c.code < 32) append("\\u%04x".format(c.code)) else append(c)
            } }
            append('"')
        }
        is Double -> if (value.isFinite()) value.toString() else "null"
        is Float -> if (value.isFinite()) value.toString() else "null"
        is Number, is Boolean -> value.toString()
        is Map<*, *> -> value.entries.joinToString(",", "{", "}") { encode(it.key.toString()) + ":" + encode(it.value) }
        is Iterable<*> -> value.joinToString(",", "[") { encode(it) }
        else -> error("Unsupported diagnostic JSON type: ${value.javaClass.name}")
    }

    fun xyz(x: Double?, y: Double?, z: Double?): Map<String, Any?> = linkedMapOf(
        "x" to x, "y" to y, "z" to z,
        "valid" to (x != null && y != null && z != null && x.isFinite() && y.isFinite() && z.isFinite()),
        "axesUnitsOrigin" to "UNVALIDATED")
}
