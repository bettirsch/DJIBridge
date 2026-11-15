package com.sok9hu.djibridge

import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import dji.sdk.keyvalue.key.DJIFlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.XYZ
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import java.util.ArrayDeque

/** Opt-in observer. Values from get() never enter DJIPoseBridge state/history or localization. */
internal object DjiRelativePositionExperiment {
    private const val CAPACITY = 512
    private val handler by lazy { Handler(Looper.getMainLooper()) }
    private val flyingKey by lazy { KeyTools.createKey(DJIFlightControllerKey.KeyIsFlying) }
    private val connectionKey by lazy { KeyTools.createKey(DJIFlightControllerKey.KeyConnection) }
    private val relativeKey by lazy { KeyTools.createKey(DJIFlightControllerKey.KeyRelateToTakeOffPosition) }
    private val owner = Any()
    private val events = ArrayDeque<String>()
    @Volatile private var session: String? = null
    @Volatile private var generation = 0L
    private var requestSequence = 0L
    private var probe = RelativePositionProbe()
    private var pending: Pending? = null
    private var flightListener = false
    private var flightListenerGeneration = 0L
    private var flying: Boolean? = null
    private var flyingTimestamp = 0L
    private var lastCallback = 0L
    private var lastStatus = 0L
    private var drops = 0L
    private var bindGeneration = 0
    private var bindError: String? = null
    private var bindReturned = false
    private data class Pending(val id: Long, val generation: Long, val at: Long, val context: Map<String, Any?>,
                               var timeoutLogged: Boolean = false)
    private val ticker = object : Runnable {
        override fun run() {
            synchronized(this@DjiRelativePositionExperiment) {
                if (session == null) return
                runCatching { tick() }.onFailure { emit("DiagnosticError", mapOf("error" to it.toString())) }
                if (session != null) handler.postDelayed(this, 250L)
            }
        }
    }

    @Synchronized fun start(id: String): Boolean {
        if (id.isBlank()) return false
        if (session == id) return true
        stopInternal()
        events.clear(); drops = 0; session = id; generation++
        probe = RelativePositionProbe(); lastCallback = 0; lastStatus = 0
        flying = null; flyingTimestamp = 0
        emit("SessionStarted", mapOf("pollIntervalMs" to 1000, "connectedDelayMs" to 2000,
            "silentAirborneRebindDelayMs" to 8000, "diagnosticOnly" to true,
            "listenerRegistrationReturned" to bindReturned, "listenerBindError" to bindError,
            "outstandingEarlierRequest" to (pending != null)))
        handler.post(ticker)
        return true
    }

    @Synchronized fun drain(id: String): String {
        if (session != id) return ""
        val result = events.joinToString("\n")
        events.clear()
        return result
    }

    @Synchronized fun stop(id: String): String {
        if (session != id) return ""
        emit("SessionStopped", mapOf("queueDrops" to drops, "requestStillPending" to (pending != null)))
        val result = drain(id)
        stopInternal()
        return result
    }

    private fun stopInternal() {
        if (session == null) return
        session = null; generation++
        handler.removeCallbacks(ticker)
        cancelFlightListener()
        // MSDK exposes no cancel-get API. Keep one outstanding request globally
        // until it completes; its old-generation result cannot enter a later session.
    }

    @Synchronized fun listenerBind(returned: Boolean, error: String? = null) {
        bindReturned = returned; bindError = error
        if (returned) bindGeneration++
        emit("ListenerRegistration", mapOf("returnedWithoutException" to returned,
            "registrationAcknowledgedBySdk" to false, "error" to error))
    }

    @Synchronized fun lastBindReturned(): Boolean = bindReturned

    fun listenerValue(value: XYZ?, timestampNs: Long) {
        if (session == null) return
        val token = generation
        val xyz = RelativePositionDiagnosticJson.xyz(value?.x, value?.y, value?.z)
        val reference = DJIPoseBridge.relativePositionDiagnosticReference()
        // Never call KeyManager or wait for the experiment monitor on an SDK callback thread.
        handler.post {
            synchronized(this) {
                if (session == null || generation != token) return@post
                probe.callback()
                val interval = if (lastCallback > 0L) (timestampNs - lastCallback) / 1_000_000.0 else null
                lastCallback = timestampNs
                emit("ListenerCallback", context(flying) + mapOf("sourceTimestampNs" to timestampNs,
                    "processingTimestampNs" to SystemClock.elapsedRealtimeNanos(), "callbackIntervalMs" to interval,
                    "valuePresent" to (value != null), "xyz" to xyz, "reference" to reference), timestampNs)
            }
        }
    }

    private fun cancelFlightListener() {
        if (!flightListener) return
        runCatching { KeyManager.getInstance().cancelListen(flyingKey, owner) }
        flightListenerGeneration++
        flightListener = false; flying = null; flyingTimestamp = 0
    }

    private fun tick() {
        val now = SystemClock.elapsedRealtimeNanos()
        val connected = DJIPlugin.isVideoBridgeReady()
        if (connected && !flightListener) {
            val token = generation
            val flightToken = ++flightListenerGeneration
            flightListener = true
            try {
                KeyManager.getInstance().listen(flyingKey, owner) { _, value ->
                    val receivedAt = SystemClock.elapsedRealtimeNanos()
                    handler.post {
                        synchronized(this) {
                            if (session != null && generation == token && flightListenerGeneration == flightToken && flightListener) {
                                flying = value; flyingTimestamp = receivedAt
                                emit("FlightStateCallback", mapOf("flying" to value, "sourceTimestampNs" to receivedAt), receivedAt)
                            }
                        }
                    }
                }
            } catch (failure: Throwable) {
                cancelFlightListener()
                throw failure
            }
            emit("FlightListenerRegistered", mapOf("registrationAcknowledgedBySdk" to false))
        } else if (!connected) cancelFlightListener()
        val flight = flying ?: if (connected) KeyManager.getInstance().getValue(flyingKey) else null
        val action = probe.tick(now, connected, flight, pending != null)
        if (action.rebind) {
            emit("ListenerRebindRequested", context(flight))
            val result = runCatching { DJIPoseBridge.rebindRelativePositionListenerForExperiment() }
            emit("ListenerRebindReturned", mapOf("returnedWithoutException" to (result.getOrNull() == true),
                "registrationAcknowledgedBySdk" to false, "error" to result.exceptionOrNull()?.toString()))
        }
        if (now - lastStatus >= 1_000_000_000L) {
            lastStatus = now
            emit("Status", context(flight) + mapOf("queueDrops" to drops, "requestPending" to (pending != null),
                "reference" to DJIPoseBridge.relativePositionDiagnosticReference()))
        }
        pending?.let {
            if (!it.timeoutLogged && now - it.at >= 5_000_000_000L) {
                it.timeoutLogged = true
                emit("GetPendingTimeout", mapOf("requestId" to it.id, "retryBlockedUntilCompletion" to true))
            }
        }
        action.reason?.let { request(it, now, flight) }
    }

    private fun context(flight: Boolean?): Map<String, Any?> = linkedMapOf(
        "sdkRegistered" to DJIPlugin.isSdkRegistered(), "productConnected" to DJIPlugin.isProductConnected(),
        "flightControllerConnected" to runCatching { KeyManager.getInstance().getValue(connectionKey) }.getOrNull(),
        "flying" to flight, "flightStateTimestampNs" to flyingTimestamp.takeIf { it > 0 },
        "flightStateProvenance" to if (flyingTimestamp > 0) "KEY_CALLBACK" else "CACHED_OR_UNAVAILABLE",
        "sdkReady" to DJIPlugin.isVideoBridgeReady(), "connectionGeneration" to probe.connectionGeneration)

    private fun request(reason: String, now: Long, flight: Boolean?) {
        val request = Pending(++requestSequence, generation, now, context(flight) + mapOf("reason" to reason))
        pending = request
        emit("GetRequested", request.context + mapOf("requestId" to request.id, "requestTimestampNs" to now,
            "reference" to DJIPoseBridge.relativePositionDiagnosticReference()), now)
        runCatching {
            KeyManager.getInstance().getValue(relativeKey, object : CommonCallbacks.CompletionCallbackWithParam<XYZ> {
                override fun onSuccess(value: XYZ?) { postResult(request, value, null, null) }
                override fun onFailure(error: IDJIError) { postResult(request, null, error, null) }
            })
        }.onFailure { postResult(request, null, null, it.toString()) }
    }

    private fun postResult(request: Pending, value: XYZ?, error: IDJIError?, exception: String?) {
        val receivedAt = SystemClock.elapsedRealtimeNanos()
        val result = mapOf("result" to when {
            error != null -> "SDK_ERROR"; exception != null -> "EXCEPTION"; value == null -> "NULL_VALUE"; else -> "VALUE"
        }, "xyz" to RelativePositionDiagnosticJson.xyz(value?.x, value?.y, value?.z),
            "errorCode" to error?.errorCode(), "errorDescription" to error?.description(),
            "errorType" to error?.errorType()?.toString(), "innerCode" to error?.innerCode(),
            "errorHint" to error?.hint(), "exception" to exception,
            "reference" to DJIPoseBridge.relativePositionDiagnosticReference())
        handler.post { complete(request, result, receivedAt) }
    }

    @Synchronized private fun complete(request: Pending, result: Map<String, Any?>, receivedAt: Long) {
        if (pending !== request) return
        pending = null
        if (session == null || generation != request.generation) return
        emit("GetResult", request.context + mapOf("requestId" to request.id, "requestTimestampNs" to request.at,
            "resultTimestampNs" to receivedAt, "roundTripMs" to (receivedAt - request.at) / 1_000_000.0,
            "processingTimestampNs" to SystemClock.elapsedRealtimeNanos(),
            "requestConnectionGeneration" to request.context["connectionGeneration"],
            "currentConnectionGeneration" to probe.connectionGeneration,
            "currentConnectionStateAtProcessing" to context(flying)) + result, receivedAt)
    }

    private fun emit(event: String, data: Map<String, Any?>, timestampNs: Long = SystemClock.elapsedRealtimeNanos()) {
        val id = session ?: return
        if (events.size >= CAPACITY) { drops++; return }
        events.addLast(RelativePositionDiagnosticJson.encode(linkedMapOf(
            "recordType" to "RelativePositionExperiment", "event" to event, "telemetrySessionId" to id,
            "monotonicTimestampNs" to timestampNs, "clockDomain" to "ANDROID_MONOTONIC_elapsedRealtimeNanos",
            "timestampMeaning" to "ANDROID_OBSERVATION_ARRIVAL_NOT_FC_MEASUREMENT_TIME",
            "fcMeasurementTimestampAvailable" to false,
            "listenerGeneration" to bindGeneration, "appliedToLocalization" to false) + data))
    }
}
