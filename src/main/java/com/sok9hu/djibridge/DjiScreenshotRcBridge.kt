package com.sok9hu.djibridge

import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIRemoteControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.KeyManager
import org.json.JSONObject

/** Read-only RC observer. No calls to the aircraft camera or button-binding actions. */
object DjiScreenshotRcBridge {
    private const val TAG = "DjiScreenshot"
    private const val MAX_PENDING_AGE_NS = 1_000_000_000L
    private val main by lazy { Handler(Looper.getMainLooper()) }
    private val lock = Any()
    private val cameraModeKey by lazy { KeyTools.createKey(CameraKey.KeyCameraMode, ComponentIndexType.LEFT_OR_MAIN) }
    private var connectionOwner: Any? = null // Android main thread only
    private var nextGeneration = 0L
    @Volatile private var requested = false
    private var requestEpoch = 0L
    private var binding: Binding? = null
    private var pending: Press? = null

    private class Binding(val generation: Long, val epoch: Long) {
        val gate = RcScreenshotPressGate()
        var mode = "UNKNOWN"
    }

    private data class Press(val key: String, val timestampNs: Long, val mode: String, val generation: Long)

    @JvmStatic
    fun setEnabled(enabled: Boolean) {
        synchronized(lock) {
            if (requested == enabled) return
            requested = enabled
            requestEpoch++
            pending = null
        }
        main.post {
            if (!enabled) detach("unity_disabled")
            reconcile("unity")
        }
    }

    internal fun onSdkStateChanged() {
        if (Looper.myLooper() == Looper.getMainLooper()) reconcile("sdk")
        else main.post { reconcile("sdk") }
    }

    @JvmStatic
    fun getCameraMode(): String {
        if (!DJIPlugin.isVideoBridgeReady()) return "UNKNOWN"
        return runCatching { KeyManager.getInstance().getValue(cameraModeKey)?.name }.getOrNull() ?: "UNKNOWN"
    }

    @JvmStatic
    fun consumePressJson(): String {
        val event = synchronized(lock) {
            val value = pending
            pending = null
            if (!requested || value?.generation != binding?.generation || !DJIPlugin.isVideoBridgeReady()) null
            else value
        } ?: return ""
        val age = SystemClock.elapsedRealtimeNanos() - event.timestampNs
        if (age !in 0..MAX_PENDING_AGE_NS) {
            Log.i(TAG, "RC_SCREENSHOT_DISCARDED key=${event.key} timestampNs=${event.timestampNs} reason=STALE_REQUEST")
            return ""
        }
        return JSONObject().put("key", event.key).put("down", true)
            .put("timestampNs", event.timestampNs).put("cameraMode", event.mode)
            .put("generation", event.generation).toString()
    }

    private fun reconcile(source: String) {
        if (!requested || !DJIPlugin.isVideoBridgeReady()) {
            detach(source)
            return
        }
        if (connectionOwner != null) return
        val owner = Any()
        connectionOwner = owner
        val manager = KeyManager.getInstance()
        val key = KeyTools.createKey(DJIRemoteControllerKey.KeyConnection)
        try {
            manager.listen(key, owner, true) { _, connected ->
                if (connected == null) return@listen
                main.post {
                    if (connectionOwner === owner && requested && DJIPlugin.isVideoBridgeReady()) {
                        if (connected) bindButtons("rc_connected") else unbindButtons("rc_disconnected")
                    }
                }
            }
            // Some combinations do not supply RC connection state. Product
            // readiness is a fallback; a known disconnection still disarms it.
            val connected = manager.getValue(key)
            if (connected != false) bindButtons(source)
            else Log.i(TAG, "RC_SCREENSHOT_LISTEN awaiting=rc_connection")
        } catch (e: Exception) {
            connectionOwner = null
            cancel(owner)
            Log.w(TAG, "RC_SCREENSHOT_LISTEN error=connection_listener type=${e.javaClass.simpleName}")
        }
    }

    private fun bindButtons(source: String) {
        val session = synchronized(lock) {
            if (binding != null || !requested) return
            Binding(++nextGeneration, requestEpoch).also { binding = it }
        }
        val manager = KeyManager.getInstance()
        fun observe(info: dji.sdk.keyvalue.key.DJIKeyInfo<Boolean>, id: RcScreenshotPressGate.Key, name: String) {
            try {
                manager.listen(KeyTools.createKey(info), session, true) { _, down ->
                    if (down != null) onButton(session, id, name, down, SystemClock.elapsedRealtimeNanos())
                }
                Log.i(TAG, "RC_SCREENSHOT_KEY_LISTEN generation=${session.generation} key=$name state=REGISTERED_VALUE_NOT_PROVEN")
            } catch (e: Exception) {
                Log.w(TAG, "RC_SCREENSHOT_KEY_LISTEN generation=${session.generation} key=$name error=${e.javaClass.simpleName}")
            }
        }
        observe(DJIRemoteControllerKey.KeyShutterButtonDown, RcScreenshotPressGate.Key.SHUTTER, "KeyShutterButtonDown")
        observe(DJIRemoteControllerKey.KeyRecordButtonDown, RcScreenshotPressGate.Key.RECORD, "KeyRecordButtonDown")
        try {
            manager.listen(cameraModeKey, session, true) { _, mode ->
                synchronized(lock) { if (binding === session) session.mode = mode?.name ?: "UNKNOWN" }
            }
        } catch (e: Exception) {
            Log.w(TAG, "RC_SCREENSHOT_MODE_LISTEN error=${e.javaClass.simpleName}")
        }
        Log.i(TAG, "RC_SCREENSHOT_LISTEN generation=${session.generation} state=BOUND source=$source")
    }

    private fun onButton(session: Binding, id: RcScreenshotPressGate.Key, key: String, down: Boolean, now: Long) {
        val line = synchronized(lock) {
            if (binding !== session || !requested || session.epoch != requestEpoch || !DJIPlugin.isVideoBridgeReady()) return
            val decision = session.gate.observe(id, down, now)
            if (decision == RcScreenshotPressGate.Decision.DUPLICATE) return
            if (pending?.let { now - it.timestampNs > MAX_PENDING_AGE_NS } == true) pending = null
            var status = decision.name
            if (decision == RcScreenshotPressGate.Decision.PRESS) {
                if (pending != null) status = "REQUEST_SLOT_BUSY"
                else pending = Press(key, now, session.mode, session.generation)
            }
            "RC_SCREENSHOT_BUTTON generation=${session.generation} key=$key down=$down cameraMode=${session.mode} timestampNs=$now decision=$status"
        }
        Log.i(TAG, line)
    }

    private fun unbindButtons(source: String) {
        val old = synchronized(lock) {
            pending = null
            binding.also { binding = null }
        } ?: return
        cancel(old)
        Log.i(TAG, "RC_SCREENSHOT_LISTEN generation=${old.generation} state=UNBOUND source=$source")
    }

    private fun detach(source: String) {
        val oldOwner = connectionOwner
        connectionOwner = null
        unbindButtons(source)
        if (oldOwner != null) cancel(oldOwner)
    }

    private fun cancel(owner: Any) {
        runCatching { KeyManager.getInstance().cancelListen(owner) }
            .onFailure { Log.w(TAG, "RC_SCREENSHOT_UNBIND error=${it.javaClass.simpleName}") }
    }
}
