package com.sok9hu.djibridge

import android.os.SystemClock
import android.util.Log
import java.util.concurrent.ConcurrentHashMap

/**
 * Lightweight logging wrapper that supports throttling repeated messages
 * by key.
 */
internal class PipelineLog(private val tag: String) {
    private val lastAt = ConcurrentHashMap<String, Long>()
    private val onceSet = ConcurrentHashMap.newKeySet<String>()

    fun d(msg: String) = Log.d(tag, msg)
    fun i(msg: String) = Log.i(tag, msg)
    fun w(msg: String, t: Throwable? = null) = if (t != null) Log.w(tag, msg, t) else Log.w(tag, msg)
    fun e(msg: String, t: Throwable? = null) = if (t != null) Log.e(tag, msg, t) else Log.e(tag, msg)

    /** Log at most once per [everyMs] per [key]. */
    fun iEvery(key: String, everyMs: Long, msg: () -> String) {
        val now = SystemClock.elapsedRealtime()
        val prev = lastAt[key] ?: 0L
        if (now - prev >= everyMs) {
            lastAt[key] = now
            Log.i(tag, msg())
        }
    }

    /** Log at most once per [key] (ever). */
    fun iOnce(key: String, msg: () -> String) {
        if (onceSet.add(key)) {
            Log.i(tag, msg())
        }
    }
}
