package com.sok9hu.djibridge.video

import android.os.Handler
import android.os.HandlerThread
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * Helper that runs decoding on a single dedicated thread and shuts it down safely.
 */
class DecodeExecutor(threadName: String) {
    private val thread = HandlerThread(threadName).also { it.start() }
    private val handler = Handler(thread.looper)

    fun post(block: () -> Unit) {
        handler.post(block)
    }

    fun shutdownSafely(onDecodeThreadCleanup: () -> Unit) {
        val latch = CountDownLatch(1)
        handler.post {
            try {
                onDecodeThreadCleanup()
            } finally {
                latch.countDown()
            }
        }

        // Best effort; do not block too long on app shutdown paths.
        try {
            latch.await(250, TimeUnit.MILLISECONDS)
        } catch (_: InterruptedException) {
        }

        thread.quitSafely()
    }
}
