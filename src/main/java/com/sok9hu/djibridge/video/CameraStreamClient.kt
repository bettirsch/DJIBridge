package com.sok9hu.djibridge.video

import android.os.SystemClock
import com.sok9hu.djibridge.PipelineLog
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.camera.StreamInfo
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager.ReceiveStreamListener

class CameraStreamClient(
    private val tag: String,
    private val manager: ICameraStreamManager,
    private val component: ComponentIndexType,
    private val onPacket: (Packet) -> Unit
) {
    private val log = PipelineLog(tag)

    private var listener: ReceiveStreamListener? = null
    private var sessionId = 0
    private var startMs = 0L
    private var bytesIn = 0L
    private var framesIn = 0L

    fun start() {
        if (listener != null) return

        sessionId++
        startMs = SystemClock.elapsedRealtime()
        bytesIn = 0
        framesIn = 0

        log.i("CameraStreamClient.start component=$component session=$sessionId")

        val l = object : ReceiveStreamListener {
            override fun onReceiveStream(data: ByteArray, offset: Int, length: Int, info: StreamInfo) {
                bytesIn += length.toLong()
                framesIn++

                log.iEvery("rx_$sessionId", 1000L) {
                    val dtMs = (SystemClock.elapsedRealtime() - startMs).coerceAtLeast(1L)
                    val kbps = (bytesIn * 8.0) / dtMs // kilobits per ms (good enough for trend)
                    "RX session=$sessionId frames=$framesIn bytes=$bytesIn (~${"%.2f".format(kbps)} kb/ms) " +
                            "off=$offset len=$length ${safeInfo(info)}"
                }

                // Copy out bytes since DJI buffer can be reused.
                val frame = ByteArray(length)
                System.arraycopy(data, offset, frame, 0, length)

                val pkt = Packet.from(frame, info)
                onPacket(pkt)
            }
        }

        listener = l
        manager.addReceiveStreamListener(component, l)

        try {
            manager.setKeepAliveDecoding(true)
            log.i("setKeepAliveDecoding(true)")
        } catch (t: Throwable) {
            log.w("setKeepAliveDecoding(true) failed", t)
        }
    }

    fun stop() {
        val l = listener ?: return
        listener = null

        log.i("CameraStreamClient.stop component=$component session=$sessionId frames=$framesIn bytes=$bytesIn")

        try {
            manager.removeReceiveStreamListener(l)
        } catch (t: Throwable) {
            log.w("removeReceiveStreamListener failed", t)
        }
    }

    private fun safeInfo(info: StreamInfo): String {
        // Works across DJI SDK variants: tries common names, otherwise falls back.
        return try {
            val w = getAnyInt(info, "width", "getWidth")
            val h = getAnyInt(info, "height", "getHeight")
            val fps = getAnyInt(info, "fps", "frameRate", "getFps", "getFrameRate")
            val mime = getAnyStr(info, "mimeType", "getMimeType", "codec", "getCodec")
            val pts = getAnyLong(info, "pts", "ptsUs", "timeStamp", "timestamp", "getPts", "getPtsUs", "getTimeStamp")

            buildString {
                append("info{")
                if (w != null) append("w=$w ")
                if (h != null) append("h=$h ")
                if (fps != null) append("fps=$fps ")
                if (mime != null) append("mime=$mime ")
                if (pts != null) append("pts=$pts ")
                append("}")
            }
        } catch (_: Throwable) {
            "info{${info}}"
        }
    }

    private fun getAnyInt(target: Any, vararg names: String): Int? =
        getAnyNumber(target, *names)?.toInt()

    private fun getAnyLong(target: Any, vararg names: String): Long? =
        getAnyNumber(target, *names)?.toLong()

    private fun getAnyNumber(target: Any, vararg names: String): Number? {
        val cls = target.javaClass
        for (name in names) {
            // 1) field
            try {
                val f = cls.getDeclaredField(name)
                f.isAccessible = true
                val v = f.get(target)
                if (v is Number) return v
            } catch (_: Throwable) {}

            // 2) zero-arg method
            try {
                val m = cls.getMethod(name)
                val v = m.invoke(target)
                if (v is Number) return v
            } catch (_: Throwable) {}
        }
        return null
    }

    private fun getAnyStr(target: Any, vararg names: String): String? {
        val cls = target.javaClass
        for (name in names) {
            // 1) field
            try {
                val f = cls.getDeclaredField(name)
                f.isAccessible = true
                val v = f.get(target)
                if (v != null) return v.toString()
            } catch (_: Throwable) {}

            // 2) zero-arg method
            try {
                val m = cls.getMethod(name)
                val v = m.invoke(target)
                if (v != null) return v.toString()
            } catch (_: Throwable) {}
        }
        return null
    }
}
