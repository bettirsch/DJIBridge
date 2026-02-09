package com.sok9hu.djibridge.video

import android.media.MediaFormat
import dji.v5.manager.interfaces.ICameraStreamManager.MimeType
import java.nio.ByteBuffer

/**
 * Codec-specific data (SPS/PPS/VPS) store and extractor from incoming packets.
 */
class CsdStore {
    // H264: csd-0=SPS, csd-1=PPS
    // H265: csd-0=VPS, csd-1=SPS, csd-2=PPS
    private var csd0: ByteArray? = null
    private var csd1: ByteArray? = null
    private var csd2: ByteArray? = null

    fun reset() {
        csd0 = null; csd1 = null; csd2 = null
    }

    fun isReadyFor(mime: String): Boolean {
        return if (mime == "video/avc") {
            csd0 != null && csd1 != null
        } else {
            csd0 != null && csd1 != null && csd2 != null
        }
    }

    fun applyTo(format: MediaFormat, mime: String) {
        when (mime) {
            "video/avc" -> {
                format.setByteBuffer("csd-0", ByteBuffer.wrap(requireNotNull(csd0)))
                format.setByteBuffer("csd-1", ByteBuffer.wrap(requireNotNull(csd1)))
            }
            "video/hevc" -> {
                format.setByteBuffer("csd-0", ByteBuffer.wrap(requireNotNull(csd0)))
                format.setByteBuffer("csd-1", ByteBuffer.wrap(requireNotNull(csd1)))
                format.setByteBuffer("csd-2", ByteBuffer.wrap(requireNotNull(csd2)))
            }
        }
    }

    fun updateFrom(pkt: Packet) {
        when (pkt.mimeEnum) {
            MimeType.H264 -> extractH264(pkt.bytes)
            MimeType.H265 -> extractH265(pkt.bytes)
            else -> Unit
        }
    }

    private fun extractH264(data: ByteArray) {
        NalUnitParser.forEachNal(data, isHevc = false) { nal, nalType ->
            when (nalType) {
                7 -> if (csd0 == null) csd0 = nal.withStartCode() // SPS
                8 -> if (csd1 == null) csd1 = nal.withStartCode() // PPS
            }
        }
    }

    private fun extractH265(data: ByteArray) {
        NalUnitParser.forEachNal(data, isHevc = true) { nal, nalType ->
            when (nalType) {
                32 -> if (csd0 == null) csd0 = nal.withStartCode() // VPS
                33 -> if (csd1 == null) csd1 = nal.withStartCode() // SPS
                34 -> if (csd2 == null) csd2 = nal.withStartCode() // PPS
            }
        }
    }

    private fun ByteArray.withStartCode(): ByteArray {
        val sc = byteArrayOf(0, 0, 0, 1)
        return ByteArray(sc.size + size).also {
            System.arraycopy(sc, 0, it, 0, sc.size)
            System.arraycopy(this, 0, it, sc.size, size)
        }
    }
}