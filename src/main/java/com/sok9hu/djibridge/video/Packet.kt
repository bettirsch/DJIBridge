package com.sok9hu.djibridge.video

import dji.v5.manager.datacenter.camera.StreamInfo
import dji.v5.manager.interfaces.ICameraStreamManager.MimeType


/* =========================
 * Packet mapping + model
 * ========================= */
data class Packet(
    val bytes: ByteArray,
    val ptsUs: Long,
    val mimeEnum: MimeType?,
    val width: Int,
    val height: Int,
    val frameRate: Int
) {
    companion object {
        fun from(frame: ByteArray, info: StreamInfo): Packet {
            // StreamInfo can be 0 during transitions; choose safe fallbacks.
            val w = if (info.width > 0) info.width else 0
            val h = if (info.height > 0) info.height else 0
            val fr = if (info.frameRate > 0) info.frameRate else 30
            val ptsUs = if (info.presentationTimeMs > 0) info.presentationTimeMs * 1000L else 0L

            return Packet(
                bytes = frame,
                ptsUs = ptsUs,
                mimeEnum = info.mimeType,
                width = w,
                height = h,
                frameRate = fr
            )
        }
    }
}
