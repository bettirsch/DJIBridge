package com.sok9hu.djibridge.video

import android.media.MediaCodec
import android.media.MediaFormat
import android.view.Surface
import com.sok9hu.djibridge.PipelineLog
import dji.v5.manager.interfaces.ICameraStreamManager.MimeType

/* =========================
 * Codec controller (fixed render logic + minSdk-friendly flags + avoids return-type inference issues)
 * ========================= */
class CodecController(
    private val tag: String,
    private val surfaceState: SurfaceState
) {
    private val log = PipelineLog(tag)

    private companion object {
        private const val INPUT_TIMEOUT_US = 10_000L
        private const val MIN_MAX_INPUT_SIZE = 4 * 1024 * 1024

        // API 34 added MediaCodec.BUFFER_FLAG_DECODE_ONLY (= 0x20). Inline for minSdk < 34.
        private const val BUFFER_FLAG_DECODE_ONLY = 0x20
    }

    private val lock = Any()

    private var codec: MediaCodec? = null
    private val outInfo = MediaCodec.BufferInfo()

    private var currentMime: String? = null
    private var currentW: Int = 0
    private var currentH: Int = 0
    private var currentMaxInputSize: Int = 0
    private var lastSurface: Surface? = null

    // True while the current codec instance is configured to render to a Surface.
    // IMPORTANT: do NOT flip this per-frame.
    private var surfaceDecodeMode = false

    // Some vendors output non-renderable buffers early; we flip true on INFO_OUTPUT_FORMAT_CHANGED.
    private var outputFormatReady = false

    fun ensureConfigured(pkt: Packet, csdStore: CsdStore, ptsGenerator: PtsGenerator) {
        synchronized(lock) {
            val snap = surfaceState.snapshot()
            if (!snap.isValid()) {
                log.iOnce("surface_invalid_ensure") { "ensureConfigured: surface invalid -> release codec" }
                releaseLocked()
                return
            }

            val mime = pkt.mimeEnum.toCodecMime()
            ptsGenerator.fpsHint = pkt.frameRate

            // Generous max input size (floor + headroom + never shrink)
            val desiredMaxInput = maxOf(
                MIN_MAX_INPUT_SIZE,
                pkt.bytes.size * 3,
                currentMaxInputSize
            )

            val paramsChanged = currentMime != null &&
                    (currentMime != mime || currentW != pkt.width || currentH != pkt.height)

            val needsRecreate =
                codec == null || paramsChanged || desiredMaxInput > currentMaxInputSize

            if (!needsRecreate) return

            val reason = when {
                codec == null -> "codec==null"
                paramsChanged ->
                    "streamParamsChanged old=$currentMime ${currentW}x${currentH} -> new=$mime ${pkt.width}x${pkt.height}"
                desiredMaxInput > currentMaxInputSize ->
                    "maxInputGrow old=$currentMaxInputSize -> new=$desiredMaxInput"
                else -> "unknown"
            }
            log.i("ensureConfigured: recreate needed ($reason)")

            if (paramsChanged) {
                log.i("Stream params changed -> reset CSD store")
                csdStore.reset()
            }

            if (!csdStore.isReadyFor(mime)) {
                log.iEvery("wait_csd_$mime", 1500L) { "Waiting for CSD ($mime) before creating codec..." }
                return
            }

            releaseLocked()

            var created: MediaCodec? = null
            try {
                val surface = snap.surface
                log.i(
                    "Creating codec mime=$mime size=${pkt.width}x${pkt.height} maxIn=$desiredMaxInput fps=${pkt.frameRate} " +
                            "surface=${surface.idStr()} valid=${surface?.isValid}"
                )

                created = MediaCodec.createDecoderByType(mime)

                val format = MediaFormat.createVideoFormat(mime, pkt.width, pkt.height).apply {
                    setInteger(MediaFormat.KEY_MAX_INPUT_SIZE, desiredMaxInput)
                    if (pkt.frameRate > 0) setInteger(MediaFormat.KEY_FRAME_RATE, pkt.frameRate)
                    csdStore.applyTo(this, mime)
                }

                if (surface == null || !surface.isValid) {
                    throw IllegalStateException("Surface became invalid before configure()")
                }

                created.configure(format, surface, null, 0)
                created.start()

                codec = created
                currentMime = mime
                currentW = pkt.width
                currentH = pkt.height
                currentMaxInputSize = desiredMaxInput
                lastSurface = surface

                surfaceDecodeMode = true
                outputFormatReady = false // becomes true on INFO_OUTPUT_FORMAT_CHANGED

                ptsGenerator.reset()

                log.i("Codec started OK ($mime ${currentW}x${currentH}) surface=${lastSurface.idStr()} name=${created.name}")
            } catch (t: Throwable) {
                log.w("Failed to create/configure codec -> reset codec", t)
                try { created?.release() } catch (_: Throwable) {}
                releaseLocked()
            }
        }
    }

    fun onSurfacePossiblyChanged() {
        synchronized(lock) {
            val c = codec ?: run {
                log.d("onSurfacePossiblyChanged: codec==null (ignore)")
                return
            }

            val snap = surfaceState.snapshot()
            val newSurface = snap.surface

            if (newSurface == null || !newSurface.isValid) {
                log.i("Surface changed -> new surface invalid/null -> release codec")
                releaseLocked()
                return
            }

            val old = lastSurface
            if (old === newSurface) {
                log.d("Surface possibly changed, but Surface object identical (no-op)")
                return
            }

            try {
                log.i("Surface changed -> switching codec output surface old=${old.idStr()} new=${newSurface.idStr()}")
                c.setOutputSurface(newSurface) // minSdk>=24 => API 23+ always available
                lastSurface = newSurface
                surfaceDecodeMode = true
                log.i("setOutputSurface OK new=${lastSurface.idStr()}")
            } catch (e: Throwable) {
                log.w("setOutputSurface failed -> recreate codec", e)
                releaseLocked()
            }
        }
    }

    fun feedAndDrain(pkt: Packet, ptsGenerator: PtsGenerator) {
        synchronized(lock) {
            val c = codec ?: return

            // 1) Queue input
            try {
                val inIndex = c.dequeueInputBuffer(INPUT_TIMEOUT_US)
                if (inIndex >= 0) {
                    val pts = ptsGenerator.monotonicUs(pkt.ptsUs)
                    val buf = c.getInputBuffer(inIndex)
                    if (buf == null) {
                        log.w("getInputBuffer($inIndex) returned null")
                        return
                    }

                    buf.clear()
                    if (pkt.bytes.size > buf.capacity()) {
                        log.w("Frame too large: frame=${pkt.bytes.size} cap=${buf.capacity()} -> force recreate")
                        currentMaxInputSize = 0
                        releaseLocked()
                        return
                    }

                    buf.put(pkt.bytes)
                    c.queueInputBuffer(inIndex, 0, pkt.bytes.size, pts, 0)
                } else {
                    log.iEvery("no_input_buf", 2000L) { "No input buffer available (dequeue=$inIndex)" }
                }
            } catch (e: Throwable) {
                log.w("Input queue failed -> reset codec", e)
                releaseLocked()
                return
            }

            // 2) Drain output
            var rendered = 0
            var dropped = 0
            var skipped = 0

            while (true) {
                val outIndex = try {
                    c.dequeueOutputBuffer(outInfo, 0)
                } catch (e: Throwable) {
                    log.w("dequeueOutputBuffer failed -> reset codec", e)
                    releaseLocked()
                    return
                }

                when {
                    outIndex >= 0 -> {
                        val flags = outInfo.flags
                        val isConfig = (flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0
                        val isEos = (flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0
                        val isDecodeOnly = (flags and BUFFER_FLAG_DECODE_ONLY) != 0

                        val surfaceValid = surfaceState.snapshot().isValid()

                        // Render ONLY if we're in Surface decode mode AND buffer is an actual frame.
                        // This avoids vendor "rendering output error -38" caused by rendering non-video buffers.
                        val render = surfaceDecodeMode &&
                                surfaceValid &&
                                outputFormatReady &&
                                !isConfig && !isEos && !isDecodeOnly

                        if (!render) {
                            skipped++
                            log.iEvery("skip_render", 1000L) {
                                "Skip render: flags=0x${flags.toString(16)} ptsUs=${outInfo.presentationTimeUs} " +
                                        "surfaceValid=$surfaceValid formatReady=$outputFormatReady surfaceMode=$surfaceDecodeMode"
                            }
                        }

                        try {
                            c.releaseOutputBuffer(outIndex, render)
                            if (render) rendered++ else dropped++
                        } catch (e: Throwable) {
                            log.w("releaseOutputBuffer(render=$render) failed -> reset codec", e)
                            releaseLocked()
                            return
                        }
                    }

                    outIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                        outputFormatReady = true
                        log.i("Output format changed: ${c.outputFormat}")
                    }

                    outIndex == MediaCodec.INFO_TRY_AGAIN_LATER -> {
                        if (rendered + dropped + skipped > 0) {
                            log.iEvery("drain_stats", 1000L) {
                                "Drain stats: rendered=$rendered dropped=$dropped skipped=$skipped"
                            }
                        }
                        return
                    }

                    else -> return
                }
            }
        }
    }

    fun release() {
        synchronized(lock) {
            releaseLocked()
        }
    }

    private fun releaseLocked() {
        val c = codec ?: run {
            // Reset state even if codec already null
            currentMime = null
            currentMaxInputSize = 0
            lastSurface = null
            currentW = 0
            currentH = 0
            surfaceDecodeMode = false
            outputFormatReady = false
            return
        }
        codec = null

        log.i(
            "Releasing codec (mime=$currentMime size=${currentW}x${currentH} maxIn=$currentMaxInputSize) " +
                    "lastSurface=${lastSurface.idStr()}"
        )

        try { c.stop() } catch (t: Throwable) { log.w("codec.stop failed", t) }
        try { c.release() } catch (t: Throwable) { log.w("codec.release failed", t) }

        currentMime = null
        currentMaxInputSize = 0
        lastSurface = null
        currentW = 0
        currentH = 0
        surfaceDecodeMode = false
        outputFormatReady = false
    }
}

/* ---------- helpers ---------- */

private fun MimeType?.toCodecMime(): String = when (this) {
    MimeType.H264 -> "video/avc"
    MimeType.H265 -> "video/hevc"
    else -> "video/avc"
}

private fun Surface?.idStr(): String =
    if (this == null) "null" else "0x" + Integer.toHexString(System.identityHashCode(this))
