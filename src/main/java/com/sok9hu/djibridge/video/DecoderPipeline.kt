package com.sok9hu.djibridge.video

import com.sok9hu.djibridge.PipelineLog

class DecoderPipeline(
    private val tag: String,
    private val surfaceState: SurfaceState
) {
    private val log = PipelineLog(tag)

    private val codecController = CodecController(tag, surfaceState)
    private val csdStore = CsdStore()
    private val ptsGenerator = PtsGenerator()

    fun onPacket(pkt: Packet) {
        try {
            val snap = surfaceState.snapshot()
            val resolved = resolvePacket(pkt, snap)

            log.iEvery("pkt", 1000L) {
                "onPacket mime=${resolved.mimeEnum} size=${resolved.width}x${resolved.height} " +
                        "fps=${resolved.frameRate} bytes=${resolved.bytes.size} " +
                        "ptsUs=${resolved.ptsUs} surfaceValid=${snap.isValid()}"
            }

            csdStore.updateFrom(resolved)

            codecController.ensureConfigured(
                pkt = resolved,
                csdStore = csdStore,
                ptsGenerator = ptsGenerator
            )

            codecController.feedAndDrain(resolved, ptsGenerator)
        } catch (t: Throwable) {
            log.e("Decode error", t)
            codecController.release()
        }
    }

    fun onSurfacePossiblyChanged() {
        log.d("onSurfacePossiblyChanged()")
        codecController.onSurfacePossiblyChanged()
    }

    fun release() {
        log.i("DecoderPipeline.release()")
        codecController.release()
    }

    private fun resolvePacket(pkt: Packet, surface: SurfaceSnapshot): Packet {
        val w = when {
            pkt.width > 0 -> pkt.width
            surface.width > 0 -> surface.width
            else -> 1280
        }
        val h = when {
            pkt.height > 0 -> pkt.height
            surface.height > 0 -> surface.height
            else -> 720
        }
        val fr = if (pkt.frameRate > 0) pkt.frameRate else 30

        if (w != pkt.width || h != pkt.height || fr != pkt.frameRate) {
            log.iEvery("resolve", 2000L) {
                "resolvePacket: pkt=${pkt.width}x${pkt.height}@${pkt.frameRate} " +
                        "surface=${surface.width}x${surface.height} -> resolved=${w}x${h}@${fr}"
            }
        }

        return pkt.copy(width = w, height = h, frameRate = fr)
    }
}

