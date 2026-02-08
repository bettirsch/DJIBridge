package com.sok9hu.djibridge.video

/* =========================
 * NAL parsing (Annex-B + AVCC)
 * ========================= */
object NalUnitParser {

    fun forEachNal(
        data: ByteArray,
        isHevc: Boolean,
        onNal: (nal: ByteArray, nalType: Int) -> Unit
    ) {
        if (data.isEmpty()) return
        if (looksLikeAnnexB(data)) forEachNalAnnexB(data, isHevc, onNal)
        else forEachNalAvcc(data, isHevc, onNal)
    }

    private fun looksLikeAnnexB(data: ByteArray): Boolean {
        if (data.size < 4) return false
        return (data[0] == 0.toByte() && data[1] == 0.toByte() && data[2] == 1.toByte()) ||
                (data[0] == 0.toByte() && data[1] == 0.toByte() && data[2] == 0.toByte() && data[3] == 1.toByte())
    }

    private fun startCodeLenAt(data: ByteArray, pos: Int): Int {
        if (pos + 3 < data.size &&
            data[pos] == 0.toByte() && data[pos + 1] == 0.toByte() &&
            data[pos + 2] == 0.toByte() && data[pos + 3] == 1.toByte()
        ) return 4

        if (pos + 2 < data.size &&
            data[pos] == 0.toByte() && data[pos + 1] == 0.toByte() &&
            data[pos + 2] == 1.toByte()
        ) return 3

        return 0
    }

    private fun forEachNalAnnexB(
        data: ByteArray,
        isHevc: Boolean,
        onNal: (nal: ByteArray, nalType: Int) -> Unit
    ) {
        var i = 0
        while (i < data.size && startCodeLenAt(data, i) == 0) i++

        while (i < data.size) {
            val scLen = startCodeLenAt(data, i)
            if (scLen == 0) {
                i++
                continue
            }

            val nalStart = i + scLen
            var j = nalStart
            while (j < data.size && startCodeLenAt(data, j) == 0) j++

            val nalEnd = j
            if (nalStart < nalEnd) {
                val nal = data.copyOfRange(nalStart, nalEnd)
                onNal(nal, nalTypeOf(nal, isHevc))
            }
            i = j
        }
    }

    private fun forEachNalAvcc(
        data: ByteArray,
        isHevc: Boolean,
        onNal: (nal: ByteArray, nalType: Int) -> Unit
    ) {
        var i = 0
        while (i + 4 <= data.size) {
            val len =
                ((data[i].toInt() and 0xFF) shl 24) or
                        ((data[i + 1].toInt() and 0xFF) shl 16) or
                        ((data[i + 2].toInt() and 0xFF) shl 8) or
                        (data[i + 3].toInt() and 0xFF)

            i += 4
            if (len <= 0 || i + len > data.size) return

            val nal = data.copyOfRange(i, i + len)
            onNal(nal, nalTypeOf(nal, isHevc))
            i += len
        }
    }

    private fun nalTypeOf(nal: ByteArray, isHevc: Boolean): Int {
        if (nal.isEmpty()) return -1
        return if (!isHevc) {
            nal[0].toInt() and 0x1F
        } else {
            (nal[0].toInt() shr 1) and 0x3F
        }
    }
}
