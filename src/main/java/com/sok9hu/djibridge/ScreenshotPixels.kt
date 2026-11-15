package com.sok9hu.djibridge

internal object ScreenshotPixels {
    fun validSize(width: Int, height: Int, bytes: Int): Boolean =
        width in 1..8192 && height in 1..8192 && width.toLong() * height <= 8_847_360 &&
            width.toLong() * height * 4 == bytes.toLong()

    // Unity supplies bottom-first RGBA; Android Bitmap consumes top-first ARGB.
    fun readTopDownRow(rgba: ByteArray, width: Int, height: Int, y: Int, row: IntArray) {
        var i = (height - y - 1) * width * 4
        for (x in 0 until width) {
            val r = rgba[i].toInt() and 255
            val g = rgba[i + 1].toInt() and 255
            val b = rgba[i + 2].toInt() and 255
            row[x] = 0xff000000.toInt() or (r shl 16) or (g shl 8) or b
            i += 4
        }
    }
}
