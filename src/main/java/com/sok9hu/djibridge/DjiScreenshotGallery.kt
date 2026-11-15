package com.sok9hu.djibridge

import android.Manifest
import android.content.ContentValues
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.media.MediaScannerConnection
import android.net.Uri
import android.os.Build
import android.os.Environment
import android.os.SystemClock
import android.provider.MediaStore
import android.util.Log
import org.json.JSONObject
import java.io.File
import java.io.IOException
import java.io.OutputStream
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/** Writes one opaque PNG at a time. Does not retain an Activity or start one. */
object DjiScreenshotGallery {
    private const val TAG = "DjiScreenshot"
    private const val ALBUM = "RoofCanopy"
    private const val MIME = "image/png"
    private val requestId = Regex("[a-f0-9]{32}")
    private val legacyTemporaryName = Regex("\\.RoofCanopy_[0-9]{8}T[0-9]{9}Z_[a-f0-9]{32}\\.png\\.part")
    private val busy = AtomicBoolean()
    private val worker by lazy { Executors.newSingleThreadExecutor { r -> Thread(r, "RoofCanopyGallery") } }
    @Volatile private var completed: Result? = null

    private data class Result(val id: String, val status: String, val name: String, val uri: String?)

    @JvmStatic
    fun needsLegacyWritePermission(): Boolean = Build.VERSION.SDK_INT < Build.VERSION_CODES.Q

    @JvmStatic
    fun isBusy(): Boolean = busy.get()

    @JvmStatic
    fun getResultJson(id: String): String {
        val result = completed?.takeIf { it.id == id } ?: return ""
        return JSONObject().put("status", result.status).put("name", result.name)
            .put("uri", result.uri ?: JSONObject.NULL).toString()
    }

    @JvmStatic
    fun trySave(id: String, width: Int, height: Int, rgba: ByteArray, requestedNs: Long): String {
        if (!id.matches(requestId) || !ScreenshotPixels.validSize(width, height, rgba.size))
            return "INVALID_IMAGE"
        val context = DJIPlugin.applicationContextOrNull() ?: return "NO_CONTEXT"
        if (needsLegacyWritePermission() &&
            context.checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED
        ) return "PERMISSION_REQUIRED"
        if (!busy.compareAndSet(false, true)) return "BUSY"
        val started = SystemClock.elapsedRealtimeNanos()
        try {
            worker.execute { write(context, id, width, height, rgba, requestedNs, started) }
        } catch (e: Exception) {
            busy.set(false)
            return "WORKER_UNAVAILABLE"
        }
        return "QUEUED"
    }

    private fun write(context: Context, id: String, width: Int, height: Int, rgba: ByteArray, requested: Long, started: Long) {
        var bitmap: Bitmap? = null
        var name = ""
        var uri: String? = null
        var status = "FAILED"
        try {
            val format = SimpleDateFormat("yyyyMMdd'T'HHmmssSSS'Z'", Locale.US).apply {
                timeZone = TimeZone.getTimeZone("UTC")
            }
            name = "RoofCanopy_${format.format(Date())}_$id.png"
            bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
            bitmap.setHasAlpha(false)
            val row = IntArray(width)
            for (y in 0 until height) {
                ScreenshotPixels.readTopDownRow(rgba, width, height, y, row)
                bitmap.setPixels(row, 0, width, 0, y, width, 1)
            }
            uri = if (needsLegacyWritePermission()) writeLegacy(context, name, bitmap) else writeMediaStore(context, name, bitmap)
            status = "SAVED"
        } catch (e: SecurityException) {
            status = "PERMISSION_DENIED"
        } catch (e: Exception) {
            Log.w(TAG, "SCREENSHOT_STORAGE_ERROR id=$id type=${e.javaClass.simpleName}")
        } catch (e: OutOfMemoryError) {
            status = "OUT_OF_MEMORY"
        } finally {
            bitmap?.recycle()
            completed = Result(id, status, name, uri)
            busy.set(false)
            val finished = SystemClock.elapsedRealtimeNanos()
            val total = if (requested in 1..finished) (finished - requested) / 1e6 else null
            Log.i(TAG, "SCREENSHOT_GALLERY_RESULT id=$id status=$status width=$width height=$height " +
                "workerStartNs=$started finishedNs=$finished durationMs=${(finished - started) / 1e6} totalMs=$total file=$name")
        }
    }

    @android.annotation.TargetApi(29)
    private fun writeMediaStore(context: Context, name: String, bitmap: Bitmap): String {
        val resolver = context.contentResolver
        val values = ContentValues().apply {
            put(MediaStore.MediaColumns.DISPLAY_NAME, name)
            put(MediaStore.MediaColumns.MIME_TYPE, MIME)
            put(MediaStore.MediaColumns.RELATIVE_PATH, "${Environment.DIRECTORY_PICTURES}/$ALBUM")
            put(MediaStore.MediaColumns.IS_PENDING, 1)
        }
        val images = MediaStore.Images.Media.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY)
        val uri = resolver.insert(images, values) ?: throw IOException("MediaStore insert returned null")
        var published = false
        try {
            (resolver.openOutputStream(uri, "w") ?: throw IOException("MediaStore stream unavailable")).use {
                encode(bitmap, it)
            }
            val ready = ContentValues().apply { put(MediaStore.MediaColumns.IS_PENDING, 0) }
            if (resolver.update(uri, ready, null, null) != 1) throw IOException("MediaStore publish failed")
            published = true
            return uri.toString()
        } finally {
            if (!published) runCatching { resolver.delete(uri, null, null) }
                .onFailure { Log.w(TAG, "SCREENSHOT_PENDING_CLEANUP_FAILED type=${it.javaClass.simpleName}") }
        }
    }

    @Suppress("DEPRECATION")
    private fun writeLegacy(context: Context, name: String, bitmap: Bitmap): String {
        if (Environment.getExternalStorageState() != Environment.MEDIA_MOUNTED) throw IOException("Storage unavailable")
        val folder = File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), ALBUM)
        if (!folder.isDirectory && !folder.mkdirs()) throw IOException("Album unavailable")
        // This process has one writer; any matching partial file is from a prior interrupted run.
        folder.listFiles { f -> f.isFile && f.name.matches(legacyTemporaryName) }?.forEach {
            if (!it.delete()) Log.w(TAG, "SCREENSHOT_TEMP_CLEANUP_FAILED")
        }
        val finalFile = File(folder, name)
        val temporary = File(folder, ".$name.part")
        if (finalFile.exists() || !temporary.createNewFile()) throw IOException("Screenshot filename already exists")
        var renamed = false
        var indexed = false
        try {
            temporary.outputStream().use { encode(bitmap, it) }
            if (!temporary.renameTo(finalFile)) throw IOException("Cannot publish screenshot")
            renamed = true
            val uri = scan(context, finalFile)
            indexed = true
            return uri.toString()
        } finally {
            if (temporary.exists() && !temporary.delete()) Log.w(TAG, "SCREENSHOT_TEMP_CLEANUP_FAILED")
            if (renamed && !indexed && !finalFile.delete()) Log.w(TAG, "SCREENSHOT_LEGACY_CLEANUP_FAILED")
        }
    }

    private fun scan(context: Context, file: File): Uri {
        val done = CountDownLatch(1)
        val result = AtomicReference<Uri?>()
        val expired = AtomicBoolean()
        MediaScannerConnection.scanFile(context, arrayOf(file.absolutePath), arrayOf(MIME)) { _, uri ->
            result.set(uri)
            done.countDown()
            if (expired.get()) result.getAndSet(null)?.let { removeIndexed(context, it) }
        }
        try {
            if (done.await(8, TimeUnit.SECONDS)) return result.get() ?: throw IOException("Gallery indexing failed")
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        expired.set(true)
        result.getAndSet(null)?.let { removeIndexed(context, it) }
        throw IOException("Gallery indexing timed out")
    }

    private fun removeIndexed(context: Context, uri: Uri) {
        runCatching { context.contentResolver.delete(uri, null, null) }
            .onFailure { Log.w(TAG, "SCREENSHOT_INDEX_CLEANUP_FAILED type=${it.javaClass.simpleName}") }
    }

    private fun encode(bitmap: Bitmap, output: OutputStream) {
        if (!bitmap.compress(Bitmap.CompressFormat.PNG, 100, output)) throw IOException("PNG encoding failed")
    }
}
