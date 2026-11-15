package com.sok9hu.djibridge

import android.content.Context
import android.util.Log
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.nio.charset.StandardCharsets
import java.text.SimpleDateFormat
import java.util.ArrayDeque
import java.util.Date
import java.util.Locale
import java.util.TimeZone

/** Diagnostic-only raw KeyManager callback capture with asynchronous JSONL I/O. */
internal object DjiRawTelemetryCallbackLogger {
    private const val TAG = "DjiRawTelemetryCallbacks"
    private const val QUEUE_CAPACITY = 1024
    private const val MAX_INTERVALS = 256

    private val stateLock = Any()
    private val queue = RawTelemetryCallbackQueue<RawCallbackRecord>(QUEUE_CAPACITY)
    private val positionIntervalsMs = ArrayDeque<Double>()

    private var accepting = false
    private var writerThread: Thread? = null
    private var positionWriter: BufferedWriter? = null
    private var velocityWriter: BufferedWriter? = null
    private var sessionId: String? = null
    private var positionPath: String? = null
    private var callbacks = 0L
    private var dropped = 0L
    private var lastPositionTimestampNs = 0L
    private var writeFailureLogged = false

    fun start(context: Context, requestedSessionId: String): Boolean {
        stop()
        if (requestedSessionId.isBlank()) {
            Log.w(TAG, "DJI_POSITION_CALLBACK_LOG_ERROR reason=EMPTY_SESSION_ID")
            return false
        }

        return try {
            val directory = context.getExternalFilesDir(null) ?: context.filesDir
            directory.mkdirs()
            val positionFile = File(directory, "dji-position-callbacks-$requestedSessionId.jsonl")
            val velocityFile = File(directory, "dji-velocity-callbacks-$requestedSessionId.jsonl")
            val positionOutput = newWriter(positionFile)
            val velocityOutput = newWriter(velocityFile)

            synchronized(stateLock) {
                queue.reopen()
                positionIntervalsMs.clear()
                callbacks = 0L
                dropped = 0L
                lastPositionTimestampNs = 0L
                writeFailureLogged = false
                sessionId = requestedSessionId
                positionPath = positionFile.absolutePath
                positionWriter = positionOutput
                velocityWriter = velocityOutput
                accepting = true
                writerThread = Thread(::writeLoop, "DjiRawTelemetryCallbackWriter").apply {
                    isDaemon = true
                    start()
                }
            }
            Log.i(
                TAG,
                "DJI_POSITION_CALLBACK_LOG_STARTED sessionId=$requestedSessionId path=${positionFile.absolutePath} " +
                    "velocityPath=${velocityFile.absolutePath} queueCapacity=$QUEUE_CAPACITY clockDomain=ANDROID_MONOTONIC"
            )
            true
        } catch (throwable: Throwable) {
            closeWriters()
            Log.e(TAG, "DJI_POSITION_CALLBACK_LOG_ERROR reason=START_FAILED message=${throwable.message}", throwable)
            false
        }
    }

    fun stop() {
        val thread: Thread?
        synchronized(stateLock) {
            if (!accepting && writerThread == null)
                return
            accepting = false
            queue.close()
            thread = writerThread
        }

        // This runs from the Unity lifecycle, never from a DJI callback.
        thread?.join(2_000L)
        if (thread?.isAlive == true) {
            Log.w(TAG, "DJI_POSITION_CALLBACK_LOG_ERROR reason=STOP_FLUSH_TIMEOUT")
            return
        }

        synchronized(stateLock) {
            writerThread = null
            closeWriters()
            Log.i(
                TAG,
                "DJI_POSITION_CALLBACK_LOG_STOPPED sessionId=${sessionId ?: "UNAVAILABLE"} callbacks=$callbacks " +
                    "dropped=$dropped medianIntervalMs=${median(positionIntervalsMs) ?: "UNAVAILABLE"} " +
                    "path=${positionPath ?: "UNAVAILABLE"}"
            )
            sessionId = null
            positionPath = null
        }
    }

    fun enqueuePosition(
        timestampNs: Long,
        utcTimestampMs: Long,
        latitude: Double,
        longitude: Double,
        altitude: Double,
        gpsValid: Boolean?,
        gpsSignalLevel: String?,
        satelliteCount: Int?
    ) {
        if (timestampNs <= 0L) return
        enqueue(
            RawPositionRecord(
                timestampNs, utcTimestampMs, latitude, longitude, altitude, gpsValid, gpsSignalLevel, satelliteCount
            )
        ) { record ->
            if (lastPositionTimestampNs > 0L && record.timestampNs >= lastPositionTimestampNs) {
                positionIntervalsMs.addLast((record.timestampNs - lastPositionTimestampNs) / 1_000_000.0)
                while (positionIntervalsMs.size > MAX_INTERVALS)
                    positionIntervalsMs.removeFirst()
            }
            lastPositionTimestampNs = record.timestampNs
        }
    }

    fun enqueueVelocity(
        timestampNs: Long,
        utcTimestampMs: Long,
        northMetersPerSecond: Double,
        eastMetersPerSecond: Double,
        downMetersPerSecond: Double
    ) {
        if (timestampNs <= 0L) return
        enqueue(
            RawVelocityRecord(timestampNs, utcTimestampMs, northMetersPerSecond, eastMetersPerSecond, downMetersPerSecond)
        ) { }
    }

    private fun <T : RawCallbackRecord> enqueue(record: T, onAccepted: (T) -> Unit) {
        synchronized(stateLock) {
            if (!accepting) return
            callbacks++
            if (!queue.tryEnqueue(record)) {
                dropped++
                return
            }
            onAccepted(record)
        }
    }

    private fun writeLoop() {
        try {
            while (true) {
                when (val record = queue.take()) {
                    null -> break
                    is RawPositionRecord -> positionWriter?.apply {
                        write(RawTelemetryCallbackJson.positionLine(sessionId.orEmpty(), record))
                        newLine()
                    }
                    is RawVelocityRecord -> velocityWriter?.apply {
                        write(RawTelemetryCallbackJson.velocityLine(sessionId.orEmpty(), record))
                        newLine()
                    }
                }
            }
            positionWriter?.flush()
            velocityWriter?.flush()
        } catch (throwable: Throwable) {
            synchronized(stateLock) { accepting = false }
            if (!writeFailureLogged) {
                writeFailureLogged = true
                Log.e(TAG, "DJI_POSITION_CALLBACK_LOG_ERROR reason=WRITE_FAILED message=${throwable.message}", throwable)
            }
        } finally {
            synchronized(stateLock) { closeWriters() }
        }
    }

    private fun newWriter(file: File): BufferedWriter = BufferedWriter(
        OutputStreamWriter(FileOutputStream(file, false), StandardCharsets.UTF_8),
        16 * 1024
    )

    private fun closeWriters() {
        runCatching { positionWriter?.flush() }
        runCatching { velocityWriter?.flush() }
        runCatching { positionWriter?.close() }
        runCatching { velocityWriter?.close() }
        positionWriter = null
        velocityWriter = null
    }

    private fun median(values: Collection<Double>): Double? {
        if (values.isEmpty()) return null
        val sorted = values.sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) / 2.0 else sorted[middle]
    }
}

internal class RawTelemetryCallbackQueue<T>(private val capacity: Int) {
    private val items = ArrayDeque<T>()
    private var closed = false

    init {
        require(capacity > 0)
    }

    @Synchronized
    fun tryEnqueue(item: T): Boolean {
        if (closed || items.size >= capacity) return false
        items.addLast(item)
        (this as java.lang.Object).notifyAll()
        return true
    }

    @Synchronized
    fun take(): T? {
        while (items.isEmpty() && !closed)
            (this as java.lang.Object).wait()
        return if (items.isEmpty()) null else items.removeFirst()
    }

    @Synchronized
    fun close() {
        closed = true
        (this as java.lang.Object).notifyAll()
    }

    @Synchronized
    fun reopen() {
        items.clear()
        closed = false
    }
}

internal sealed interface RawCallbackRecord {
    val timestampNs: Long
    val utcTimestampMs: Long
}

internal data class RawPositionRecord(
    override val timestampNs: Long,
    override val utcTimestampMs: Long,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val gpsValid: Boolean?,
    val gpsSignalLevel: String?,
    val satelliteCount: Int?
) : RawCallbackRecord

internal data class RawVelocityRecord(
    override val timestampNs: Long,
    override val utcTimestampMs: Long,
    val northMetersPerSecond: Double,
    val eastMetersPerSecond: Double,
    val downMetersPerSecond: Double
) : RawCallbackRecord

internal object RawTelemetryCallbackJson {
    private val utcFormatter = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US).apply {
        timeZone = TimeZone.getTimeZone("UTC")
    }

    fun positionLine(sessionId: String, record: RawPositionRecord): String = buildString {
        append('{')
        stringProperty("telemetrySessionId", sessionId)
        numberProperty("timestampNs", record.timestampNs)
        stringProperty("utc", utcFormatter.format(Date(record.utcTimestampMs)))
        numberProperty("latitude", record.latitude)
        numberProperty("longitude", record.longitude)
        numberProperty("altitude", record.altitude)
        nullableBooleanProperty("gpsValid", record.gpsValid)
        nullableStringProperty("gpsSignalLevel", record.gpsSignalLevel)
        nullableNumberProperty("satelliteCount", record.satelliteCount)
        stringProperty("qualitySnapshot", "LATEST_KNOWN_AT_LOCATION_CALLBACK", final = true)
        append('}')
    }

    fun velocityLine(sessionId: String, record: RawVelocityRecord): String {
        val speed = kotlin.math.sqrt(
            record.northMetersPerSecond * record.northMetersPerSecond +
                record.eastMetersPerSecond * record.eastMetersPerSecond +
                record.downMetersPerSecond * record.downMetersPerSecond
        )
        return buildString {
            append('{')
            stringProperty("telemetrySessionId", sessionId)
            numberProperty("timestampNs", record.timestampNs)
            stringProperty("utc", utcFormatter.format(Date(record.utcTimestampMs)))
            numberProperty("north", record.northMetersPerSecond)
            numberProperty("east", record.eastMetersPerSecond)
            numberProperty("down", record.downMetersPerSecond)
            numberProperty("speedMagnitude", speed, final = true)
            append('}')
        }
    }

    private fun StringBuilder.stringProperty(name: String, value: String, final: Boolean = false) {
        appendQuoted(name).append(':').appendQuoted(value).appendSeparator(final)
    }

    private fun StringBuilder.nullableStringProperty(name: String, value: String?) {
        appendQuoted(name).append(':')
        if (value == null) append("null") else appendQuoted(value)
        appendSeparator()
    }

    private fun StringBuilder.numberProperty(name: String, value: Number, final: Boolean = false) {
        appendQuoted(name).append(':').append(value).appendSeparator(final)
    }

    private fun StringBuilder.nullableNumberProperty(name: String, value: Number?) {
        appendQuoted(name).append(':').append(value ?: "null").appendSeparator()
    }

    private fun StringBuilder.nullableBooleanProperty(name: String, value: Boolean?) {
        appendQuoted(name).append(':').append(value ?: "null").appendSeparator()
    }

    private fun StringBuilder.appendSeparator(final: Boolean = false) {
        if (!final) append(',')
    }

    private fun StringBuilder.appendQuoted(value: String): StringBuilder = append('"').append(
        value.replace("\\", "\\\\").replace("\"", "\\\"")
    ).append('"')
}
