package com.sok9hu.djibridge

import java.io.File
import java.security.MessageDigest
import java.util.UUID
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

/** Strict packed-Y contract. No conversion, row padding, orientation or resampling. */
internal object DetectorPgm {
    fun encode(pixels: ByteArray, width: Int, height: Int, stride: Int): ByteArray {
        require(width > 0 && height > 0 && width.toLong() * height <= 16 * 1024 * 1024)
        require(stride == width && pixels.size.toLong() == width.toLong() * height)
        return "P5\n$width $height\n255\n".toByteArray(Charsets.US_ASCII) + pixels
    }
}

internal fun captureQuote(value: String): String = "\"" + buildString {
    value.forEach { c -> when (c) {
        '\\' -> append("\\\\")
        '"' -> append("\\\"")
        '\n' -> append("\\n")
        '\r' -> append("\\r")
        '\t' -> append("\\t")
        else -> if (c.code < 32) append("\\u%04x".format(c.code)) else append(c)
    } }
} + "\""

internal data class DetectorCaptureIdentity(
    val sessionId: String, val generation: Long, val frameSequence: Long,
    val acceptedSequence: Long, val callbackTimestampNs: Long,
    val width: Int, val height: Int, val stride: Int
)

internal data class DetectorCapturePacket(
    val identity: DetectorCaptureIdentity, val pixels: ByteArray,
    val detectorStartNs: Long, val detectorEndNs: Long,
    val nativeResult: String, val poseEvidence: String, val categories: Set<String>
) {
    fun manifest(file: String, imageHash: String, pixelHash: String): String {
        val i = identity
        return "{\"recordType\":\"ExactDetectorInput\",\"telemetrySessionId\":${captureQuote(i.sessionId)}," +
            "\"listenerGeneration\":${i.generation},\"frameSequence\":${i.frameSequence}," +
            "\"acceptedSequence\":${i.acceptedSequence},\"callbackTimestampNs\":${i.callbackTimestampNs}," +
            "\"detectorStartTimestampNs\":$detectorStartNs,\"detectorEndTimestampNs\":$detectorEndNs," +
            "\"exposureTimestampNs\":null,\"clockDomain\":\"ANDROID_MONOTONIC\",\"width\":${i.width}," +
            "\"height\":${i.height},\"rowStride\":${i.stride},\"pixelFormat\":\"NV21_LUMA8\"," +
            "\"rotationDegrees\":0,\"crop\":\"NONE\",\"resize\":\"NONE\",\"mirror\":false," +
            "\"file\":${captureQuote(file)},\"imageSha256\":${captureQuote(imageHash)},\"pixelSha256\":${captureQuote(pixelHash)}," +
            "\"categories\":[${categories.sorted().joinToString(",") { captureQuote(it) }}]," +
            "\"unityAdmission\":null,\"unityAdmissionJoin\":\"EXACT_SESSION_AND_SOURCE_TIMESTAMP_ONLY\"," +
            "\"nativeResult\":$nativeResult,\"poseEvidence\":$poseEvidence}"
    }
}

/** Nonblocking admission; all filesystem work and hashing run on one bounded worker. */
internal class ExactDetectorCapture(
    private val root: File, val sessionId: String, private val clock: () -> Long,
    private val write: (File, ByteArray) -> Unit = { file, bytes -> file.outputStream().use { it.write(bytes) } },
    private val maxFrames: Int = 16, private val maxBytes: Long = 64L * 1024 * 1024,
    private val windowNs: Long = 60_000_000_000L
) : AutoCloseable {
    companion object {
        val EXPECTED_CATEGORIES = setOf("NO_TAG", "TAGS_1", "TAGS_2", "TAGS_3", "TAGS_4", "NEAR_BORDER", "OBLIQUE", "RAPID_POSE_ORIENTATION_CHANGE")
        fun sha(bytes: ByteArray): String = MessageDigest.getInstance("SHA-256").digest(bytes).joinToString("") { "%02x".format(it) }
    }
    private val queue = ArrayBlockingQueue<DetectorCapturePacket>(2)
    private val worker = Thread({ runWriter() }, "DjiExactYWriter").apply { isDaemon = true }
    @Volatile private var enabled = true
    @Volatile private var deadline = 0L
    @Volatile private var nextOpportunity = 0L
    @Volatile private var stored = 0
    @Volatile private var consumedArm = false
    @Volatile private var pendingRequest = 0
    private var directory: File? = null
    private var frameLimit = maxFrames
    private var bytesReserved = 0L
    private var lastStoredNs = 0L
    private var lastFrameSequence = 0L
    private var lastGeneration = 0L
    private var attemptedWrites = 0
    private val observed = mutableSetOf<String>()
    private val saved = mutableSetOf<String>()
    val queueDrops = AtomicLong()
    val errors = AtomicLong()
    val invalidPackets = AtomicLong()
    val opportunities = AtomicLong()
    init { require(maxFrames in 1..16 && maxBytes in 1..64L * 1024 * 1024 && windowNs in 1..60_000_000_000L); worker.start() }

    fun request(count: Int): String {
        if (!enabled || consumedArm) return ""
        pendingRequest = count.coerceIn(1, maxFrames)
        return root.absolutePath
    }

    // Called only by the existing single detector worker, never by the DJI callback.
    fun wantsFrame(nowNs: Long): Boolean {
        if (!enabled || deadline == 0L || nowNs > deadline || stored >= frameLimit || nowNs < nextOpportunity) return false
        if (queue.remainingCapacity() == 0) { queueDrops.incrementAndGet(); return false }
        nextOpportunity = nowNs + 500_000_000L
        opportunities.incrementAndGet()
        return true
    }

    fun submit(packet: DetectorCapturePacket): Boolean {
        val i = packet.identity
        if (!enabled || deadline == 0L || clock() > deadline || stored >= frameLimit) return false
        if (i.sessionId != sessionId || i.generation <= 0 || i.frameSequence <= 0 || i.generation < lastGeneration ||
            (i.generation == lastGeneration && i.frameSequence <= lastFrameSequence) ||
            i.callbackTimestampNs <= 0 || packet.detectorStartNs < i.callbackTimestampNs || packet.detectorEndNs < packet.detectorStartNs ||
            i.width <= 0 || i.height <= 0 || i.stride != i.width ||
            i.width.toLong() * i.height != packet.pixels.size.toLong() || packet.pixels.size > 16 * 1024 * 1024) {
            invalidPackets.incrementAndGet(); return false
        }
        lastFrameSequence = i.frameSequence
        lastGeneration = i.generation
        // The caller owns this immutable frame until submission. Retain a copy so later mutation cannot corrupt evidence.
        if (!queue.offer(packet.copy(pixels = packet.pixels.copyOf()))) { queueDrops.incrementAndGet(); return false }
        return true
    }

    private fun arm(count: Int, nonce: String) {
        if (consumedArm) return
        consumedArm = true
        require(root.isDirectory || root.mkdirs())
        bytesReserved = root.walkTopDown().filter { it.isFile && it.extension == "pgm" }.sumOf { it.length() }
        directory = File(root, "capture_${clock()}_${UUID.randomUUID()}").also { check(it.mkdir()) }
        frameLimit = count.coerceIn(1, maxFrames)
        deadline = clock() + windowNs
        File(directory, "arm.json").writeText("{\"telemetrySessionId\":${captureQuote(sessionId)},\"nonce\":${captureQuote(nonce)},\"deadlineNs\":$deadline,\"maxFrames\":$frameLimit,\"maxBytes\":$maxBytes}")
    }

    private fun checkArm() {
        if (consumedArm) return
        val requested = pendingRequest
        if (requested > 0) { pendingRequest = 0; arm(requested, "IN_PROCESS_REQUEST"); return }
        val token = File(root, "arm.once")
        if (!token.isFile) return
        val consumed = File(root, "arm.consumed-${clock()}-${UUID.randomUUID()}")
        if (!token.renameTo(consumed)) return
        require(consumed.length() <= 256) { "ARM_TOKEN_TOO_LARGE" }
        val parts = consumed.readText().trim().split(Regex("\\s+"))
        require(parts.size == 3 && parts[0] == "ARM_ONCE" && parts[1].matches(Regex("[A-Za-z0-9_-]{1,64}"))) { "INVALID_ARM_TOKEN" }
        arm(parts[2].toInt(), parts[1])
    }

    private fun save(packet: DetectorCapturePacket) {
        observed.addAll(packet.categories)
        if (attemptedWrites >= frameLimit) return
        val time = packet.identity.callbackTimestampNs
        if (stored > 0 && packet.categories.all { it in saved } && time - lastStoredNs < 5_000_000_000L) return
        val encoded = DetectorPgm.encode(packet.pixels, packet.identity.width, packet.identity.height, packet.identity.stride)
        if (bytesReserved + encoded.size > maxBytes) return
        // Reserve even failed writes, conservatively bounding residual/partial files.
        bytesReserved += encoded.size
        attemptedWrites++
        val name = "frame_${stored.toString().padStart(3, '0')}_${packet.identity.frameSequence}_$time.pgm"
        val dir = checkNotNull(directory)
        write(File(dir, name), encoded)
        File(dir, "capture_manifest.jsonl").appendText(packet.manifest(name, sha(encoded), sha(packet.pixels)) + "\n")
        stored++; saved.addAll(packet.categories); lastStoredNs = time
    }

    private fun runWriter() {
        try {
            while (enabled || queue.isNotEmpty()) {
                try {
                    if (enabled) checkArm()
                    val packet = queue.poll(250, TimeUnit.MILLISECONDS)
                    if (packet != null) save(packet)
                    if (deadline != 0L && (clock() > deadline || stored >= frameLimit)) enabled = false
                } catch (_: Exception) { errors.incrementAndGet() }
            }
        } finally {
            try {
                directory?.let { dir ->
                    File(dir, "summary.json").writeText("{\"telemetrySessionId\":${captureQuote(sessionId)},\"storedFrames\":$stored,\"bytesReservedIncludingPreviousCaptures\":$bytesReserved," +
                        "\"queueDrops\":${queueDrops.get()},\"writeOrControlErrors\":${errors.get()},\"invalidPackets\":${invalidPackets.get()},\"opportunities\":${opportunities.get()}," +
                        "\"observedCategories\":[${observed.sorted().joinToString(",") { captureQuote(it) }}]," +
                        "\"missingCategories\":[${(EXPECTED_CATEGORIES - saved).sorted().joinToString(",") { captureQuote(it) }}]," +
                        "\"partialTagVisibility\":\"REQUIRES_IMAGE_INSPECTION\",\"unityRejectionCategory\":\"REQUIRES_EXACT_JSONL_JOIN\",\"alternativeSolutions\":\"OFFLINE_REPLAY_REQUIRED\"}")
                }
            } catch (_: Exception) { errors.incrementAndGet() }
        }
    }

    override fun close() { enabled = false }
    fun awaitClosed(timeoutMs: Long): Boolean { worker.join(timeoutMs); return !worker.isAlive }
}
