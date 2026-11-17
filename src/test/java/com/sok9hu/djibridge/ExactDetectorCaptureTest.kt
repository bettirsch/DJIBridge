package com.sok9hu.djibridge

import org.junit.Assert.*
import org.junit.Test
import java.io.File
import java.nio.file.Files
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

class ExactDetectorCaptureTest {
    private fun root() = Files.createTempDirectory("exact-y-test").toFile()
    private fun packet(sequence: Long = 1, time: Long = 100, pixels: ByteArray = byteArrayOf(0, 10, -1, 92), status: Int = -8) =
        DetectorCapturePacket(DetectorCaptureIdentity("test-session", 7, sequence, sequence + 10, time, 2, 2, 2),
            pixels, time + 1, time + 2, "{\"timestampNs\":$time,\"status\":$status}", "null", setOf(if (status == -8) "NO_TAG" else "TAGS_4"))
    private fun waitUntil(condition: () -> Boolean) {
        val deadline = System.nanoTime() + 3_000_000_000L
        while (System.nanoTime() < deadline) {
            if (condition()) return
            Thread.sleep(5)
        }
        fail("Timed out")
    }
    private fun arm(root: File, capture: ExactDetectorCapture, now: AtomicLong) {
        val pending = File(root, "arm.pending")
        pending.writeText("ARM_ONCE fixture 16")
        assertTrue(pending.renameTo(File(root, "arm.once")))
        waitUntil { capture.wantsFrame(now.get()) }
    }

    @Test fun pgmRoundTripPreservesEveryByteAndRealNewlines() {
        val bytes = ByteArray(256) { it.toByte() }
        val encoded = DetectorPgm.encode(bytes, 16, 16, 16)
        val header = "P5\n16 16\n255\n".toByteArray()
        assertArrayEquals(header, encoded.copyOfRange(0, header.size))
        assertArrayEquals(bytes, encoded.copyOfRange(header.size, encoded.size))
        assertEquals(header.size + 256, encoded.size)
    }
    @Test fun wrongStrideRejected() { try { DetectorPgm.encode(ByteArray(4), 2, 2, 3); fail() } catch (_: IllegalArgumentException) {} }
    @Test fun truncatedPayloadRejected() { try { DetectorPgm.encode(ByteArray(3), 2, 2, 2); fail() } catch (_: IllegalArgumentException) {} }
    @Test fun surplusPayloadRejected() { try { DetectorPgm.encode(ByteArray(5), 2, 2, 2); fail() } catch (_: IllegalArgumentException) {} }
    @Test fun invalidDimensionsRejected() { try { DetectorPgm.encode(ByteArray(0), 0, 2, 0); fail() } catch (_: IllegalArgumentException) {} }
    @Test fun disabledUntilExplicitArmAndNeverWritesAnImage() {
        val r = root(); val c = ExactDetectorCapture(r, "test-session", { 100L })
        assertFalse(c.wantsFrame(100)); assertFalse(c.submit(packet())); c.close(); assertTrue(c.awaitClosed(2000))
        assertEquals(0, r.walkTopDown().count { it.extension == "pgm" })
    }
    @Test fun exactIdentityNativeErrorAndPixelFixture() {
        val r = root(); val now = AtomicLong(100); val c = ExactDetectorCapture(r, "test-session", now::get)
        arm(r, c, now); val p = packet(); assertTrue(c.submit(p)); c.close(); assertTrue(c.awaitClosed(2000))
        val line = r.walkTopDown().first { it.name == "capture_manifest.jsonl" }.readText()
        assertTrue(line.contains("\"listenerGeneration\":7")); assertTrue(line.contains("\"frameSequence\":1"))
        assertTrue(line.contains("\"callbackTimestampNs\":100")); assertTrue(line.contains("\"detectorStartTimestampNs\":101"))
        assertTrue(line.contains("\"detectorEndTimestampNs\":102")); assertTrue(line.contains("\"status\":-8"))
        assertTrue(line.contains("\"exposureTimestampNs\":null")); assertTrue(line.endsWith("\n"))
        val image = r.walkTopDown().first { it.extension == "pgm" }
        assertArrayEquals(DetectorPgm.encode(p.pixels, 2, 2, 2), image.readBytes())
        System.getenv("DJI_CAPTURE_TEST_OUTPUT_DIR")?.let { output ->
            File(output).mkdirs(); image.copyTo(File(output, "jvm-exact-y.pgm"), overwrite = true)
            File(output, "jvm-exact-y.jsonl").writeText(line)
        }
    }
    @Test fun successfulResultCapturedWithoutMutation() {
        val r = root(); val now = AtomicLong(100); val c = ExactDetectorCapture(r, "test-session", now::get)
        arm(r, c, now); val p = packet(status = 2); val original = p.pixels.copyOf(); val result = p.nativeResult
        assertTrue(c.submit(p)); c.close(); assertTrue(c.awaitClosed(2000))
        assertArrayEquals(original, p.pixels); assertEquals(result, p.nativeResult)
        assertTrue(r.walkTopDown().first { it.name == "capture_manifest.jsonl" }.readText().contains("\"status\":2"))
    }
    @Test fun rejectsMismatchedSessionGenerationTimingAndDuplicateSequence() {
        val now = AtomicLong(100); val r = root(); val c = ExactDetectorCapture(r, "test-session", now::get); arm(r, c, now)
        val p = packet()
        assertFalse(c.submit(p.copy(identity = p.identity.copy(sessionId = "wrong"))))
        assertFalse(c.submit(p.copy(identity = p.identity.copy(generation = 0))))
        assertFalse(c.submit(p.copy(detectorStartNs = 99)))
        assertTrue(c.submit(p)); assertFalse(c.submit(p)); assertEquals(4, c.invalidPackets.get())
        c.close(); assertTrue(c.awaitClosed(2000))
    }
    @Test fun timeoutStopsNewFramesWithoutDeletingEvidence() {
        val now = AtomicLong(100); val r = root(); val c = ExactDetectorCapture(r, "test-session", now::get)
        arm(r, c, now); now.set(60_000_000_101L)
        assertFalse(c.wantsFrame(now.get())); assertFalse(c.submit(packet())); assertTrue(c.awaitClosed(2000))
        assertTrue(r.walkTopDown().any { it.name == "summary.json" })
    }
    @Test fun maximumFrameCountIsSixteenIncludingDiverseResults() {
        val now = AtomicLong(100); val r = root(); val c = ExactDetectorCapture(r, "test-session", now::get); arm(r, c, now)
        for (n in 1..30) {
            now.set(n.toLong() * 1_000_000_000L)
            c.submit(packet(n.toLong(), now.get()).copy(categories = setOf("TEST_$n")))
            Thread.sleep(15)
        }
        c.close(); assertTrue(c.awaitClosed(2000)); assertEquals(16, r.walkTopDown().count { it.extension == "pgm" })
    }
    @Test fun byteBudgetIncludesPriorEvidenceAndNeverDeletesIt() {
        val r = root(); val old = File(r, "previous.pgm"); old.writeBytes(ByteArray(30))
        val now = AtomicLong(100); val c = ExactDetectorCapture(r, "test-session", now::get, maxBytes = 40)
        arm(r, c, now); assertTrue(c.submit(packet())); c.close(); assertTrue(c.awaitClosed(2000))
        assertEquals(1, r.walkTopDown().count { it.extension == "pgm" }); assertEquals(30L, old.length())
    }
    @Test fun queueBoundedAndSourceOwnedCopyIsImmutable() {
        val r = root(); val now = AtomicLong(100); val entered = CountDownLatch(1); val release = CountDownLatch(1)
        val c = ExactDetectorCapture(r, "test-session", now::get, write = { f, bytes -> entered.countDown(); release.await(2, TimeUnit.SECONDS); f.writeBytes(bytes) })
        arm(r, c, now); val p = packet(); assertTrue(c.submit(p)); assertTrue(entered.await(1, TimeUnit.SECONDS))
        p.pixels.fill(55)
        assertTrue(c.submit(packet(2))); assertTrue(c.submit(packet(3))); assertFalse(c.submit(packet(4)))
        assertEquals(1, c.queueDrops.get()); c.close(); release.countDown(); assertTrue(c.awaitClosed(2000))
        assertArrayEquals(DetectorPgm.encode(byteArrayOf(0, 10, -1, 92), 2, 2, 2), r.walkTopDown().first { it.extension == "pgm" }.readBytes())
    }
    @Test fun fileFailureIsSeparateAndCannotEscapeToDetectorCaller() {
        val now = AtomicLong(100); val r = root()
        val c = ExactDetectorCapture(r, "test-session", now::get, write = { _, _ -> throw java.io.IOException("injected") })
        arm(r, c, now); assertTrue(c.submit(packet())); c.close(); assertTrue(c.awaitClosed(2000)); assertEquals(1, c.errors.get())
    }
    @Test fun armFileOneShotIsRenamedNotDeletedAndCountClamped() {
        val r = root(); File(r, "arm.once").writeText("ARM_ONCE developer_123 999\n")
        val now = AtomicLong(100); val c = ExactDetectorCapture(r, "test-session", now::get)
        waitUntil { c.wantsFrame(100) }; assertFalse(File(r, "arm.once").exists())
        c.close(); assertTrue(c.awaitClosed(2000)); assertTrue(r.listFiles()!!.any { it.name.startsWith("arm.consumed-") })
        assertTrue(r.walkTopDown().first { it.name == "arm.json" }.readText().contains("\"maxFrames\":16"))
    }
    @Test fun representativeMissingCasesAreExplicitNotFabricated() {
        val now = AtomicLong(100); val r = root(); val c = ExactDetectorCapture(r, "test-session", now::get); arm(r, c, now)
        c.submit(packet()); c.close(); assertTrue(c.awaitClosed(2000))
        val summary = r.walkTopDown().first { it.name == "summary.json" }.readText()
        assertTrue(summary.contains("REQUIRES_IMAGE_INSPECTION")); assertTrue(summary.contains("TAGS_4")); assertTrue(summary.contains("OFFLINE_REPLAY_REQUIRED"))
    }
    @Test fun listenerRestartRetainsExactGenerationWithoutFalseSequenceDeduplication() {
        val now = AtomicLong(100); val r = root(); val c = ExactDetectorCapture(r, "test-session", now::get); arm(r, c, now)
        assertTrue(c.submit(packet(20)))
        waitUntil { c.wantsFrame(500_000_100) }
        assertTrue(c.submit(packet(1).copy(identity = packet(1).identity.copy(generation = 8))))
        assertFalse(c.submit(packet(21)))
        c.close(); assertTrue(c.awaitClosed(2000))
    }
    @Test fun repeatedArmingCannotExtendWindowOrFrameBudget() {
        val now = AtomicLong(100); val r = root(); val c = ExactDetectorCapture(r, "test-session", now::get); arm(r, c, now)
        File(r, "arm.once").writeText("ARM_ONCE second 16")
        now.set(60_000_000_101L); assertTrue(c.awaitClosed(2000)); assertTrue(File(r, "arm.once").exists())
        assertFalse(c.submit(packet())); assertEquals(1, r.listFiles()!!.count { it.isDirectory })
    }
    @Test fun diskWritesRunOnlyOnDedicatedWorkerAndControlledOverheadIsRecorded() {
        val r = root(); val now = AtomicLong(100); var threadName = ""
        val c = ExactDetectorCapture(r, "test-session", now::get, write = { file, bytes -> threadName = Thread.currentThread().name; file.writeBytes(bytes) })
        val disabledStart = System.nanoTime()
        repeat(100_000) { assertFalse(c.wantsFrame(100)) }
        val disabledNs = (System.nanoTime() - disabledStart) / 100_000.0
        val pixels = ByteArray(1920 * 1080) { it.toByte() }
        repeat(10) { pixels.copyOf() }
        val start = System.nanoTime()
        var checksum = 0
        repeat(100) { checksum += pixels.copyOf()[123].toInt() }
        val copyMs = (System.nanoTime() - start) / 100.0 / 1e6
        val encodeStart = System.nanoTime(); val encoded = DetectorPgm.encode(pixels, 1920, 1080, 1920)
        val encodeMs = (System.nanoTime() - encodeStart) / 1e6
        assertArrayEquals(pixels, encoded.copyOfRange(encoded.size - pixels.size, encoded.size))
        arm(r, c, now); val submitStart = System.nanoTime()
        assertTrue(c.submit(packet().copy(identity = packet().identity.copy(width = 1920, height = 1080, stride = 1920), pixels = pixels)))
        val submitMs = (System.nanoTime() - submitStart) / 1e6
        c.close(); assertTrue(c.awaitClosed(3000)); assertEquals("DjiExactYWriter", threadName)
        System.getenv("DJI_CAPTURE_TEST_OUTPUT_DIR")?.let { output ->
            File(output, "jvm-overhead.json").writeText("{\"platform\":\"Windows JVM; not Android detector timing\",\"disabledAdmissionNsIncludingAssertion\":$disabledNs,\"copy1080pMeanMs\":$copyMs,\"pgm1080pEncodeMs\":$encodeMs,\"submit1080pMs\":$submitMs,\"checksum\":$checksum}")
        }
    }
}
