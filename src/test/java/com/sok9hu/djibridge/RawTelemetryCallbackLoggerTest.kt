package com.sok9hu.djibridge

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class RawTelemetryCallbackLoggerTest {
    @Test
    fun queuePreservesOrderAndDropsOnlyAtBoundedCapacity() {
        val queue = RawTelemetryCallbackQueue<Int>(2)

        assertTrue(queue.tryEnqueue(100))
        assertTrue(queue.tryEnqueue(200))
        assertFalse(queue.tryEnqueue(300))
        assertEquals(100, queue.take())
        assertEquals(200, queue.take())

        queue.close()
        assertEquals(null, queue.take())
    }

    @Test
    fun closedQueueRetainsExistingRecordsForFlushButRejectsNewRecords() {
        val queue = RawTelemetryCallbackQueue<Int>(2)
        assertTrue(queue.tryEnqueue(42))
        queue.close()

        assertFalse(queue.tryEnqueue(43))
        assertEquals(42, queue.take())
    }

    @Test
    fun positionJsonPreservesSessionIdAndCallbackTimestamp() {
        val line = RawTelemetryCallbackJson.positionLine(
            "session-123",
            RawPositionRecord(123_456L, 1_700_000_000_000L, 47.5, 19.0, 120.0, true, "LEVEL_5", 20)
        )

        assertTrue(line.contains("\"telemetrySessionId\":\"session-123\""))
        assertTrue(line.contains("\"timestampNs\":123456"))
        assertTrue(line.contains("\"satelliteCount\":20"))
    }
}
