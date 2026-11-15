package com.sok9hu.djibridge

import com.sok9hu.djibridge.RcScreenshotPressGate.Decision.*
import com.sok9hu.djibridge.RcScreenshotPressGate.Key.RECORD
import com.sok9hu.djibridge.RcScreenshotPressGate.Key.SHUTTER
import org.junit.Assert.*
import org.junit.Test

class ScreenshotRegressionTest {
    @Test fun heldAndRepeatedCallbacksAreNotNewPresses() {
        val gate = RcScreenshotPressGate()
        assertEquals(INITIAL, gate.observe(SHUTTER, true, 0))
        assertEquals(DUPLICATE, gate.observe(SHUTTER, true, 500_000_000))
        assertEquals(RELEASE, gate.observe(SHUTTER, false, 600_000_000))
        assertEquals(PRESS, gate.observe(SHUTTER, true, 700_000_000))
        assertEquals(DUPLICATE, gate.observe(SHUTTER, true, 2_000_000_000))
        assertEquals(RELEASE, gate.observe(SHUTTER, false, 3_000_000_000))
        assertEquals(PRESS, gate.observe(SHUTTER, true, 3_300_000_000))
    }

    @Test fun overlappingAndStaggeredAliasesAreCoalesced() {
        val gate = RcScreenshotPressGate()
        assertEquals(INITIAL, gate.observe(SHUTTER, false, 0))
        assertEquals(INITIAL, gate.observe(RECORD, false, 1))
        assertEquals(PRESS, gate.observe(SHUTTER, true, 1_000_000_000))
        assertEquals(OTHER_KEY_HELD, gate.observe(RECORD, true, 1_020_000_000))
        assertEquals(RELEASE, gate.observe(SHUTTER, false, 1_040_000_000))
        assertEquals(RELEASE, gate.observe(RECORD, false, 1_050_000_000))
        assertEquals(COALESCED, gate.observe(RECORD, true, 1_070_000_000))
        assertEquals(RELEASE, gate.observe(RECORD, false, 1_080_000_000))
        assertEquals(PRESS, gate.observe(RECORD, true, 1_500_000_000))
    }

    @Test fun reconnectDoesNotTurnAHeldButtonIntoAPress() {
        val gate = RcScreenshotPressGate()
        assertEquals(INITIAL, gate.observe(RECORD, true, 100))
        assertEquals(RELEASE, gate.observe(RECORD, false, 200))
        assertEquals(DUPLICATE, gate.observe(RECORD, true, 199))
        assertEquals(PRESS, gate.observe(RECORD, true, 300))
    }

    @Test fun aliasJustAfterALongHoldReleaseDoesNotTriggerAgain() {
        val gate = RcScreenshotPressGate()
        assertEquals(INITIAL, gate.observe(SHUTTER, false, 0))
        assertEquals(INITIAL, gate.observe(RECORD, false, 1))
        assertEquals(PRESS, gate.observe(SHUTTER, true, 100_000_000))
        assertEquals(RELEASE, gate.observe(SHUTTER, false, 2_000_000_000))
        assertEquals(COALESCED, gate.observe(RECORD, true, 2_040_000_000))
        assertEquals(RELEASE, gate.observe(RECORD, false, 2_050_000_000))
        assertEquals(PRESS, gate.observe(RECORD, true, 2_300_000_000))
    }

    @Test fun asymmetricPixelsFlipOnlyTheRowsAndPreserveColors() {
        // Unity order: bottom row red/green, top row blue/white.
        val rgba = byteArrayOf(-1,0,0,-1, 0,-1,0,-1, 0,0,-1,-1, -1,-1,-1,0)
        val row = IntArray(2)
        assertTrue(ScreenshotPixels.validSize(2, 2, rgba.size))
        ScreenshotPixels.readTopDownRow(rgba, 2, 2, 0, row)
        assertArrayEquals(intArrayOf(0xff0000ff.toInt(), 0xffffffff.toInt()), row)
        ScreenshotPixels.readTopDownRow(rgba, 2, 2, 1, row)
        assertArrayEquals(intArrayOf(0xffff0000.toInt(), 0xff00ff00.toInt()), row)
        assertFalse(ScreenshotPixels.validSize(2, 2, 15))
        assertFalse(ScreenshotPixels.validSize(Int.MAX_VALUE, 2, 16))
    }
}
