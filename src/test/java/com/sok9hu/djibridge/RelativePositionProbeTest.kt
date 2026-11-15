package com.sok9hu.djibridge

import org.junit.Assert.*
import org.junit.Test

class RelativePositionProbeTest {
    private val second = 1_000_000_000L
    @Test fun disconnectedNeverRequests() {
        val p = RelativePositionProbe()
        repeat(20) { assertNull(p.tick(it * second, false, true, false).reason) }
    }
    @Test fun connectedReadWaitsTwoSecondsThenUsesOneHzWithoutCatchupBursts() {
        val p = RelativePositionProbe()
        assertNull(p.tick(0, true, false, false).reason)
        assertNull(p.tick(second, true, false, false).reason)
        assertEquals("CONNECTED_DELAYED_GET", p.tick(2 * second, true, false, false).reason)
        assertNull(p.tick(2 * second, true, false, false).reason)
        assertEquals("PERIODIC_GET", p.tick(20 * second, true, false, false).reason)
        assertNull(p.tick(20 * second + 1, true, false, false).reason)
    }
    @Test fun airborneGetOccursOnceAndDoesNotSendFlightCommands() {
        val p = RelativePositionProbe(); p.tick(0, true, false, false)
        p.tick(2 * second, true, false, false)
        assertEquals("AIRBORNE_GET", p.tick(3 * second, true, true, false).reason)
        assertEquals("PERIODIC_GET", p.tick(4 * second, true, true, false).reason)
    }
    @Test fun unknownFlightStateDoesNotFabricateAirborneEvent() {
        val p = RelativePositionProbe(); p.tick(0, true, null, false)
        p.tick(2 * second, true, null, false)
        assertEquals("PERIODIC_GET", p.tick(3 * second, true, null, false).reason)
        assertFalse(p.tick(30 * second, true, null, false).rebind)
    }
    @Test fun outstandingRequestBlocksFurtherRequests() {
        val p = RelativePositionProbe(); p.tick(0, true, false, false)
        assertNull(p.tick(20 * second, true, false, true).reason)
        assertEquals("CONNECTED_DELAYED_GET", p.tick(21 * second, true, false, false).reason)
    }
    @Test fun silentAirborneListenerIsReboundOnlyOncePerConnection() {
        val p = RelativePositionProbe(); p.tick(0, true, true, false)
        assertFalse(p.tick(7 * second, true, true, false).rebind)
        assertTrue(p.tick(8 * second, true, true, false).rebind)
        assertFalse(p.tick(30 * second, true, true, false).rebind)
        p.tick(31 * second, false, null, false); p.tick(32 * second, true, true, false)
        assertTrue(p.tick(40 * second, true, true, false).rebind)
        assertEquals(2, p.connectionGeneration)
    }
    @Test fun actualCallbackPreventsUnnecessaryRebindEvenForNullValue() {
        val p = RelativePositionProbe(); p.tick(0, true, true, false); p.callback()
        assertFalse(p.tick(20 * second, true, true, false).rebind)
    }
    @Test fun landingNeverTriggersAirborneRebindAndSecondTakeoffGetsItsOwnRead() {
        val p = RelativePositionProbe(); p.tick(0, true, true, false)
        p.tick(2 * second, true, true, false); p.tick(3 * second, true, true, false)
        assertFalse(p.tick(20 * second, true, false, false).rebind)
        assertEquals("AIRBORNE_GET", p.tick(21 * second, true, true, false).reason)
    }
    @Test fun callbackBeforeFirstTimerTickStillPreventsRebind() {
        val p = RelativePositionProbe(); p.callback(); p.tick(0, true, true, false)
        assertFalse(p.tick(20 * second, true, true, false).rebind)
    }
    @Test fun xyzValuesAndNullsArePreservedWithoutAxisAssumptions() {
        val xyz = RelativePositionDiagnosticJson.xyz(-2.5, 0.0, 17.25)
        assertEquals(true, xyz["valid"])
        assertEquals(-2.5, xyz["x"])
        assertEquals("UNVALIDATED", xyz["axesUnitsOrigin"])
        assertEquals(false, RelativePositionDiagnosticJson.xyz(null, null, null)["valid"])
        assertEquals(false, RelativePositionDiagnosticJson.xyz(Double.NaN, 1.0, 2.0)["valid"])
    }
    @Test fun serializationPreservesTimestampsErrorsAndEscapesControlCharacters() {
        val line = RelativePositionDiagnosticJson.encode(mapOf(
            "timestampNs" to 4793398671554241L, "errorCode" to "REQUEST_HANDLER_NOT_FOUND",
            "errorDescription" to "a\n\"b\"", "xyz" to RelativePositionDiagnosticJson.xyz(null, 1.0, Double.NaN)))
        assertTrue(line.contains("4793398671554241"))
        assertTrue(line.contains("REQUEST_HANDLER_NOT_FOUND"))
        assertTrue(line.contains("a\\n\\\"b\\\""))
        assertTrue(line.contains("\"z\":null")); assertFalse(line.contains("NaN"))
    }
}
