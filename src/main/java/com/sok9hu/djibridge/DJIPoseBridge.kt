package com.sok9hu.djibridge

import android.util.Log
import dji.sdk.keyvalue.key.DJIFlightControllerKey
import dji.sdk.keyvalue.key.DJIGimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.v5.manager.KeyManager
import org.json.JSONObject

/**
 * Collects the latest aircraft and gimbal pose values and exposes them to Unity
 * through a simple polling API.
 */
object DJIPoseBridge {

    private const val TAG = "DJIPoseBridge"
    private val listenerOwner = Any()

    private val aircraftLocationKey = KeyTools.createKey(DJIFlightControllerKey.KeyAircraftLocation)
    private val aircraftAltitudeKey = KeyTools.createKey(DJIFlightControllerKey.KeyAltitude)
    private val aircraftAttitudeKey = KeyTools.createKey(DJIFlightControllerKey.KeyAircraftAttitude)
    private val gimbalAttitudeKey =
        KeyTools.createKey(DJIGimbalKey.KeyGimbalAttitude, ComponentIndexType.LEFT_OR_MAIN)
    private val gimbalYawRelativeKey =
        KeyTools.createKey(DJIGimbalKey.KeyYawRelativeToAircraftHeading, ComponentIndexType.LEFT_OR_MAIN)

    @Volatile
    private var listenersAttached = false

    @Volatile
    private var state = PoseState()

    @JvmStatic
    fun onSdkStateChanged(source: String) {
        if (!DJIPlugin.isVideoBridgeReady()) {
            if (listenersAttached) {
                Log.i(TAG, "SDK no longer ready via $source; stopping pose listeners")
                detachListeners()
            }
            state = PoseState()
            return
        }

        state = state.copy(sdkReady = true)
        attachListenersIfNeeded(source)
    }

    @JvmStatic
    fun getLatestPoseJson(): String = buildPoseJson().toString()

    @JvmStatic
    fun isPoseAvailable(): Boolean = state.hasPose

    private fun attachListenersIfNeeded(source: String) {
        if (listenersAttached) return

        val keyManager = KeyManager.getInstance()
        listenersAttached = true
        Log.i(TAG, "Attaching pose listeners via $source")

        keyManager.listen(aircraftLocationKey, listenerOwner) { _, newValue ->
            applyAircraftLocation(newValue)
        }
        keyManager.listen(aircraftAltitudeKey, listenerOwner) { _, newValue ->
            applyAircraftAltitude(newValue)
        }
        keyManager.listen(aircraftAttitudeKey, listenerOwner) { _, newValue ->
            applyAircraftAttitude(newValue)
        }
        keyManager.listen(gimbalAttitudeKey, listenerOwner) { _, newValue ->
            applyGimbalAttitude(newValue)
        }
        keyManager.listen(gimbalYawRelativeKey, listenerOwner) { _, newValue ->
            applyGimbalYawRelative(newValue)
        }

        applyAircraftLocation(keyManager.getValue(aircraftLocationKey))
        applyAircraftAltitude(keyManager.getValue(aircraftAltitudeKey))
        applyAircraftAttitude(keyManager.getValue(aircraftAttitudeKey))
        applyGimbalAttitude(keyManager.getValue(gimbalAttitudeKey))
        applyGimbalYawRelative(keyManager.getValue(gimbalYawRelativeKey))
    }

    private fun detachListeners() {
        listenersAttached = false
        runCatching {
            KeyManager.getInstance().cancelListen(listenerOwner)
        }.onFailure { throwable ->
            Log.w(TAG, "Failed to detach pose listeners", throwable)
        }
    }

    private fun applyAircraftLocation(location: LocationCoordinate2D?) {
        if (location == null) return

        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            aircraft = state.aircraft.copy(
                latitude = location.latitude,
                longitude = location.longitude,
                hasLocation = true
            )
        ).withPoseAvailability()
    }

    private fun applyAircraftAltitude(altitude: Double?) {
        if (altitude == null) return

        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            aircraft = state.aircraft.copy(
                altitude = altitude,
                hasAltitude = true
            )
        ).withPoseAvailability()
    }

    private fun applyAircraftAttitude(attitude: Attitude?) {
        if (attitude == null) return

        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            aircraft = state.aircraft.copy(
                pitch = attitude.pitch,
                roll = attitude.roll,
                yaw = attitude.yaw,
                hasAttitude = true
            )
        ).withPoseAvailability()
    }

    private fun applyGimbalAttitude(attitude: Attitude?) {
        if (attitude == null) return

        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            gimbal = state.gimbal.copy(
                pitch = attitude.pitch,
                roll = attitude.roll,
                yaw = attitude.yaw,
                hasAttitude = true
            )
        ).withPoseAvailability()
    }

    private fun applyGimbalYawRelative(yawRelative: Double?) {
        if (yawRelative == null) return

        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            gimbal = state.gimbal.copy(
                yawRelativeToAircraftHeading = yawRelative,
                hasYawRelativeToAircraft = true
            )
        ).withPoseAvailability()
    }

    private fun buildPoseJson(): JSONObject {
        val snapshot = state

        val aircraft = JSONObject()
            .put("latitude", snapshot.aircraft.latitude)
            .put("longitude", snapshot.aircraft.longitude)
            .put("altitude", snapshot.aircraft.altitude)
            .put("pitch", snapshot.aircraft.pitch)
            .put("roll", snapshot.aircraft.roll)
            .put("yaw", snapshot.aircraft.yaw)
            .put("hasLocation", snapshot.aircraft.hasLocation)
            .put("hasAltitude", snapshot.aircraft.hasAltitude)
            .put("hasAttitude", snapshot.aircraft.hasAttitude)

        val gimbal = JSONObject()
            .put("pitch", snapshot.gimbal.pitch)
            .put("roll", snapshot.gimbal.roll)
            .put("yaw", snapshot.gimbal.yaw)
            .put("yawRelativeToAircraftHeading", snapshot.gimbal.yawRelativeToAircraftHeading)
            .put("hasAttitude", snapshot.gimbal.hasAttitude)
            .put("hasYawRelativeToAircraftHeading", snapshot.gimbal.hasYawRelativeToAircraft)

        return JSONObject()
            .put("sdkReady", snapshot.sdkReady)
            .put("hasPose", snapshot.hasPose)
            .put("timestampMs", snapshot.timestampMs)
            .put("aircraft", aircraft)
            .put("gimbal", gimbal)
    }

    private fun PoseState.withPoseAvailability(): PoseState =
        copy(
            hasPose = aircraft.hasLocation &&
                aircraft.hasAltitude &&
                aircraft.hasAttitude &&
                gimbal.hasAttitude
        )

    private data class PoseState(
        val sdkReady: Boolean = false,
        val hasPose: Boolean = false,
        val timestampMs: Long = 0L,
        val aircraft: AircraftPose = AircraftPose(),
        val gimbal: GimbalPose = GimbalPose()
    )

    private data class AircraftPose(
        val latitude: Double = 0.0,
        val longitude: Double = 0.0,
        val altitude: Double = 0.0,
        val pitch: Double = 0.0,
        val roll: Double = 0.0,
        val yaw: Double = 0.0,
        val hasLocation: Boolean = false,
        val hasAltitude: Boolean = false,
        val hasAttitude: Boolean = false
    )

    private data class GimbalPose(
        val pitch: Double = 0.0,
        val roll: Double = 0.0,
        val yaw: Double = 0.0,
        val yawRelativeToAircraftHeading: Double = 0.0,
        val hasAttitude: Boolean = false,
        val hasYawRelativeToAircraft: Boolean = false
    )
}
