package com.sok9hu.djibridge

import android.util.Log
import dji.sdk.keyvalue.key.DJIBatteryKey
import dji.sdk.keyvalue.key.DJIFlightControllerKey
import dji.sdk.keyvalue.key.DJIGimbalKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.DJIProductKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sdk.keyvalue.value.common.XYZ
import dji.sdk.keyvalue.value.flightcontroller.GPSInterferenceState
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel
import dji.sdk.keyvalue.value.flightcontroller.HeightAboveSeaLevelMsg
import dji.sdk.keyvalue.value.flightcontroller.PositioningLevelMsg
import dji.v5.manager.KeyManager
import dji.v5.common.callback.CommonCallbacks
import org.json.JSONArray
import org.json.JSONObject

/**
 * Collects the latest aircraft and gimbal pose values and exposes them to Unity
 * through a simple polling API.
 */
object DJIPoseBridge {

    private const val TAG = "DJIPoseBridge"
    private const val MAXIMUM_VELOCITY_HISTORY_SAMPLES = 160
    private const val MAXIMUM_POSITION_HISTORY_SAMPLES = 160
    private val listenerOwner = Any()

    private val aircraftLocationKey = KeyTools.createKey(DJIFlightControllerKey.KeyAircraftLocation)
    private val aircraftLocation3DKey = KeyTools.createKey(DJIFlightControllerKey.KeyAircraftLocation3D)
    private val aircraftAltitudeKey = KeyTools.createKey(DJIFlightControllerKey.KeyAltitude)
    private val aircraftVelocityKey = KeyTools.createKey(DJIFlightControllerKey.KeyAircraftVelocity)
    private val gpsSatelliteCountKey = KeyTools.createKey(DJIFlightControllerKey.KeyGPSSatelliteCount)
    // The Mini 3 Pro exposes this as the aircraft smart-battery component.
    // No index is supplied so KeyTools resolves the primary aircraft battery.
    private val aircraftBatteryPercentKey = KeyTools.createKey(DJIBatteryKey.KeyChargeRemainingInPercent)
    private val gpsSignalLevelKey = KeyTools.createKey(DJIFlightControllerKey.KeyGPSSignalLevel)
    private val gpsIsValidKey = KeyTools.createKey(DJIFlightControllerKey.KeyGPSIsValid)
    private val gpsInterferenceStateKey = KeyTools.createKey(DJIFlightControllerKey.KeyGPSInterferenceState)
    private val fusionPositioningLevelKey = KeyTools.createKey(DJIFlightControllerKey.KeyFusionPositioningLevel)
    private val homeLocationKey = KeyTools.createKey(DJIFlightControllerKey.KeyHomeLocation)
    private val homeLocationSetKey = KeyTools.createKey(DJIFlightControllerKey.KeyIsHomeLocationSet)
    private val takeoffLocationAltitudeKey = KeyTools.createKey(DJIFlightControllerKey.KeyTakeoffLocationAltitude)
    private val heightAboveSeaLevelKey = KeyTools.createKey(DJIFlightControllerKey.KeyHeightAboveSeaLevel)
    private val relateToTakeOffPositionKey = KeyTools.createKey(DJIFlightControllerKey.KeyRelateToTakeOffPosition)
    private val aircraftAttitudeKey = KeyTools.createKey(DJIFlightControllerKey.KeyAircraftAttitude)
    private val flightControllerConnectionKey = KeyTools.createKey(DJIFlightControllerKey.KeyConnection)
    private val productTypeKey = KeyTools.createKey(DJIProductKey.KeyProductType)
    private val gimbalAttitudeKey =
        KeyTools.createKey(DJIGimbalKey.KeyGimbalAttitude, ComponentIndexType.LEFT_OR_MAIN)
    private val gimbalYawRelativeKey =
        KeyTools.createKey(DJIGimbalKey.KeyYawRelativeToAircraftHeading, ComponentIndexType.LEFT_OR_MAIN)

    @Volatile
    private var listenersAttached = false

    @Volatile
    private var state = PoseState()

    @Volatile
    private var nextPositionAuditLogElapsedMs = 0L

    @Volatile
    private var nextVelocityTimestampLogElapsedMs = 0L

    @Volatile
    private var nextPositionTimestampLogElapsedMs = 0L

    // These logs are emitted only when the logical satellite sample changes.
    @Volatile
    private var lastLoggedSatelliteCallbackReceived: Boolean? = null

    @Volatile
    private var lastLoggedSatelliteRaw: String? = null

    @Volatile
    private var lastLoggedSatelliteSerializedPresent: Boolean? = null

    @Volatile
    private var lastLoggedSatelliteSerializedRaw: String? = null

    @Volatile
    private var lastLoggedBatteryPercent: Int? = null

    @Volatile
    private var lastLoggedBatteryAvailable = false

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

    @JvmStatic
    fun startRelativePositionExperiment(sessionId: String): Boolean = DjiRelativePositionExperiment.start(sessionId)

    @JvmStatic
    fun drainRelativePositionExperiment(sessionId: String): String = DjiRelativePositionExperiment.drain(sessionId)

    @JvmStatic
    fun stopRelativePositionExperiment(sessionId: String): String = DjiRelativePositionExperiment.stop(sessionId)

    internal fun relativePositionDiagnosticReference(): Map<String, Any?> {
        val snapshot = state
        val audit = snapshot.positionAudit
        val p = audit.aircraftPositionHistory.lastOrNull()
        val v = audit.velocityHistory.lastOrNull()
        return linkedMapOf(
            "meaning" to "LATEST_KNOWN_AT_EVENT_NOT_TIMESTAMP_MATCHED",
            "location3D" to p?.let { mapOf("timestampNs" to it.timestampNs,
                "latitude" to it.latitude, "longitude" to it.longitude, "altitude" to it.altitude) },
            "velocityNed" to v?.let { mapOf("timestampNs" to it.timestampNs,
                "north" to it.northMetersPerSecond, "east" to it.eastMetersPerSecond, "down" to it.downMetersPerSecond) },
            "satelliteCountRaw" to audit.gpsSatelliteCountRaw,
            "gpsSignalLevelRaw" to audit.gpsSignalLevelRaw, "gpsValidRaw" to audit.gpsIsValidRaw,
            "aircraftAttitude" to if (snapshot.aircraft.hasAttitude) mapOf(
                "timestampNs" to snapshot.aircraftAttitudeArrivalTimestampNs,
                "pitch" to snapshot.aircraft.pitch, "roll" to snapshot.aircraft.roll, "yaw" to snapshot.aircraft.yaw) else null,
            "gimbalAttitude" to if (snapshot.gimbal.hasAttitude) mapOf(
                "timestampNs" to snapshot.gimbalAttitudeArrivalTimestampNs,
                "pitch" to snapshot.gimbal.pitch, "roll" to snapshot.gimbal.roll, "yaw" to snapshot.gimbal.yaw) else null)
    }

    @JvmStatic
    fun startRawTelemetryCallbackLogging(telemetrySessionId: String): Boolean {
        val application = DJIPlugin.applicationContextOrNull() ?: run {
            Log.w(TAG, "DJI_POSITION_CALLBACK_LOG_ERROR reason=APPLICATION_CONTEXT_UNAVAILABLE")
            return false
        }
        return DjiRawTelemetryCallbackLogger.start(application, telemetrySessionId)
    }

    @JvmStatic
    fun stopRawTelemetryCallbackLogging() {
        DjiRawTelemetryCallbackLogger.stop()
    }

    private fun attachListenersIfNeeded(source: String) {
        if (listenersAttached) return

        val keyManager = KeyManager.getInstance()
        listenersAttached = true
        Log.i(TAG, "Attaching pose listeners via $source")

        listenPositionKey(keyManager, "KeyAircraftLocation", aircraftLocationKey, ::applyAircraftLocation)
        listenTimestampedPositionKey(keyManager, "KeyAircraftLocation3D", aircraftLocation3DKey) { value, timestampNs, callbackReceived ->
            applyAircraftLocation3D(value, timestampNs, callbackReceived)
        }
        listenTimestampedPositionKey(keyManager, "KeyAltitude", aircraftAltitudeKey) { value, timestampNs, callbackReceived ->
            applyAircraftAltitude(value, timestampNs, callbackReceived)
        }
        listenAircraftVelocityKey(keyManager)
        listenGpsSatelliteCountKey(keyManager)
        listenAircraftBatteryPercentKey(keyManager)
        listenPositionKey(keyManager, "KeyGPSSignalLevel", gpsSignalLevelKey, ::applyGpsSignalLevel)
        listenPositionKey(keyManager, "KeyGPSIsValid", gpsIsValidKey, ::applyGpsIsValid)
        listenTimestampedPositionKey(
            keyManager,
            "KeyGPSInterferenceState",
            gpsInterferenceStateKey,
            ::applyGpsInterferenceCallbackError
        ) { value, timestampNs, callbackReceived ->
            applyGpsInterferenceState(value, timestampNs, callbackReceived)
        }
        listenTimestampedPositionKey(
            keyManager,
            "KeyFusionPositioningLevel",
            fusionPositioningLevelKey,
            ::applyFusionPositioningLevelCallbackError
        ) { value, timestampNs, callbackReceived ->
            applyFusionPositioningLevel(value, timestampNs, callbackReceived)
        }
        listenPositionKey(keyManager, "KeyHomeLocation", homeLocationKey, ::applyHomeLocation)
        listenPositionKey(keyManager, "KeyIsHomeLocationSet", homeLocationSetKey, ::applyHomeLocationSet)
        listenPositionKey(keyManager, "KeyTakeoffLocationAltitude", takeoffLocationAltitudeKey, ::applyTakeoffLocationAltitude)
        listenTimestampedPositionKey(keyManager, "KeyHeightAboveSeaLevel", heightAboveSeaLevelKey) { value, timestampNs, callbackReceived ->
            applyHeightAboveSeaLevel(value, timestampNs, callbackReceived)
        }
        listenRelativePositionKey(keyManager)
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
        applyAircraftLocation3D(keyManager.getValue(aircraftLocation3DKey))
        applyAircraftAltitude(keyManager.getValue(aircraftAltitudeKey))
        applyAircraftVelocity(keyManager.getValue(aircraftVelocityKey))
        // An initial KeyManager read may expose an SDK default such as zero.
        // It is diagnostic-only until the listener has delivered a real sample.
        applyGpsSatelliteCount(keyManager.getValue(gpsSatelliteCountKey), callbackReceived = false)
        // A cached SDK read is not sufficient to distinguish an absent value
        // from a real 0%. The listener establishes battery availability.
        applyAircraftBatteryPercent(keyManager.getValue(aircraftBatteryPercentKey), callbackReceived = false)
        applyGpsSignalLevel(keyManager.getValue(gpsSignalLevelKey))
        applyGpsIsValid(keyManager.getValue(gpsIsValidKey))
        applyGpsInterferenceState(keyManager.getValue(gpsInterferenceStateKey))
        applyFusionPositioningLevel(keyManager.getValue(fusionPositioningLevelKey))
        applyHomeLocation(keyManager.getValue(homeLocationKey))
        applyHomeLocationSet(keyManager.getValue(homeLocationSetKey))
        applyTakeoffLocationAltitude(keyManager.getValue(takeoffLocationAltitudeKey))
        applyHeightAboveSeaLevel(keyManager.getValue(heightAboveSeaLevelKey))
        applyRelativeToTakeoffPosition(keyManager.getValue(relateToTakeOffPositionKey))
        recordPositionKeySupport(keyManager)
        applyAircraftAttitude(keyManager.getValue(aircraftAttitudeKey))
        applyGimbalAttitude(keyManager.getValue(gimbalAttitudeKey))
        applyGimbalYawRelative(keyManager.getValue(gimbalYawRelativeKey))
        logTelemetryDataPathDiagnostics(keyManager, source)
    }

    private fun listenRelativePositionKey(keyManager: KeyManager) {
        listenTimestampedPositionKey(keyManager, "KeyRelateToTakeOffPosition", relateToTakeOffPositionKey) { value, timestampNs, callbackReceived ->
            runCatching { DjiRelativePositionExperiment.listenerValue(value, timestampNs) }
                .onFailure { Log.w(TAG, "RELATIVE_POSITION_DIAGNOSTIC_OBSERVER_FAILED", it) }
            applyRelativeToTakeoffPosition(value, timestampNs, callbackReceived)
        }
    }

    internal fun rebindRelativePositionListenerForExperiment(): Boolean {
        if (!listenersAttached || !DJIPlugin.isVideoBridgeReady()) return false
        val manager = KeyManager.getInstance()
        manager.cancelListen(relateToTakeOffPositionKey, listenerOwner)
        listenRelativePositionKey(manager)
        return DjiRelativePositionExperiment.lastBindReturned()
    }

    private fun <T> listenPositionKey(
        keyManager: KeyManager,
        keyName: String,
        key: DJIKey<T>,
        onValue: (T?) -> Unit
    ) {
        logKeyConstruction("POSITION_KEY", keyName, key, listenerRegistered = true)
        Log.i(
            TAG,
            "DJI_POSITION_LISTENER_BIND key=$keyName connected=${DJIPlugin.isVideoBridgeReady()} " +
                "timestampElapsedNs=${android.os.SystemClock.elapsedRealtimeNanos()} state=${DJIPlugin.describeState()}"
        )
        runCatching {
            keyManager.listen(key, listenerOwner) { oldValue, newValue ->
                Log.i(
                    TAG,
                    "DJI_POSITION_LISTENER key=$keyName oldValue=${formatDiagnosticValue(oldValue)} " +
                        "newValue=${formatDiagnosticValue(newValue)} callbackReceived=true"
                )
                onValue(newValue)
            }
        }.onFailure { throwable ->
            Log.e(TAG, "DJI_POSITION_LISTENER_BIND_FAILED key=$keyName error=${throwable.message}", throwable)
        }
    }

    private fun <T> listenTimestampedPositionKey(
        keyManager: KeyManager,
        keyName: String,
        key: DJIKey<T>,
        onListenerError: (Throwable) -> Unit = {},
        onValue: (T?, Long, Boolean) -> Unit
    ) {
        logKeyConstruction("POSITION_KEY", keyName, key, listenerRegistered = true)
        runCatching {
            keyManager.listen(key, listenerOwner) { oldValue, newValue ->
                val callbackTimestampNs = android.os.SystemClock.elapsedRealtimeNanos()
                Log.i(
                    TAG,
                    "DJI_POSITION_LISTENER key=$keyName oldValue=${formatDiagnosticValue(oldValue)} " +
                        "newValue=${formatDiagnosticValue(newValue)} callbackReceived=true timestampNs=$callbackTimestampNs"
                )
                onValue(newValue, callbackTimestampNs, true)
            }
            if (keyName == "KeyRelateToTakeOffPosition")
                DjiRelativePositionExperiment.listenerBind(true)
        }.onFailure { throwable ->
            Log.e(TAG, "DJI_POSITION_LISTENER_BIND_FAILED key=$keyName error=${throwable.message}", throwable)
            if (keyName == "KeyRelateToTakeOffPosition")
                DjiRelativePositionExperiment.listenerBind(false, throwable.toString())
            onListenerError(throwable)
        }
    }

    private fun listenGpsSatelliteCountKey(keyManager: KeyManager) {
        logKeyConstruction("POSITION_KEY", "KeyGPSSatelliteCount", gpsSatelliteCountKey, listenerRegistered = true)
        Log.i(
            TAG,
            "DJI_POSITION_LISTENER_BIND key=KeyGPSSatelliteCount connected=${DJIPlugin.isVideoBridgeReady()} " +
                "timestampElapsedNs=${android.os.SystemClock.elapsedRealtimeNanos()} state=${DJIPlugin.describeState()}"
        )
        runCatching {
            keyManager.listen(gpsSatelliteCountKey, listenerOwner) { _, newValue ->
                applyGpsSatelliteCount(newValue, callbackReceived = true)
            }
        }.onFailure { throwable ->
            Log.e(TAG, "DJI_POSITION_LISTENER_BIND_FAILED key=KeyGPSSatelliteCount error=${throwable.message}", throwable)
        }
    }

    private fun listenAircraftVelocityKey(keyManager: KeyManager) {
        logKeyConstruction("POSITION_KEY", "KeyAircraftVelocity", aircraftVelocityKey, listenerRegistered = true)
        Log.i(
            TAG,
            "DJI_POSITION_LISTENER_BIND key=KeyAircraftVelocity connected=${DJIPlugin.isVideoBridgeReady()} " +
                "timestampElapsedNs=${android.os.SystemClock.elapsedRealtimeNanos()} state=${DJIPlugin.describeState()}"
        )
        runCatching {
            keyManager.listen(aircraftVelocityKey, listenerOwner) { oldValue, newValue ->
                // This belongs exclusively to the velocity KeyManager callback.
                val callbackTimestampNs = android.os.SystemClock.elapsedRealtimeNanos()
                Log.i(
                    TAG,
                    "DJI_POSITION_LISTENER key=KeyAircraftVelocity oldValue=${formatDiagnosticValue(oldValue)} " +
                        "newValue=${formatDiagnosticValue(newValue)} callbackReceived=true timestampNs=$callbackTimestampNs"
                )
                applyAircraftVelocity(newValue, callbackTimestampNs, callbackReceived = true)
            }
        }.onFailure { throwable ->
            Log.e(TAG, "DJI_POSITION_LISTENER_BIND_FAILED key=KeyAircraftVelocity error=${throwable.message}", throwable)
        }
    }

    private fun listenAircraftBatteryPercentKey(keyManager: KeyManager) {
        logKeyConstruction(
            "BATTERY_KEY",
            "KeyChargeRemainingInPercent",
            aircraftBatteryPercentKey,
            "DJIBatteryKey",
            listenerRegistered = true
        )
        runCatching {
            keyManager.listen(aircraftBatteryPercentKey, listenerOwner) { _, newValue ->
                applyAircraftBatteryPercent(newValue, callbackReceived = true)
            }
        }.onFailure { throwable ->
            Log.e(TAG, "DJI_BATTERY_LISTENER_BIND_FAILED key=KeyChargeRemainingInPercent error=${throwable.message}", throwable)
        }
    }

    private fun logTelemetryDataPathDiagnostics(keyManager: KeyManager, source: String) {
        Log.i(
            TAG,
            "DJI_POSITION_CONNECTION_AUDIT productModelPending=KeyProductType " +
                "aircraftConnected=${DJIPlugin.isVideoBridgeReady()} flightControllerConnectionPending=KeyConnection " +
                "sdkState=${DJIPlugin.describeState()} attachSource=$source"
        )
        logKeyConstruction("KNOWN_GOOD_KEY", "KeyAircraftAttitude", aircraftAttitudeKey, "DJIFlightControllerKey", listenerRegistered = true)
        logKeyConstruction("POSITION_KEY", "KeyAircraftLocation3D", aircraftLocation3DKey, "DJIFlightControllerKey", listenerRegistered = true)
        logKeyConstruction("POSITION_KEY", "KeyGPSIsValid", gpsIsValidKey, "DJIFlightControllerKey", listenerRegistered = true)
        logKeyConstruction("POSITION_KEY", "KeyGPSSatelliteCount", gpsSatelliteCountKey, "DJIFlightControllerKey", listenerRegistered = true)
        logKeyConstruction("QUALITY_KEY", "KeyGPSInterferenceState", gpsInterferenceStateKey, "DJIFlightControllerKey", listenerRegistered = true)
        logKeyConstruction("QUALITY_KEY", "KeyFusionPositioningLevel", fusionPositioningLevelKey, "DJIFlightControllerKey", listenerRegistered = true)
        logKeyConstruction("CONNECTION_KEY", "KeyFlightControllerConnection", flightControllerConnectionKey, "DJIFlightControllerKey", listenerRegistered = false)
        logKeyConstruction("CONNECTION_KEY", "KeyProductType", productTypeKey, "DJIProductKey", listenerRegistered = false)

        requestDiagnosticValue(keyManager, "KNOWN_GOOD_KEY:KeyAircraftAttitude", aircraftAttitudeKey)
        requestDiagnosticValue(keyManager, "KeyAircraftLocation3D", aircraftLocation3DKey)
        requestDiagnosticValue(keyManager, "KeyGPSIsValid", gpsIsValidKey)
        requestDiagnosticValue(keyManager, "KeyGPSSignalLevel", gpsSignalLevelKey)
        requestDiagnosticValue(keyManager, "KeyGPSSatelliteCount", gpsSatelliteCountKey)
        requestDiagnosticValue(keyManager, "KeyGPSInterferenceState", gpsInterferenceStateKey)
        requestDiagnosticValue(keyManager, "KeyFusionPositioningLevel", fusionPositioningLevelKey)
        requestDiagnosticValue(keyManager, "KeyHomeLocation", homeLocationKey)
        requestDiagnosticValue(keyManager, "KeyRelateToTakeOffPosition", relateToTakeOffPositionKey)
        requestDiagnosticValue(keyManager, "KeyHeightAboveSeaLevel", heightAboveSeaLevelKey)
        requestDiagnosticValue(keyManager, "KeyAircraftVelocity", aircraftVelocityKey)
        requestDiagnosticValue(keyManager, "KeyFlightControllerConnection", flightControllerConnectionKey)
        requestDiagnosticValue(keyManager, "KeyProductType", productTypeKey)
    }

    private fun <T> requestDiagnosticValue(keyManager: KeyManager, keyName: String, key: DJIKey<T>) {
        keyManager.getValue(key, object : CommonCallbacks.CompletionCallbackWithParam<T> {
            override fun onSuccess(value: T) {
                Log.i(TAG, "DJI_POSITION_DIRECT_GET key=$keyName value=${formatDiagnosticValue(value)} error=none")
            }

            override fun onFailure(error: dji.v5.common.error.IDJIError) {
                Log.w(
                    TAG,
                    "DJI_POSITION_DIRECT_GET key=$keyName value=null " +
                        "errorCode=${error.errorCode()} errorDescription=${error.description()} " +
                        "errorType=${error.errorType()} innerCode=${error.innerCode()} hint=${error.hint()}"
                )
            }
        })
    }

    private fun <T> logKeyConstruction(
        kind: String,
        keyName: String,
        key: DJIKey<T>,
        declaringKeyClass: String = "DJIFlightControllerKey",
        listenerRegistered: Boolean
    ) {
        val info = key.keyInfo
        val valueType = info.typeConverter?.classType?.name ?: "unknown"
        Log.i(
            TAG,
            "$kind key=$keyName declaringKeyClass=$declaringKeyClass keyClass=${key.javaClass.name} " +
                "identifier=${key.keyIdentifier} componentOwner=${key.componentType} componentIndex=${key.componentIndex} " +
                "subComponentType=${key.subComponentType} subComponentIndex=${key.subComponentIndex} " +
                "valueType=$valueType canGet=${key.canGet()} canListen=${key.canListen()} " +
                "listenerRegistered=$listenerRegistered"
        )
    }

    private fun formatDiagnosticValue(value: Any?): String = value?.toString() ?: "null"

    private fun String?.toNullableBoolean(): Boolean? = when (this?.lowercase()) {
        "true" -> true
        "false" -> false
        else -> null
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
        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            aircraft = state.aircraft.copy(
                latitude = location?.latitude ?: 0.0,
                longitude = location?.longitude ?: 0.0,
                hasLocation = isValidCoordinate(location?.latitude, location?.longitude)
            ),
            positionAudit = state.positionAudit.copy(
                aircraftLocation2DStatus = valueStatus(
                    location != null,
                    location != null && isValidCoordinate(location.latitude, location.longitude)
                ),
                aircraftLocation2DRaw = location?.toString() ?: "null"
            )
        ).withPoseAvailability()
        logPositionAuditIfDue()
    }

    private fun applyAircraftAltitude(
        altitude: Double?,
        callbackTimestampNs: Long = 0L,
        callbackReceived: Boolean = false
    ) {
        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            aircraft = state.aircraft.copy(
                altitude = altitude ?: 0.0,
                hasAltitude = altitude != null && altitude.isFinite()
            ),
            positionAudit = state.positionAudit.copy(
                keyAltitudeStatus = valueStatus(altitude != null, altitude != null && altitude.isFinite()),
                keyAltitudeRaw = altitude?.toString() ?: "null",
                altitudeHistory = appendScalarHistory(
                    state.positionAudit.altitudeHistory,
                    callbackTimestampNs,
                    altitude,
                    callbackReceived
                )
            )
        ).withPoseAvailability()
        logPositionAuditIfDue()
    }

    private fun applyAircraftLocation3D(
        location: LocationCoordinate3D?,
        callbackTimestampNs: Long = 0L,
        callbackReceived: Boolean = false
    ) {
        val valid = location != null && isValidCoordinate(location.latitude, location.longitude) && location.altitude.isFinite()
        updatePositionAudit {
            it.copy(
                aircraftLocation3DStatus = valueStatus(location != null, valid),
                aircraftLocation3DRaw = location?.toString() ?: "null",
                aircraftPositionHistory = if (callbackReceived && valid && callbackTimestampNs > 0L) {
                    appendPositionHistory(
                        it.aircraftPositionHistory,
                        AircraftPositionSample(callbackTimestampNs, location!!.latitude, location.longitude, location.altitude)
                    )
                } else it.aircraftPositionHistory
            )
        }
        if (callbackReceived && valid && callbackTimestampNs > 0L) {
            state.positionAudit.aircraftPositionHistory.lastOrNull()?.let(::logPositionTimestampIfDue)
            val audit = state.positionAudit
            DjiRawTelemetryCallbackLogger.enqueuePosition(
                timestampNs = callbackTimestampNs,
                utcTimestampMs = System.currentTimeMillis(),
                latitude = location!!.latitude,
                longitude = location.longitude,
                altitude = location.altitude,
                gpsValid = audit.gpsIsValidRaw.toNullableBoolean(),
                gpsSignalLevel = audit.gpsSignalLevelRaw.takeUnless { it == "null" },
                satelliteCount = if (audit.hasGpsSatelliteCount) audit.gpsSatelliteCountRaw.toIntOrNull() else null
            )
        }
    }

    private fun applyAircraftVelocity(
        velocity: Velocity3D?,
        callbackTimestampNs: Long = 0L,
        callbackReceived: Boolean = false
    ) {
        val valid = velocity != null && velocity.x.isFinite() && velocity.y.isFinite() && velocity.z.isFinite()
        var appendedSample: VelocitySample? = null
        var rejectionReason: String? = null
        updatePositionAudit { audit ->
            var updatedAudit = audit.copy(
                aircraftVelocityStatus = valueStatus(velocity != null, valid),
                aircraftVelocityRaw = velocity?.toString() ?: "null",
                hasAircraftVelocity = valid,
                aircraftVelocityX = if (valid) velocity!!.x.toFloat() else 0f,
                aircraftVelocityY = if (valid) velocity!!.y.toFloat() else 0f,
                aircraftVelocityZ = if (valid) velocity!!.z.toFloat() else 0f
            )

            if (!callbackReceived || !valid)
                return@updatePositionAudit updatedAudit

            val previousTimestampNs = audit.velocityHistory.lastOrNull()?.timestampNs ?: Long.MIN_VALUE
            when {
                callbackTimestampNs <= 0L -> rejectionReason = "ZERO_TIMESTAMP"
                callbackTimestampNs == previousTimestampNs -> rejectionReason = "DUPLICATE_TIMESTAMP"
                callbackTimestampNs < previousTimestampNs -> rejectionReason = "NON_MONOTONIC_TIMESTAMP"
                else -> {
                    val sample = VelocitySample(
                        timestampNs = callbackTimestampNs,
                        northMetersPerSecond = velocity!!.x,
                        eastMetersPerSecond = velocity.y,
                        downMetersPerSecond = velocity.z
                    )
                    appendedSample = sample
                    updatedAudit = updatedAudit.copy(
                        latestVelocityTimestampNs = callbackTimestampNs,
                        velocityHistory = (audit.velocityHistory + sample).takeLast(MAXIMUM_VELOCITY_HISTORY_SAMPLES)
                    )
                }
            }
            updatedAudit
        }

        rejectionReason?.let { reason ->
            Log.w(TAG, "DJI_VELOCITY_HISTORY_REJECTED reason=$reason timestampNs=$callbackTimestampNs")
        }
        appendedSample?.let { sample ->
            logVelocityTimestampIfDue(sample, state.positionAudit.velocityHistory.size)
        }
        if (callbackReceived && valid && callbackTimestampNs > 0L) {
            // Raw capture keeps repeated values as separate callbacks; the
            // bounded history's timestamp de-duplication remains independent.
            DjiRawTelemetryCallbackLogger.enqueueVelocity(
                timestampNs = callbackTimestampNs,
                utcTimestampMs = System.currentTimeMillis(),
                northMetersPerSecond = velocity!!.x,
                eastMetersPerSecond = velocity.y,
                downMetersPerSecond = velocity.z
            )
        }
    }

    private fun logVelocityTimestampIfDue(sample: VelocitySample, historyCount: Int) {
        val nowElapsedMs = android.os.SystemClock.elapsedRealtime()
        if (nowElapsedMs < nextVelocityTimestampLogElapsedMs) return

        nextVelocityTimestampLogElapsedMs = nowElapsedMs + 1000L
        val speed = kotlin.math.sqrt(
            sample.northMetersPerSecond * sample.northMetersPerSecond +
                sample.eastMetersPerSecond * sample.eastMetersPerSecond +
                sample.downMetersPerSecond * sample.downMetersPerSecond
        )
        Log.i(
            TAG,
            "DJI_VELOCITY_TIMESTAMP timestampNs=${sample.timestampNs} " +
                "north=${sample.northMetersPerSecond} east=${sample.eastMetersPerSecond} down=${sample.downMetersPerSecond} " +
                "speed=$speed historyCount=$historyCount newestHistoryTimestampNs=${sample.timestampNs} " +
                "clockDomain=ANDROID_MONOTONIC VELOCITY_UNIT_VALIDATION=PENDING"
        )
    }

    private fun applyGpsSatelliteCount(satelliteCount: Int?, callbackReceived: Boolean) {
        val rawValue = satelliteCount?.toString() ?: "null"
        logSatelliteRawIfChanged(callbackReceived, rawValue)
        updatePositionAudit {
            it.copy(
                gpsSatelliteCountStatus = valueStatus(satelliteCount != null, satelliteCount != null && satelliteCount > 0),
                gpsSatelliteCountRaw = rawValue,
                // Presence is independent of GNSS validity: an explicit callback
                // reporting zero must still render as SAT 0, while no callback is SAT --.
                hasGpsSatelliteCount = it.hasGpsSatelliteCount || callbackReceived
            )
        }
    }

    private fun applyAircraftBatteryPercent(percent: Int?, callbackReceived: Boolean) {
        val valid = percent != null && percent in 0..100
        val prior = state.aircraftBattery
        val updated = AircraftBatteryState(
            percent = if (valid) percent!! else 0,
            // Presence is callback-driven so an SDK default integer cannot be
            // rendered as a real battery percentage. A later unavailable
            // callback must likewise become BAT -- rather than a false 0%.
            hasPercent = if (callbackReceived) valid else prior.hasPercent
        )
        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            aircraftBattery = updated
        )

        if (!callbackReceived ||
            (lastLoggedBatteryAvailable == updated.hasPercent && lastLoggedBatteryPercent == if (updated.hasPercent) updated.percent else null)
        ) {
            return
        }

        lastLoggedBatteryAvailable = updated.hasPercent
        lastLoggedBatteryPercent = if (updated.hasPercent) updated.percent else null
        Log.i(
            TAG,
            "DJI_BATTERY_PERCENT_CHANGED value=${if (updated.hasPercent) updated.percent else "UNAVAILABLE"} " +
                "source=KeyChargeRemainingInPercent"
        )
    }

    private fun logSatelliteRawIfChanged(callbackReceived: Boolean, rawValue: String) {
        if (lastLoggedSatelliteCallbackReceived == callbackReceived && lastLoggedSatelliteRaw == rawValue) return

        lastLoggedSatelliteCallbackReceived = callbackReceived
        lastLoggedSatelliteRaw = rawValue
        val audit = state.positionAudit
        Log.i(
            TAG,
            "DJI_SAT_RAW callbackReceived=$callbackReceived value=$rawValue " +
                "gpsIsValid=${audit.gpsIsValidRaw} gpsSignalLevel=${audit.gpsSignalLevelRaw}"
        )
    }

    private fun applyGpsSignalLevel(signalLevel: GPSSignalLevel?) {
        val valid = signalLevel != null &&
            signalLevel != GPSSignalLevel.UNKNOWN &&
            signalLevel != GPSSignalLevel.LEVEL_NONE &&
            signalLevel != GPSSignalLevel.LEVEL_0
        updatePositionAudit {
            it.copy(
                gpsSignalLevelStatus = valueStatus(signalLevel != null, valid),
                gpsSignalLevelRaw = signalLevel?.name ?: "null"
            )
        }
    }

    private fun applyGpsIsValid(gpsIsValid: Boolean?) {
        updatePositionAudit {
            it.copy(
                gpsIsValidStatus = valueStatus(gpsIsValid != null, gpsIsValid == true),
                gpsIsValidRaw = gpsIsValid?.toString() ?: "null"
            )
        }
    }

    private fun applyGpsInterferenceState(
        interferenceState: GPSInterferenceState?,
        callbackTimestampNs: Long = 0L,
        callbackReceived: Boolean = false
    ) {
        val rawValue = interferenceState?.name ?: "null"
        updatePositionAudit {
            it.copy(
                gpsInterferenceStateStatus = valueStatus(interferenceState != null, interferenceState != null),
                gpsInterferenceStateRaw = rawValue,
                gpsInterferenceStateError = null,
                gpsInterferenceStateHistory = if (callbackReceived && callbackTimestampNs > 0L) {
                    appendHorizontalQualityHistory(
                        it.gpsInterferenceStateHistory,
                        HorizontalQualitySample(callbackTimestampNs, rawValue, valueStatus(interferenceState != null, interferenceState != null))
                    )
                } else it.gpsInterferenceStateHistory
            )
        }
    }

    private fun applyFusionPositioningLevel(
        positioningLevel: PositioningLevelMsg?,
        callbackTimestampNs: Long = 0L,
        callbackReceived: Boolean = false
    ) {
        val rawValue = positioningLevel?.level?.name ?: "null"
        val valid = positioningLevel?.level != null
        updatePositionAudit {
            it.copy(
                fusionPositioningLevelStatus = valueStatus(positioningLevel != null, valid),
                fusionPositioningLevelRaw = rawValue,
                fusionPositioningLevelError = null,
                fusionPositioningLevelHistory = if (callbackReceived && callbackTimestampNs > 0L) {
                    appendHorizontalQualityHistory(
                        it.fusionPositioningLevelHistory,
                        HorizontalQualitySample(callbackTimestampNs, rawValue, valueStatus(positioningLevel != null, valid))
                    )
                } else it.fusionPositioningLevelHistory
            )
        }
    }

    private fun applyGpsInterferenceCallbackError(throwable: Throwable) {
        updateHorizontalQualityCallbackError(
            source = "KeyGPSInterferenceState",
            timestampNs = android.os.SystemClock.elapsedRealtimeNanos(),
            error = throwable.message ?: throwable.javaClass.simpleName
        ) { audit, sample ->
            audit.copy(
                gpsInterferenceStateStatus = "CALLBACK_ERROR",
                gpsInterferenceStateError = sample.error,
                gpsInterferenceStateHistory = appendHorizontalQualityHistory(audit.gpsInterferenceStateHistory, sample)
            )
        }
    }

    private fun applyFusionPositioningLevelCallbackError(throwable: Throwable) {
        updateHorizontalQualityCallbackError(
            source = "KeyFusionPositioningLevel",
            timestampNs = android.os.SystemClock.elapsedRealtimeNanos(),
            error = throwable.message ?: throwable.javaClass.simpleName
        ) { audit, sample ->
            audit.copy(
                fusionPositioningLevelStatus = "CALLBACK_ERROR",
                fusionPositioningLevelError = sample.error,
                fusionPositioningLevelHistory = appendHorizontalQualityHistory(audit.fusionPositioningLevelHistory, sample)
            )
        }
    }

    private fun updateHorizontalQualityCallbackError(
        source: String,
        timestampNs: Long,
        error: String,
        update: (PositionAuditState, HorizontalQualitySample) -> PositionAuditState
    ) {
        val sample = HorizontalQualitySample(timestampNs, "null", "CALLBACK_ERROR", error)
        updatePositionAudit { audit -> update(audit, sample) }
        Log.w(TAG, "DJI_HORIZONTAL_QUALITY_CALLBACK_ERROR source=$source timestampNs=$timestampNs error=$error")
    }

    private fun applyHomeLocation(location: LocationCoordinate2D?) {
        val valid = location != null && isValidCoordinate(location.latitude, location.longitude)
        updatePositionAudit {
            it.copy(
                homeLocationStatus = valueStatus(location != null, valid),
                homeLocationRaw = location?.toString() ?: "null"
            )
        }
    }

    private fun applyHomeLocationSet(isSet: Boolean?) {
        updatePositionAudit {
            it.copy(
                homeLocationSetStatus = valueStatus(isSet != null, isSet == true),
                homeLocationSetRaw = isSet?.toString() ?: "null"
            )
        }
    }

    private fun applyTakeoffLocationAltitude(altitude: Double?) {
        updatePositionAudit {
            it.copy(
                takeoffLocationAltitudeStatus = valueStatus(altitude != null, altitude != null && altitude.isFinite()),
                takeoffLocationAltitudeRaw = altitude?.toString() ?: "null"
            )
        }
    }

    private fun applyHeightAboveSeaLevel(
        height: HeightAboveSeaLevelMsg?,
        callbackTimestampNs: Long = 0L,
        callbackReceived: Boolean = false
    ) {
        val valid = height != null &&
            height.height.isFinite() &&
            height.quality != null &&
            height.quality.value() > 0
        updatePositionAudit {
            it.copy(
                heightAboveSeaLevelStatus = valueStatus(height != null, valid),
                heightAboveSeaLevelRaw = height?.toString() ?: "null",
                heightAboveSeaLevelCallbackHistory = if (callbackReceived && callbackTimestampNs > 0L) {
                    appendHeightAboveSeaLevelCallbackHistory(
                        it.heightAboveSeaLevelCallbackHistory,
                        HeightAboveSeaLevelCallbackSample(
                            timestampNs = callbackTimestampNs,
                            hasHeight = height != null,
                            height = height?.height,
                            hasQuality = height?.quality != null,
                            quality = height?.quality?.value(),
                            status = when {
                                height == null -> "NO_SAMPLE"
                                height.quality == null -> "QUALITY_UNAVAILABLE"
                                height.quality.value() <= 0 -> "QUALITY_NON_POSITIVE"
                                !height.height.isFinite() -> "INVALID_HEIGHT"
                                else -> "VALID"
                            }
                        )
                    )
                } else it.heightAboveSeaLevelCallbackHistory,
                heightAboveSeaLevelHistory = if (callbackReceived && valid && callbackTimestampNs > 0L) {
                    appendHeightAboveSeaLevelHistory(
                        it.heightAboveSeaLevelHistory,
                        HeightAboveSeaLevelSample(callbackTimestampNs, height!!.height, height.quality.value())
                    )
                } else it.heightAboveSeaLevelHistory
            )
        }
    }

    private fun applyRelativeToTakeoffPosition(
        position: XYZ?,
        callbackTimestampNs: Long = 0L,
        callbackReceived: Boolean = false
    ) {
        val valid = position != null && position.x.isFinite() && position.y.isFinite() && position.z.isFinite()
        updatePositionAudit {
            it.copy(
                relativeToTakeoffPositionStatus = valueStatus(position != null, valid),
                relativeToTakeoffPositionRaw = position?.toString() ?: "null",
                relativeToTakeoffHistory = if (callbackReceived && valid && callbackTimestampNs > 0L) {
                    appendRelativePositionHistory(
                        it.relativeToTakeoffHistory,
                        RelativePositionSample(callbackTimestampNs, position!!.x, position.y, position.z)
                    )
                } else it.relativeToTakeoffHistory
            )
        }
    }

    private fun appendPositionHistory(
        history: List<AircraftPositionSample>,
        sample: AircraftPositionSample
    ): List<AircraftPositionSample> {
        if (history.isNotEmpty() && sample.timestampNs <= history.last().timestampNs) {
            // KeyManager can replay an already-observed value. The bridge
            // history remains an idempotent callback-time sequence.
            return history
        }
        return (history + sample).takeLast(MAXIMUM_POSITION_HISTORY_SAMPLES)
    }

    private fun appendScalarHistory(
        history: List<ScalarAltitudeSample>,
        timestampNs: Long,
        value: Double?,
        callbackReceived: Boolean
    ): List<ScalarAltitudeSample> {
        if (!callbackReceived || timestampNs <= 0L || value == null || !value.isFinite()) return history
        if (history.isNotEmpty() && timestampNs <= history.last().timestampNs) return history
        return (history + ScalarAltitudeSample(timestampNs, value)).takeLast(MAXIMUM_POSITION_HISTORY_SAMPLES)
    }

    private fun appendHeightAboveSeaLevelHistory(
        history: List<HeightAboveSeaLevelSample>,
        sample: HeightAboveSeaLevelSample
    ): List<HeightAboveSeaLevelSample> {
        if (history.isNotEmpty() && sample.timestampNs <= history.last().timestampNs) return history
        return (history + sample).takeLast(MAXIMUM_POSITION_HISTORY_SAMPLES)
    }

    private fun appendHeightAboveSeaLevelCallbackHistory(
        history: List<HeightAboveSeaLevelCallbackSample>,
        sample: HeightAboveSeaLevelCallbackSample
    ): List<HeightAboveSeaLevelCallbackSample> {
        if (history.isNotEmpty() && sample.timestampNs <= history.last().timestampNs) return history
        return (history + sample).takeLast(MAXIMUM_POSITION_HISTORY_SAMPLES)
    }

    private fun appendRelativePositionHistory(
        history: List<RelativePositionSample>,
        sample: RelativePositionSample
    ): List<RelativePositionSample> {
        if (history.isNotEmpty() && sample.timestampNs <= history.last().timestampNs) return history
        return (history + sample).takeLast(MAXIMUM_POSITION_HISTORY_SAMPLES)
    }

    private fun appendHorizontalQualityHistory(
        history: List<HorizontalQualitySample>,
        sample: HorizontalQualitySample
    ): List<HorizontalQualitySample> {
        if (sample.timestampNs <= 0L) return history
        if (history.isNotEmpty() && sample.timestampNs <= history.last().timestampNs) return history
        return (history + sample).takeLast(MAXIMUM_POSITION_HISTORY_SAMPLES)
    }

    private fun logPositionTimestampIfDue(sample: AircraftPositionSample) {
        val nowElapsedMs = android.os.SystemClock.elapsedRealtime()
        if (nowElapsedMs < nextPositionTimestampLogElapsedMs) return

        nextPositionTimestampLogElapsedMs = nowElapsedMs + 1000L
        Log.i(
            TAG,
            "DJI_POSITION_TIMESTAMP timestampNs=${sample.timestampNs} " +
                "lat=${sample.latitude} lon=${sample.longitude} altitude=${sample.altitude} " +
                "historyCount=${state.positionAudit.aircraftPositionHistory.size} " +
                "clockDomain=ANDROID_MONOTONIC ALTITUDE_REFERENCE_VALIDATION=PENDING"
        )
    }

    private fun recordPositionKeySupport(keyManager: KeyManager) {
        updatePositionAudit {
            it.copy(
                aircraftLocation2DSupport = keyManager.isKeySupported(aircraftLocationKey),
                aircraftLocation3DSupport = keyManager.isKeySupported(aircraftLocation3DKey),
                keyAltitudeSupport = keyManager.isKeySupported(aircraftAltitudeKey),
                aircraftVelocitySupport = keyManager.isKeySupported(aircraftVelocityKey),
                gpsSatelliteCountSupport = keyManager.isKeySupported(gpsSatelliteCountKey),
                gpsSignalLevelSupport = keyManager.isKeySupported(gpsSignalLevelKey),
                gpsIsValidSupport = keyManager.isKeySupported(gpsIsValidKey),
                gpsInterferenceStateSupport = keyManager.isKeySupported(gpsInterferenceStateKey),
                gpsInterferenceStateStatus = unsupportedStatus(
                    keyManager.isKeySupported(gpsInterferenceStateKey),
                    it.gpsInterferenceStateStatus
                ),
                fusionPositioningLevelSupport = keyManager.isKeySupported(fusionPositioningLevelKey),
                fusionPositioningLevelStatus = unsupportedStatus(
                    keyManager.isKeySupported(fusionPositioningLevelKey),
                    it.fusionPositioningLevelStatus
                ),
                homeLocationSupport = keyManager.isKeySupported(homeLocationKey),
                homeLocationSetSupport = keyManager.isKeySupported(homeLocationSetKey),
                takeoffLocationAltitudeSupport = keyManager.isKeySupported(takeoffLocationAltitudeKey),
                heightAboveSeaLevelSupport = keyManager.isKeySupported(heightAboveSeaLevelKey),
                relativeToTakeoffPositionSupport = keyManager.isKeySupported(relateToTakeOffPositionKey)
            )
        }
    }

    private fun updatePositionAudit(transform: (PositionAuditState) -> PositionAuditState) {
        state = state.copy(
            timestampMs = System.currentTimeMillis(),
            positionAudit = transform(state.positionAudit)
        ).withPoseAvailability()
        logPositionAuditIfDue()
    }

    private fun logPositionAuditIfDue() {
        val nowElapsedMs = android.os.SystemClock.elapsedRealtime()
        if (nowElapsedMs < nextPositionAuditLogElapsedMs) return

        nextPositionAuditLogElapsedMs = nowElapsedMs + 1000L
        val audit = state.positionAudit
        val aircraft = state.aircraft
        Log.i(
            TAG,
            "DJI_POSITION_AUDIT " +
                "aircraftLocation3D=${audit.aircraftLocation3DRaw} " +
                "latitude=${if (aircraft.hasLocation) aircraft.latitude else "UNAVAILABLE"} " +
                "longitude=${if (aircraft.hasLocation) aircraft.longitude else "UNAVAILABLE"} " +
                "heightAboveSeaLevel=${audit.heightAboveSeaLevelRaw} " +
                "takeoffRelativePosition=${audit.relativeToTakeoffPositionRaw} " +
                "takeoffLocationAltitude=${audit.takeoffLocationAltitudeRaw} " +
                "aircraftVelocity=${audit.aircraftVelocityRaw} " +
                "gpsIsValid=${audit.gpsIsValidRaw} gpsSignalLevel=${audit.gpsSignalLevelRaw} " +
                "gpsInterferenceState=${audit.gpsInterferenceStateRaw} " +
                "fusionPositioningLevel=${audit.fusionPositioningLevelRaw} " +
                "gpsSatelliteCount=${audit.gpsSatelliteCountRaw} homeLocation=${audit.homeLocationRaw} " +
                "horizontalValid=${aircraft.hasLocation} " +
                "verticalValid=false hasTelemetryPosition=false " +
                "takeoffLocationAltitudeSemantics=UNVALIDATED_FOR_AR_POSITION"
        )
        listOf(
            "KeyAircraftLocation" to audit.aircraftLocation2DStatus,
            "KeyAircraftLocation3D" to audit.aircraftLocation3DStatus,
            "KeyAltitude" to audit.keyAltitudeStatus,
            "KeyAircraftVelocity" to audit.aircraftVelocityStatus,
            "KeyGPSSatelliteCount" to audit.gpsSatelliteCountStatus,
            "KeyGPSSignalLevel" to audit.gpsSignalLevelStatus,
            "KeyGPSIsValid" to audit.gpsIsValidStatus,
            "KeyGPSInterferenceState" to audit.gpsInterferenceStateStatus,
            "KeyFusionPositioningLevel" to audit.fusionPositioningLevelStatus,
            "KeyHomeLocation" to audit.homeLocationStatus,
            "KeyIsHomeLocationSet" to audit.homeLocationSetStatus,
            "KeyTakeoffLocationAltitude" to audit.takeoffLocationAltitudeStatus,
            "KeyHeightAboveSeaLevel" to audit.heightAboveSeaLevelStatus,
            "KeyRelateToTakeOffPosition" to audit.relativeToTakeoffPositionStatus
        ).forEach { (key, status) ->
            val support = if (audit.supportFor(key)) "KEY_SUPPORTED" else "KEY_UNSUPPORTED"
            Log.i(TAG, "DJI_POSITION_KEY_AUDIT key=$key support=$support valueStatus=$status")
        }
        Log.i(
            TAG,
            "DJI_POSITION_CAPABILITY_AUDIT " +
                "POSITION_KEYS_SUPPORTED=${audit.allAuditedKeysSupported()} " +
                "POSITION_DATA_CURRENTLY_VALID=${audit.hasCurrentlyValidPositionData()} " +
                "TELEMETRY_TRANSLATION_CAPABILITY=NONE " +
                "FUTURE_LOCAL_NED_POSSIBLE=UNVALIDATED"
        )
    }

    private fun valueStatus(hasValue: Boolean, valid: Boolean): String = when {
        !hasValue -> "KEY_NO_VALUE"
        valid -> "KEY_AVAILABLE_VALID"
        else -> "KEY_AVAILABLE_BUT_INVALID"
    }

    private fun unsupportedStatus(supported: Boolean, currentStatus: String): String =
        if (!supported && currentStatus != "CALLBACK_ERROR") "KEY_UNSUPPORTED" else currentStatus

    private fun isValidCoordinate(latitude: Double?, longitude: Double?): Boolean =
        latitude != null && longitude != null &&
            latitude.isFinite() && longitude.isFinite() &&
            latitude in -90.0..90.0 && longitude in -180.0..180.0 &&
            !(latitude == 0.0 && longitude == 0.0) &&
            !isKnownDjiCoordinateSentinel(latitude, longitude)

    // The SDK has reported this unset pair on-device; range checks alone are not enough.
    private fun isKnownDjiCoordinateSentinel(latitude: Double, longitude: Double): Boolean =
        latitude == 45_836_623.61046586 && longitude == 45_836_623.61046586

    private fun applyAircraftAttitude(attitude: Attitude?) {
        if (attitude == null) return

        // This is the KeyManager callback-arrival time, not a DJI exposure time.
        // It must survive polling unchanged so Unity can bracket a vision frame.
        val arrivalTimestampNs = android.os.SystemClock.elapsedRealtimeNanos()
        val wallClockTimestampMs = System.currentTimeMillis()
        state = state.copy(
            timestampMs = wallClockTimestampMs,
            telemetryArrivalTimestampNs = arrivalTimestampNs,
            aircraftAttitudeArrivalTimestampNs = arrivalTimestampNs,
            telemetryWallClockTimestampMs = wallClockTimestampMs,
            telemetryTimestampSource = "SystemClock.elapsedRealtimeNanos_on_attitude_key_update",
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

        // Keep the timestamp of the actual telemetry update rather than of a
        // later Unity poll. Either orientation callback is sufficient.
        val arrivalTimestampNs = android.os.SystemClock.elapsedRealtimeNanos()
        val wallClockTimestampMs = System.currentTimeMillis()
        state = state.copy(
            timestampMs = wallClockTimestampMs,
            telemetryArrivalTimestampNs = arrivalTimestampNs,
            gimbalAttitudeArrivalTimestampNs = arrivalTimestampNs,
            telemetryWallClockTimestampMs = wallClockTimestampMs,
            telemetryTimestampSource = "SystemClock.elapsedRealtimeNanos_on_attitude_key_update",
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
            .put("telemetryArrivalTimestampNs", snapshot.telemetryArrivalTimestampNs)
            .put("aircraftAttitudeArrivalTimestampNs", snapshot.aircraftAttitudeArrivalTimestampNs)
            .put("gimbalAttitudeArrivalTimestampNs", snapshot.gimbalAttitudeArrivalTimestampNs)
            .put("telemetryWallClockTimestampMs", snapshot.telemetryWallClockTimestampMs)
            .put("telemetryTimestampSource", snapshot.telemetryTimestampSource)
            .put("aircraft", aircraft)
            .put("gimbal", gimbal)
            .put(
                "aircraftBattery",
                JSONObject()
                    .put("hasPercent", snapshot.aircraftBattery.hasPercent)
                    .put("percent", snapshot.aircraftBattery.percent)
            )
            .put("positionAudit", buildPositionAuditJson(snapshot.positionAudit))
    }

    private fun buildPositionAuditJson(audit: PositionAuditState): JSONObject {
        logSatelliteSerializationIfChanged(audit)
        val velocityHistory = JSONArray()
        audit.velocityHistory.forEach { sample ->
            velocityHistory.put(
                JSONObject()
                    .put("timestampNs", sample.timestampNs)
                    .put("northMetersPerSecond", sample.northMetersPerSecond)
                    .put("eastMetersPerSecond", sample.eastMetersPerSecond)
                    .put("downMetersPerSecond", sample.downMetersPerSecond)
            )
        }
        val aircraftPositionHistory = JSONArray()
        audit.aircraftPositionHistory.forEach { sample ->
            aircraftPositionHistory.put(
                JSONObject()
                    .put("timestampNs", sample.timestampNs)
                    .put("latitude", sample.latitude)
                    .put("longitude", sample.longitude)
                    .put("altitude", sample.altitude)
            )
        }
        val altitudeHistory = JSONArray()
        audit.altitudeHistory.forEach { sample ->
            altitudeHistory.put(JSONObject().put("timestampNs", sample.timestampNs).put("value", sample.value))
        }
        val heightAboveSeaLevelHistory = JSONArray()
        audit.heightAboveSeaLevelHistory.forEach { sample ->
            heightAboveSeaLevelHistory.put(
                JSONObject().put("timestampNs", sample.timestampNs).put("height", sample.height).put("quality", sample.quality)
            )
        }
        val heightAboveSeaLevelCallbackHistory = JSONArray()
        audit.heightAboveSeaLevelCallbackHistory.forEach { sample ->
            heightAboveSeaLevelCallbackHistory.put(
                JSONObject()
                    .put("timestampNs", sample.timestampNs)
                    .put("hasHeight", sample.hasHeight)
                    .put("height", sample.height)
                    .put("hasQuality", sample.hasQuality)
                    .put("quality", sample.quality)
                    .put("status", sample.status)
            )
        }
        val relativeToTakeoffHistory = JSONArray()
        audit.relativeToTakeoffHistory.forEach { sample ->
            relativeToTakeoffHistory.put(
                JSONObject().put("timestampNs", sample.timestampNs).put("x", sample.x).put("y", sample.y).put("z", sample.z)
            )
        }
        fun horizontalQualityHistory(history: List<HorizontalQualitySample>): JSONArray = JSONArray().also { json ->
            history.forEach { sample ->
                json.put(
                    JSONObject()
                        .put("timestampNs", sample.timestampNs)
                        .put("rawValue", sample.rawValue)
                        .put("status", sample.status)
                        .put("error", sample.error ?: JSONObject.NULL)
                )
            }
        }
        val gpsInterferenceStateHistory = horizontalQualityHistory(audit.gpsInterferenceStateHistory)
        val fusionPositioningLevelHistory = horizontalQualityHistory(audit.fusionPositioningLevelHistory)
        return JSONObject()
        .put("aircraftLocation2DSupport", audit.aircraftLocation2DSupport)
        .put("aircraftLocation2DStatus", audit.aircraftLocation2DStatus)
        .put("aircraftLocation2DRaw", audit.aircraftLocation2DRaw)
        .put("aircraftLocation3DSupport", audit.aircraftLocation3DSupport)
        .put("aircraftLocation3DStatus", audit.aircraftLocation3DStatus)
        .put("aircraftLocation3DRaw", audit.aircraftLocation3DRaw)
        .put("aircraftPositionHistoryCount", audit.aircraftPositionHistory.size)
        .put("newestAircraftPositionTimestampNs", audit.aircraftPositionHistory.lastOrNull()?.timestampNs ?: 0L)
        .put("aircraftPositionHistory", aircraftPositionHistory)
        .put("altitudeReferenceValidation", "PENDING")
        .put("keyAltitudeSupport", audit.keyAltitudeSupport)
        .put("keyAltitudeStatus", audit.keyAltitudeStatus)
        .put("keyAltitudeRaw", audit.keyAltitudeRaw)
        .put("altitudeHistory", altitudeHistory)
        .put("aircraftVelocitySupport", audit.aircraftVelocitySupport)
        .put("aircraftVelocityStatus", audit.aircraftVelocityStatus)
        .put("aircraftVelocityRaw", audit.aircraftVelocityRaw)
        .put("hasAircraftVelocity", audit.hasAircraftVelocity)
        .put("aircraftVelocityX", audit.aircraftVelocityX)
        .put("aircraftVelocityY", audit.aircraftVelocityY)
        .put("aircraftVelocityZ", audit.aircraftVelocityZ)
        .put("latestVelocityTimestampNs", audit.latestVelocityTimestampNs)
        .put("velocityHistoryCount", audit.velocityHistory.size)
        .put("newestVelocityHistoryTimestampNs", audit.velocityHistory.lastOrNull()?.timestampNs ?: 0L)
        .put("velocityHistory", velocityHistory)
        .put("velocityUnitValidation", "PENDING")
        .put("gpsSatelliteCountSupport", audit.gpsSatelliteCountSupport)
        .put("gpsSatelliteCountStatus", audit.gpsSatelliteCountStatus)
        .put("gpsSatelliteCountRaw", audit.gpsSatelliteCountRaw)
        .put("hasGpsSatelliteCount", audit.hasGpsSatelliteCount)
        .put("gpsSignalLevelSupport", audit.gpsSignalLevelSupport)
        .put("gpsSignalLevelStatus", audit.gpsSignalLevelStatus)
        .put("gpsSignalLevelRaw", audit.gpsSignalLevelRaw)
        .put("gpsIsValidSupport", audit.gpsIsValidSupport)
        .put("gpsIsValidStatus", audit.gpsIsValidStatus)
        .put("gpsIsValidRaw", audit.gpsIsValidRaw)
        .put("gpsInterferenceStateSupport", audit.gpsInterferenceStateSupport)
        .put("gpsInterferenceStateStatus", audit.gpsInterferenceStateStatus)
        .put("gpsInterferenceStateRaw", audit.gpsInterferenceStateRaw)
        .put("gpsInterferenceStateError", audit.gpsInterferenceStateError ?: JSONObject.NULL)
        .put("gpsInterferenceStateHistoryCount", audit.gpsInterferenceStateHistory.size)
        .put("newestGpsInterferenceStateTimestampNs", audit.gpsInterferenceStateHistory.lastOrNull()?.timestampNs ?: 0L)
        .put("gpsInterferenceStateHistory", gpsInterferenceStateHistory)
        .put("fusionPositioningLevelSupport", audit.fusionPositioningLevelSupport)
        .put("fusionPositioningLevelStatus", audit.fusionPositioningLevelStatus)
        .put("fusionPositioningLevelRaw", audit.fusionPositioningLevelRaw)
        .put("fusionPositioningLevelError", audit.fusionPositioningLevelError ?: JSONObject.NULL)
        .put("fusionPositioningLevelHistoryCount", audit.fusionPositioningLevelHistory.size)
        .put("newestFusionPositioningLevelTimestampNs", audit.fusionPositioningLevelHistory.lastOrNull()?.timestampNs ?: 0L)
        .put("fusionPositioningLevelHistory", fusionPositioningLevelHistory)
        .put("homeLocationSupport", audit.homeLocationSupport)
        .put("homeLocationStatus", audit.homeLocationStatus)
        .put("homeLocationRaw", audit.homeLocationRaw)
        .put("homeLocationSetSupport", audit.homeLocationSetSupport)
        .put("homeLocationSetStatus", audit.homeLocationSetStatus)
        .put("homeLocationSetRaw", audit.homeLocationSetRaw)
        .put("takeoffLocationAltitudeSupport", audit.takeoffLocationAltitudeSupport)
        .put("takeoffLocationAltitudeStatus", audit.takeoffLocationAltitudeStatus)
        .put("takeoffLocationAltitudeRaw", audit.takeoffLocationAltitudeRaw)
        .put("heightAboveSeaLevelSupport", audit.heightAboveSeaLevelSupport)
        .put("heightAboveSeaLevelStatus", audit.heightAboveSeaLevelStatus)
        .put("heightAboveSeaLevelRaw", audit.heightAboveSeaLevelRaw)
        .put("heightAboveSeaLevelHistory", heightAboveSeaLevelHistory)
        .put("heightAboveSeaLevelCallbackHistory", heightAboveSeaLevelCallbackHistory)
        .put("relativeToTakeoffPositionSupport", audit.relativeToTakeoffPositionSupport)
        .put("relativeToTakeoffPositionStatus", audit.relativeToTakeoffPositionStatus)
        .put("relativeToTakeoffPositionRaw", audit.relativeToTakeoffPositionRaw)
        .put("relativeToTakeoffHistory", relativeToTakeoffHistory)
    }

    private fun logSatelliteSerializationIfChanged(audit: PositionAuditState) {
        if (lastLoggedSatelliteSerializedPresent == audit.hasGpsSatelliteCount &&
            lastLoggedSatelliteSerializedRaw == audit.gpsSatelliteCountRaw) return

        lastLoggedSatelliteSerializedPresent = audit.hasGpsSatelliteCount
        lastLoggedSatelliteSerializedRaw = audit.gpsSatelliteCountRaw
        Log.i(
            TAG,
            "DJI_SAT_SERIALIZED fieldPresent=${audit.hasGpsSatelliteCount} value=${audit.gpsSatelliteCountRaw} " +
                "gpsIsValid=${audit.gpsIsValidRaw} gpsSignalLevel=${audit.gpsSignalLevelRaw}"
        )
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
        // Monotonic callback-arrival time for the latest valid attitude update.
        // This belongs to the snapshot and must never be refreshed by polling.
        val telemetryArrivalTimestampNs: Long = 0L,
        val aircraftAttitudeArrivalTimestampNs: Long = 0L,
        val gimbalAttitudeArrivalTimestampNs: Long = 0L,
        val telemetryWallClockTimestampMs: Long = 0L,
        val telemetryTimestampSource: String = "UNAVAILABLE",
        val aircraft: AircraftPose = AircraftPose(),
        val gimbal: GimbalPose = GimbalPose(),
        val aircraftBattery: AircraftBatteryState = AircraftBatteryState(),
        val positionAudit: PositionAuditState = PositionAuditState()
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

    private data class AircraftBatteryState(
        val percent: Int = 0,
        val hasPercent: Boolean = false
    )

    private data class VelocitySample(
        val timestampNs: Long,
        val northMetersPerSecond: Double,
        val eastMetersPerSecond: Double,
        val downMetersPerSecond: Double
    )

    // Every sample owns the callback-arrival timestamp from its own key. None
    // of these vertical values is used to construct a telemetry camera pose.
    private data class AircraftPositionSample(
        val timestampNs: Long,
        val latitude: Double,
        val longitude: Double,
        val altitude: Double
    )

    private data class ScalarAltitudeSample(val timestampNs: Long, val value: Double)

    private data class HeightAboveSeaLevelSample(
        val timestampNs: Long,
        val height: Double,
        val quality: Int
    )

    // This audit list preserves rejected callback metadata without changing
    // the accepted HASEL history's quality contract.
    private data class HeightAboveSeaLevelCallbackSample(
        val timestampNs: Long,
        val hasHeight: Boolean,
        val height: Double?,
        val hasQuality: Boolean,
        val quality: Int?,
        val status: String
    )

    private data class RelativePositionSample(
        val timestampNs: Long,
        val x: Double,
        val y: Double,
        val z: Double
    )

    private data class HorizontalQualitySample(
        val timestampNs: Long,
        val rawValue: String,
        val status: String,
        val error: String? = null
    )

    private data class PositionAuditState(
        val aircraftLocation2DSupport: Boolean = false,
        val aircraftLocation2DStatus: String = "KEY_NOT_SUBSCRIBED",
        val aircraftLocation2DRaw: String = "null",
        val aircraftLocation3DSupport: Boolean = false,
        val aircraftLocation3DStatus: String = "KEY_NOT_SUBSCRIBED",
        val aircraftLocation3DRaw: String = "null",
        val aircraftPositionHistory: List<AircraftPositionSample> = emptyList(),
        val keyAltitudeSupport: Boolean = false,
        val keyAltitudeStatus: String = "KEY_NOT_SUBSCRIBED",
        val keyAltitudeRaw: String = "null",
        val altitudeHistory: List<ScalarAltitudeSample> = emptyList(),
        val aircraftVelocitySupport: Boolean = false,
        val aircraftVelocityStatus: String = "KEY_NOT_SUBSCRIBED",
        val aircraftVelocityRaw: String = "null",
        val hasAircraftVelocity: Boolean = false,
        val aircraftVelocityX: Float = 0f,
        val aircraftVelocityY: Float = 0f,
        val aircraftVelocityZ: Float = 0f,
        // Velocity callback time and samples stay separate from attitude data.
        val latestVelocityTimestampNs: Long = 0L,
        val velocityHistory: List<VelocitySample> = emptyList(),
        val gpsSatelliteCountSupport: Boolean = false,
        val gpsSatelliteCountStatus: String = "KEY_NOT_SUBSCRIBED",
        val gpsSatelliteCountRaw: String = "null",
        val hasGpsSatelliteCount: Boolean = false,
        val gpsSignalLevelSupport: Boolean = false,
        val gpsSignalLevelStatus: String = "KEY_NOT_SUBSCRIBED",
        val gpsSignalLevelRaw: String = "null",
        val gpsIsValidSupport: Boolean = false,
        val gpsIsValidStatus: String = "KEY_NOT_SUBSCRIBED",
        val gpsIsValidRaw: String = "null",
        val gpsInterferenceStateSupport: Boolean = false,
        val gpsInterferenceStateStatus: String = "KEY_NOT_SUBSCRIBED",
        val gpsInterferenceStateRaw: String = "null",
        val gpsInterferenceStateError: String? = null,
        val gpsInterferenceStateHistory: List<HorizontalQualitySample> = emptyList(),
        val fusionPositioningLevelSupport: Boolean = false,
        val fusionPositioningLevelStatus: String = "KEY_NOT_SUBSCRIBED",
        val fusionPositioningLevelRaw: String = "null",
        val fusionPositioningLevelError: String? = null,
        val fusionPositioningLevelHistory: List<HorizontalQualitySample> = emptyList(),
        val homeLocationSupport: Boolean = false,
        val homeLocationStatus: String = "KEY_NOT_SUBSCRIBED",
        val homeLocationRaw: String = "null",
        val homeLocationSetSupport: Boolean = false,
        val homeLocationSetStatus: String = "KEY_NOT_SUBSCRIBED",
        val homeLocationSetRaw: String = "null",
        val takeoffLocationAltitudeSupport: Boolean = false,
        val takeoffLocationAltitudeStatus: String = "KEY_NOT_SUBSCRIBED",
        val takeoffLocationAltitudeRaw: String = "null",
        val heightAboveSeaLevelSupport: Boolean = false,
        val heightAboveSeaLevelStatus: String = "KEY_NOT_SUBSCRIBED",
        val heightAboveSeaLevelRaw: String = "null",
        val heightAboveSeaLevelHistory: List<HeightAboveSeaLevelSample> = emptyList(),
        val heightAboveSeaLevelCallbackHistory: List<HeightAboveSeaLevelCallbackSample> = emptyList(),
        val relativeToTakeoffPositionSupport: Boolean = false,
        val relativeToTakeoffPositionStatus: String = "KEY_NOT_SUBSCRIBED",
        val relativeToTakeoffPositionRaw: String = "null",
        val relativeToTakeoffHistory: List<RelativePositionSample> = emptyList()
    ) {
        fun allAuditedKeysSupported(): Boolean =
            aircraftLocation2DSupport && aircraftLocation3DSupport && keyAltitudeSupport &&
                aircraftVelocitySupport && gpsSatelliteCountSupport && gpsSignalLevelSupport &&
                gpsIsValidSupport && homeLocationSupport && homeLocationSetSupport &&
                takeoffLocationAltitudeSupport && heightAboveSeaLevelSupport && relativeToTakeoffPositionSupport

        fun hasCurrentlyValidPositionData(): Boolean =
            aircraftLocation3DStatus == "KEY_AVAILABLE_VALID" &&
                gpsIsValidStatus == "KEY_AVAILABLE_VALID" &&
                gpsSignalLevelStatus == "KEY_AVAILABLE_VALID" &&
                gpsSatelliteCountStatus == "KEY_AVAILABLE_VALID" &&
                heightAboveSeaLevelStatus == "KEY_AVAILABLE_VALID"

        fun supportFor(key: String): Boolean = when (key) {
            "KeyAircraftLocation" -> aircraftLocation2DSupport
            "KeyAircraftLocation3D" -> aircraftLocation3DSupport
            "KeyAltitude" -> keyAltitudeSupport
            "KeyAircraftVelocity" -> aircraftVelocitySupport
            "KeyGPSSatelliteCount" -> gpsSatelliteCountSupport
            "KeyGPSSignalLevel" -> gpsSignalLevelSupport
            "KeyGPSIsValid" -> gpsIsValidSupport
            "KeyGPSInterferenceState" -> gpsInterferenceStateSupport
            "KeyFusionPositioningLevel" -> fusionPositioningLevelSupport
            "KeyHomeLocation" -> homeLocationSupport
            "KeyIsHomeLocationSet" -> homeLocationSetSupport
            "KeyTakeoffLocationAltitude" -> takeoffLocationAltitudeSupport
            "KeyHeightAboveSeaLevel" -> heightAboveSeaLevelSupport
            "KeyRelateToTakeOffPosition" -> relativeToTakeoffPositionSupport
            else -> false
        }
    }
}
