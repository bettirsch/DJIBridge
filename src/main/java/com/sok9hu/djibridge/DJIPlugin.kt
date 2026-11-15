package com.sok9hu.djibridge

import android.app.Application
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbAccessory
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import com.cySdkyc.clx.Helper
import com.sok9hu.djibridge.initMobileSDK.MSDKManagerVM
import com.sok9hu.djibridge.initMobileSDK.globalViewModels
import org.json.JSONObject
import java.util.concurrent.ConcurrentHashMap

/**
 * Plugin initializer that starts the DJI Mobile SDK once in the application.
 */
object DJIPlugin {
    private const val TAG = "DJIPlugin"
    private const val USB_PERMISSION_ACTION = "com.sok9hu.djibridge.USB_PERMISSION"
    private val mainHandler = Handler(Looper.getMainLooper())

    private val expectedAccessoryModels = setOf(
        "T600",
        "HG210",
        "WM160",
        "com.dji.logiclink",
        "com.dji.link"
    )

    private var initialized = false
    private var observersAttached = false
    private var usbReceiverRegistered = false
    private lateinit var msdkManagerVM: MSDKManagerVM

    @Volatile private var sdkRegistered = false
    @Volatile private var productConnected = false
    @Volatile private var appContext: Context? = null
    @Volatile private var usbConnectionSnapshot = UsbConnectionSnapshot()

    // A denial must not trigger an endless permission-dialog loop. A new physical
    // attachment clears the remembered request key and permits one new request.
    private val permissionRequestsForCurrentAttachment = ConcurrentHashMap.newKeySet<String>()

    private data class UsbConnectionSnapshot(
        val expectedDeviceAttached: Boolean = false,
        val permissionGranted: Boolean = false
    )

    private val usbBroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            when (intent.action) {
                USB_PERMISSION_ACTION -> {
                    val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    Log.i(TAG, "DJI USB permission result granted=$granted")
                    refreshUsbConnectionSnapshot("permissionResult", requestPermission = false)
                }

                UsbManager.ACTION_USB_DEVICE_ATTACHED,
                UsbManager.ACTION_USB_ACCESSORY_ATTACHED ->
                    refreshUsbConnectionSnapshot("usbAttached", requestPermission = true)

                UsbManager.ACTION_USB_DEVICE_DETACHED,
                UsbManager.ACTION_USB_ACCESSORY_DETACHED ->
                    refreshUsbConnectionSnapshot("usbDetached", requestPermission = false)
            }
        }
    }

    @JvmStatic
    fun init(application: Application) {
        appContext = application.applicationContext
        if (Looper.myLooper() != Looper.getMainLooper()) {
            Log.i(TAG, "init() called off main thread; reposting to Android main thread")
            mainHandler.post { init(application) }
            return
        }

        Log.i(TAG, "init() called")
        registerUsbReceiverIfNeeded(application)
        // The Unity Home screen requests permission only after it is visible.
        // Do not attempt to launch a system dialog from the startup provider.
        refreshUsbConnectionSnapshot("beforeInit", requestPermission = false)

        if (initialized) {
            attachObserversIfNeeded()
            return
        }
        initialized = true

        // Required by DJI to load protected libs before touching SDK classes
        Helper.install(application)

        // Init -> wait for INITIALIZE_COMPLETE -> registerApp
        msdkManagerVM = application.globalViewModels<MSDKManagerVM>().value
        attachObserversIfNeeded()
        Log.i(TAG, "Calling initMobileSDK()")
        msdkManagerVM.initMobileSDK(application)
    }

    internal fun isVideoBridgeReady(): Boolean =
        initialized && sdkRegistered && productConnected

    internal fun isProductConnected(): Boolean = productConnected

    internal fun isSdkRegistered(): Boolean = sdkRegistered

    internal fun describeState(): String =
        "initialized=$initialized registered=$sdkRegistered productConnected=$productConnected"

    /**
     * Unity HomeScene polls this lightweight state; it does not create another
     * DJI SDK listener or require the DroneView video pipeline to start.
     */
    @JvmStatic
    fun getHomeConnectionStatusJson(): String {
        refreshUsbConnectionSnapshot("unityHomePoll", requestPermission = true)
        val usb = usbConnectionSnapshot
        return JSONObject()
            .put("expectedUsbDeviceAttached", usb.expectedDeviceAttached)
            .put("usbPermissionGranted", usb.permissionGranted)
            .put("sdkRegistered", sdkRegistered)
            .put("productConnected", productConnected)
            .toString()
    }

    /** Application context used only for diagnostic detector-frame capture. */
    internal fun applicationContextOrNull(): Context? = appContext

    private fun attachObserversIfNeeded() {
        if (observersAttached) return
        observersAttached = true

        msdkManagerVM.lvRegisterState.observeForever { (isRegistered, error) ->
            sdkRegistered = isRegistered
            if (isRegistered) {
                Log.i(TAG, "DJI SDK registration ready")
            } else {
                Log.w(TAG, "DJI SDK registration unavailable: ${error?.description() ?: "unknown"}")
            }
            DJIPoseBridge.onSdkStateChanged("registerState")
            DjiScreenshotRcBridge.onSdkStateChanged()
            notifyVideoBridgeIfReady("registerState")
        }

        msdkManagerVM.lvProductConnectionState.observeForever { (isConnected, productId) ->
            productConnected = isConnected
            Log.i(TAG, "DJI product connection changed: connected=$isConnected productId=$productId")
            refreshUsbConnectionSnapshot("productConnection", requestPermission = false)
            DJIPoseBridge.onSdkStateChanged("productConnection")
            DjiScreenshotRcBridge.onSdkStateChanged()
            DJIUnityVideoBridge.onProductConnectionChanged(isConnected, "productConnection")
            notifyVideoBridgeIfReady("productConnection")
        }
    }

    private fun notifyVideoBridgeIfReady(source: String) {
        if (!isVideoBridgeReady()) return

        Log.i(TAG, "Video bridge became ready via $source (${describeState()})")
        DJIUnityVideoBridge.onSdkReadyChanged(source)
    }

    private fun registerUsbReceiverIfNeeded(application: Application) {
        if (usbReceiverRegistered) return

        val filter = IntentFilter().apply {
            addAction(USB_PERMISSION_ACTION)
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
            addAction(UsbManager.ACTION_USB_ACCESSORY_ATTACHED)
            addAction(UsbManager.ACTION_USB_ACCESSORY_DETACHED)
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            application.registerReceiver(usbBroadcastReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
        } else {
            application.registerReceiver(usbBroadcastReceiver, filter)
        }
        usbReceiverRegistered = true
    }

    private fun refreshUsbConnectionSnapshot(source: String, requestPermission: Boolean) {
        if (requestPermission && Looper.myLooper() != Looper.getMainLooper()) {
            mainHandler.post { refreshUsbConnectionSnapshot(source, requestPermission = true) }
            return
        }

        val application = appContext ?: return
        val usbManager = application.getSystemService(Context.USB_SERVICE) as? UsbManager ?: return

        val accessories = usbManager.accessoryList.orEmpty().filter(::isExpectedDjiAccessory)
        val devices = usbManager.deviceList.values.filter(::isExpectedDjiDevice)
        val targetKeys = HashSet<String>()
        accessories.forEach { targetKeys.add(accessoryKey(it)) }
        devices.forEach { targetKeys.add(deviceKey(it)) }
        permissionRequestsForCurrentAttachment.retainAll(targetKeys)

        val permissionGranted = accessories.any(usbManager::hasPermission) || devices.any(usbManager::hasPermission)
        usbConnectionSnapshot = UsbConnectionSnapshot(
            expectedDeviceAttached = targetKeys.isNotEmpty(),
            permissionGranted = permissionGranted
        )

        if (requestPermission && !permissionGranted) {
            val accessory = accessories.firstOrNull { !usbManager.hasPermission(it) }
            val device = devices.firstOrNull { !usbManager.hasPermission(it) }
            when {
                accessory != null -> requestUsbPermission(usbManager, accessoryKey(accessory), accessory, source)
                device != null -> requestUsbPermission(usbManager, deviceKey(device), device, source)
            }
        }
    }

    private fun requestUsbPermission(
        usbManager: UsbManager,
        targetKey: String,
        accessory: UsbAccessory,
        source: String
    ) {
        if (!permissionRequestsForCurrentAttachment.add(targetKey)) return
        Log.i(TAG, "DJI USB permission requested target=$targetKey source=$source")
        usbManager.requestPermission(accessory, createUsbPermissionIntent())
    }

    private fun requestUsbPermission(
        usbManager: UsbManager,
        targetKey: String,
        device: UsbDevice,
        source: String
    ) {
        if (!permissionRequestsForCurrentAttachment.add(targetKey)) return
        Log.i(TAG, "DJI USB permission requested target=$targetKey source=$source")
        usbManager.requestPermission(device, createUsbPermissionIntent())
    }

    private fun createUsbPermissionIntent(): PendingIntent {
        val context = requireNotNull(appContext)
        val flags = PendingIntent.FLAG_UPDATE_CURRENT or
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) PendingIntent.FLAG_MUTABLE else 0
        return PendingIntent.getBroadcast(
            context,
            0,
            Intent(USB_PERMISSION_ACTION).setPackage(context.packageName),
            flags
        )
    }

    private fun isExpectedDjiAccessory(accessory: UsbAccessory): Boolean =
        accessory.manufacturer?.equals("DJI", ignoreCase = true) == true &&
            expectedAccessoryModels.contains(accessory.model)

    private fun isExpectedDjiDevice(device: UsbDevice): Boolean {
        val identity = listOfNotNull(device.manufacturerName, device.productName, device.deviceName)
            .joinToString(" ")
        return identity.contains("dji", ignoreCase = true)
    }

    private fun accessoryKey(accessory: UsbAccessory): String =
        "accessory:${accessory.manufacturer}:${accessory.model}:${accessory.version}"

    private fun deviceKey(device: UsbDevice): String =
        "device:${device.deviceName}:${device.vendorId}:${device.productId}"

    private fun logUsbState(source: String, application: Application) {
        try {
            val usbManager = application.getSystemService(Context.USB_SERVICE) as? UsbManager
            if (usbManager == null) {
                Log.w(TAG, "USB state [$source]: UsbManager unavailable")
                return
            }

            val accessories = usbManager.accessoryList.orEmpty()
            val devices = usbManager.deviceList.values.toList()

            Log.i(
                TAG,
                "USB state [$source]: accessories=${accessories.size} devices=${devices.size}"
            )

            accessories.forEachIndexed { index, accessory ->
                Log.i(
                    TAG,
                    "USB accessory[$index] manufacturer=${accessory.manufacturer} " +
                        "model=${accessory.model} version=${accessory.version} " +
                        "permission=${usbManager.hasPermission(accessory)}"
                )
            }

            devices.forEach { device ->
                Log.i(
                    TAG,
                    "USB device name=${device.deviceName} vendorId=${device.vendorId} " +
                        "productId=${device.productId} permission=${usbManager.hasPermission(device)}"
                )
            }
        } catch (t: Throwable) {
            Log.w(TAG, "USB state [$source] failed", t)
        }
    }
}
