package com.sok9hu.djibridge

import android.app.Application
import android.content.Context
import android.hardware.usb.UsbManager
import android.os.Handler
import android.os.Looper
import android.util.Log
import com.cySdkyc.clx.Helper
import com.sok9hu.djibridge.initMobileSDK.MSDKManagerVM
import com.sok9hu.djibridge.initMobileSDK.globalViewModels

/**
 * Plugin initializer that starts the DJI Mobile SDK once in the application.
 */
object DJIPlugin {
    private const val TAG = "DJIPlugin"
    private val mainHandler = Handler(Looper.getMainLooper())

    private var initialized = false
    private var observersAttached = false
    private lateinit var msdkManagerVM: MSDKManagerVM

    @Volatile private var sdkRegistered = false
    @Volatile private var productConnected = false

    @JvmStatic
    fun init(application: Application) {
        if (Looper.myLooper() != Looper.getMainLooper()) {
            Log.i(TAG, "init() called off main thread; reposting to Android main thread")
            mainHandler.post { init(application) }
            return
        }

        Log.i(TAG, "init() called")
        logUsbState("beforeInit", application)

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

    internal fun describeState(): String =
        "initialized=$initialized registered=$sdkRegistered productConnected=$productConnected"

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
            notifyVideoBridgeIfReady("registerState")
        }

        msdkManagerVM.lvProductConnectionState.observeForever { (isConnected, productId) ->
            productConnected = isConnected
            Log.i(TAG, "DJI product connection changed: connected=$isConnected productId=$productId")
            DJIPoseBridge.onSdkStateChanged("productConnection")
            notifyVideoBridgeIfReady("productConnection")
        }
    }

    private fun notifyVideoBridgeIfReady(source: String) {
        if (!isVideoBridgeReady()) return

        Log.i(TAG, "Video bridge became ready via $source (${describeState()})")
        DJIUnityVideoBridge.onSdkReadyChanged(source)
    }

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
