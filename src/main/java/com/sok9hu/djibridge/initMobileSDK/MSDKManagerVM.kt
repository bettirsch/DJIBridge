package com.sok9hu.djibridge.initMobileSDK

import android.content.Context
import android.util.Log
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import dji.v5.network.DJINetworkManager

/**
 * ViewModel for handling DJI Mobile SDK initialization, registration, and connection states.
 */
class MSDKManagerVM : ViewModel() {

    private val TAG = "MSDKManagerVM"
    // The data is held in livedata mode, but you can also save the results of the sdk callbacks any way you like.
    val lvRegisterState = MutableLiveData<Pair<Boolean, IDJIError?>>()
    val lvProductConnectionState = MutableLiveData<Pair<Boolean, Int>>()
    val lvProductChanges = MutableLiveData<Int>()
    val lvInitProcess = MutableLiveData<Pair<DJISDKInitEvent, Int>>()
    val lvDBDownloadProgress = MutableLiveData<Pair<Long, Long>>()
    var isInit = false

    fun initMobileSDK(appContext: Context) {
        // Initialize and set the sdk callback, which is held internally by the sdk until destroy() is called
        SDKManager.getInstance().init(appContext, object : SDKManagerCallback {
            override fun onRegisterSuccess() {
                Log.i(TAG, "sok9hu - onRegisterSuccess")
                lvRegisterState.postValue(Pair(true, null))
            }

            override fun onRegisterFailure(error: IDJIError) {
                Log.i(TAG, "sok9hu - onRegisterFailure")
                lvRegisterState.postValue(Pair(false, error))
            }

            override fun onProductDisconnect(productId: Int) {
                Log.i(TAG, "sok9hu - onProductDisconnect")
                lvProductConnectionState.postValue(Pair(false, productId))
            }

            override fun onProductConnect(productId: Int) {
                Log.i(TAG, "sok9hu - onProductConnect")
                lvProductConnectionState.postValue(Pair(true, productId))
            }

            override fun onProductChanged(productId: Int) {
                Log.i(TAG, "sok9hu - onProductChanged")
                lvProductChanges.postValue(productId)
            }

            override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                Log.i(TAG, "sok9hu - onInitProcess")
                lvInitProcess.postValue(Pair(event, totalProcess))
                // Don't forget to call the registerApp()
                if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                    isInit = true
                    SDKManager.getInstance().registerApp()
                }
            }

            override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                Log.i(TAG, "sok9hu - onDatabaseDownloadProgress")
                lvDBDownloadProgress.postValue(Pair(current, total))
            }
        })

        DJINetworkManager.getInstance().addNetworkStatusListener { isAvailable ->
            if (isInit && isAvailable && !SDKManager.getInstance().isRegistered) {
                Log.i(TAG, "sok9hu - registerApp start")
                SDKManager.getInstance().registerApp()
                Log.i(TAG, "sok9hu - registerApp end")
            }
        }
    }

    fun destroyMobileSDK() {
        SDKManager.getInstance().destroy()
    }

}