package com.sok9hu.djibridge

import android.app.Application
import com.cySdkyc.clx.Helper
import com.sok9hu.djibridge.initMobileSDK.MSDKManagerVM
import com.sok9hu.djibridge.initMobileSDK.globalViewModels

/**
 * Plugin initializer that starts the DJI Mobile SDK once in the application.
 */

object DJIPlugin {
    private var initialized = false
    private lateinit var app: Application
    private lateinit var msdkManagerVM: MSDKManagerVM

    @JvmStatic
    fun init(application: Application) {
        if (initialized) return
        initialized = true
        this.app = application

        // Required by DJI to load protected libs before touching SDK classes
        Helper.install(application)

        // Init -> wait for INITIALIZE_COMPLETE -> registerApp
        msdkManagerVM = application.globalViewModels<MSDKManagerVM>().value
        msdkManagerVM.initMobileSDK(application)
    }

}
