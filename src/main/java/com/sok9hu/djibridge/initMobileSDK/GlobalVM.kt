package com.sok9hu.djibridge.initMobileSDK

import android.app.Application
import androidx.annotation.MainThread
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelLazy
import androidx.lifecycle.ViewModelProvider

import androidx.lifecycle.ViewModelStore

val globalViewModelStore = ViewModelStore()
@MainThread
inline fun <reified VM : ViewModel> Application.globalViewModels(): Lazy<VM> {
    val factory = ViewModelProvider.AndroidViewModelFactory.getInstance(this)
    return ViewModelLazy(
        VM::class,
        { globalViewModelStore },
        { factory })
}