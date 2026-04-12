package com.sok9hu.djibridge

import android.app.Application
import android.content.ContentProvider
import android.content.ContentValues
import android.database.Cursor
import android.net.Uri
import android.util.Log

/**
 * Starts the DJI plugin as early as possible in the app process, before Unity scene bootstrap.
 */
class DJIStartupProvider : ContentProvider() {
    override fun onCreate(): Boolean {
        val application = context?.applicationContext as? Application
        if (application == null) {
            Log.w(TAG, "Startup provider could not access Application")
            return true
        }

        try {
            Log.i(TAG, "Early startup provider initializing DJI plugin")
            DJIPlugin.init(application)
        } catch (t: Throwable) {
            Log.e(TAG, "Early startup provider failed", t)
        }

        return true
    }

    override fun query(
        uri: Uri,
        projection: Array<out String>?,
        selection: String?,
        selectionArgs: Array<out String>?,
        sortOrder: String?
    ): Cursor? = null

    override fun getType(uri: Uri): String? = null

    override fun insert(uri: Uri, values: ContentValues?): Uri? = null

    override fun delete(uri: Uri, selection: String?, selectionArgs: Array<out String>?): Int = 0

    override fun update(
        uri: Uri,
        values: ContentValues?,
        selection: String?,
        selectionArgs: Array<out String>?
    ): Int = 0

    private companion object {
        const val TAG = "DJIStartupProvider"
    }
}
