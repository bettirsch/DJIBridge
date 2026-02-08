import org.gradle.api.tasks.bundling.Zip

plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.sok9hu.djibridge"
    compileSdk = 36

    defaultConfig {
        minSdk = 24
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        resourceConfigurations += listOf("en", "hu")
    }

    lint {
        abortOnError = false
        checkReleaseBuilds = false
        disable += setOf(
            "MissingTranslation",
            "ExtraTranslation",
            "StringFormatInvalid",
            "StringFormatMatches",
            "StringFormatInconsistent"
        )
    }

    androidResources {
        ignoreAssetsPattern = "!*"
        noCompress += listOf("tflite", "lite")
    }

    packaging {
        resources {
            excludes += listOf(
                "META-INF/*",
                "res/**/com.sok9hu.djibridge_*",
                "res/**/com.sok9hu.djibridge/**"
            )
            pickFirsts += listOf(
                "lib/arm64-v8a/libc++_shared.so",
                "lib/armeabi-v7a/libc++_shared.so"
            )
        }
        jniLibs {
            keepDebugSymbols += listOf(
                "*/*/libconstants.so",
                "*/*/libdji_innertools.so",
                "*/*/libdjibase.so",
                "*/*/libDJICSDKCommon.so",
                "*/*/libDJIFlySafeCore-CSDK.so",
                "*/*/libdjifs_jni-CSDK.so",
                "*/*/libDJIRegister.so",
                "*/*/libdjisdk_jni.so",
                "*/*/libDJIUpgradeCore.so",
                "*/*/libDJIUpgradeJNI.so",
                "*/*/libDJIWaypointV2Core-CSDK.so",
                "*/*/libdjiwpv2-CSDK.so",
                "*/*/libFlightRecordEngine.so",
                "*/*/libvideo-framing.so",
                "*/*/libwaes.so",
                "*/*/libagora-rtsa-sdk.so",
                "*/*/libc++.so",
                "*/*/libc++_shared.so",
                "*/*/libmrtc_28181.so",
                "*/*/libmrtc_agora.so",
                "*/*/libmrtc_core.so",
                "*/*/libmrtc_core_jni.so",
                "*/*/libmrtc_data.so",
                "*/*/libmrtc_log.so",
                "*/*/libmrtc_onvif.so",
                "*/*/libmrtc_rtmp.so",
                "*/*/libmrtc_rtsp.so",
                "*/*/libSdkyclx_clx.so"
            )
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    kotlinOptions {
        jvmTarget = "11"
    }
}

/**
 * Stable AAR rename (avoids internal AGP APIs)
 */
tasks.withType<Zip>().configureEach {
    when (name) {
        "bundleReleaseAar" -> archiveFileName.set("DJIUnityBridge.aar")
        "bundleDebugAar" -> archiveFileName.set("DJIUnityBridge-debug.aar")
    }
}

dependencies {
    compileOnly(files("libs/unity-classes.jar"))

    implementation(libs.dji.sdk.v5.aircraft)
    implementation(libs.dji.sdk.v5.networkimp)
    compileOnly(libs.dji.sdk.v5.aircraft.provided)

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)

    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
}