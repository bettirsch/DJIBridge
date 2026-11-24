import org.gradle.api.tasks.bundling.Zip
import groovy.json.JsonOutput
import groovy.json.JsonSlurper
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.security.MessageDigest
import java.util.zip.ZipFile

val fourTagMultiStartExperiment = providers.gradleProperty("DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT")
    .map { it.toBooleanStrict() }.getOrElse(true)
require(fourTagMultiStartExperiment) {
    "Android bridge builds require DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT=true. " +
        "OFF is supported only by the standalone DJI_BOARD_POSE_REPLAY_TEST harness."
}

plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.sok9hu.djibridge"
    compileSdk = 36
    ndkVersion = "27.0.12077973"

    defaultConfig {
        minSdk = 24
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        resourceConfigurations += listOf("en", "hu")
        // The consumer app resolves DJI_API_KEY when its manifest is merged.
        externalNativeBuild {
            cmake {
                cppFlags("-std=c++17")
                arguments("-DANDROID_STL=c++_shared")
                arguments("-DDJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT=${if (fourTagMultiStartExperiment) "ON" else "OFF"}")
            }
        }
    }

    buildFeatures {
        prefab = true
    }

    externalNativeBuild {
        cmake {
            path("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
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
                "*/*/libSdkyclx_clx.so",
                "*/*/libdjiunity.so"
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

object BridgeFeatureProvenance {
    const val manifestPath = "roofcanopy-bridge-features.json"
    const val getter = "Java_com_sok9hu_djibridge_DjiBoardVisionNative_isFourTagMultiStartExperimentEnabled"
    val requiredAbis = setOf("arm64-v8a", "armeabi-v7a", "x86", "x86_64")

    fun sha256(bytes: ByteArray): String =
        MessageDigest.getInstance("SHA-256").digest(bytes).joinToString("") { "%02x".format(it) }

    // Resolve the exported function through ELF .dynsym, never by scanning for a byte pattern.
    fun requireCompiledOn(bytes: ByteArray, abi: String) {
        val elf = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
        require(bytes.size >= 64 && elf.getInt(0) == 0x464c457f && bytes[5] == 1.toByte()) {
            "Invalid little-endian ELF for $abi"
        }
        val is64 = bytes[4] == 2.toByte()
        require(is64 || bytes[4] == 1.toByte()) { "Unsupported ELF class for $abi" }
        val machine = elf.getShort(18).toInt() and 0xffff
        val expectedMachine = mapOf("arm64-v8a" to 183, "armeabi-v7a" to 40, "x86" to 3, "x86_64" to 62)
        require(machine == expectedMachine[abi] && is64 == (abi in setOf("arm64-v8a", "x86_64"))) {
            "ELF architecture does not match $abi"
        }
        fun index(value: Long): Int {
            require(value in 0..bytes.size.toLong()) { "ELF offset out of bounds for $abi" }
            return value.toInt()
        }
        fun word(offset: Int): Long = if (is64) elf.getLong(offset) else elf.getInt(offset).toLong() and 0xffffffffL
        val sections = index(word(if (is64) 40 else 32))
        val stride = elf.getShort(if (is64) 58 else 46).toInt() and 0xffff
        val count = elf.getShort(if (is64) 60 else 48).toInt() and 0xffff
        require(stride >= (if (is64) 64 else 40) && count > 0) { "Missing ELF sections for $abi" }
        fun section(i: Int): Int {
            require(i in 0 until count) { "Invalid ELF section for $abi" }
            return index(sections.toLong() + i.toLong() * stride).also {
                require(it.toLong() + stride <= bytes.size) { "Truncated ELF section for $abi" }
            }
        }
        for (i in 0 until count) {
            val table = section(i)
            if (elf.getInt(table + 4) != 11) continue
            val symbols = index(word(table + if (is64) 24 else 16))
            val size = word(table + if (is64) 32 else 20)
            val entrySize = word(table + if (is64) 56 else 36)
            require(entrySize >= (if (is64) 24 else 16) && size % entrySize == 0L) { "Invalid ELF symbols for $abi" }
            val stringsSection = section(elf.getInt(table + if (is64) 40 else 24))
            val strings = index(word(stringsSection + if (is64) 24 else 16))
            val stringsEnd = index(strings + word(stringsSection + if (is64) 32 else 20))
            for (n in 0 until size / entrySize) {
                val symbol = index(symbols + n * entrySize)
                val name = index(strings.toLong() + (elf.getInt(symbol).toLong() and 0xffffffffL))
                require(name < stringsEnd) { "Invalid ELF symbol name for $abi" }
                var end = name
                while (end < stringsEnd && bytes[end] != 0.toByte()) end++
                require(end < stringsEnd) { "Unterminated ELF symbol for $abi" }
                if (String(bytes, name, end - name, Charsets.US_ASCII) != getter) continue
                val info = bytes[symbol + if (is64) 4 else 12].toInt() and 0xff
                val visibility = bytes[symbol + if (is64) 5 else 13].toInt() and 3
                require((info and 0xf) == 2 && (info ushr 4) in 1..2 && visibility in setOf(0, 3)) {
                    "Getter is not a visible exported ELF function for $abi"
                }
                val codeSection = section(elf.getShort(symbol + if (is64) 6 else 14).toInt() and 0xffff)
                val flags = word(codeSection + 8)
                require(elf.getInt(codeSection + 4) == 1 && (flags and 6L) == 6L) {
                    "Getter is not in allocated executable ELF PROGBITS for $abi"
                }
                val value = word(symbol + if (is64) 8 else 4)
                val codeAddress = if (abi == "armeabi-v7a") value and -2L else value
                val address = word(codeSection + if (is64) 16 else 12)
                val offset = word(codeSection + if (is64) 24 else 16)
                val functionSize = word(symbol + if (is64) 16 else 8)
                val expected = when (abi) {
                    "arm64-v8a" -> "20008052c0035fd6"
                    "armeabi-v7a" -> "01207047"
                    else -> "b001c3"
                }.chunked(2).map { it.toInt(16).toByte() }.toByteArray()
                val sectionSize = word(codeSection + if (is64) 32 else 20)
                require(codeAddress >= address && functionSize == expected.size.toLong() &&
                    codeAddress - address + functionSize <= sectionSize) { "Unexpected compiled getter shape for $abi" }
                val start = index(offset + codeAddress - address)
                val finish = index(start.toLong() + functionSize)
                require(bytes.copyOfRange(start, finish).contentEquals(expected)) { "Compiled multistart getter is not ON for $abi" }
                return
            }
        }
        error("Missing compiled multistart JNI getter for $abi")
    }

    fun manifest(libraries: Map<String, ByteArray>): Map<String, Any> {
        require(libraries.keys == requiredAbis) { "Expected all four bridge ABIs, got ${libraries.keys}" }
        return linkedMapOf(
            "schemaVersion" to 1,
            "fourTagMultiStart" to true,
            "nativeLibraries" to libraries.toSortedMap().map { (abi, bytes) ->
                requireCompiledOn(bytes, abi)
                linkedMapOf("abi" to abi, "sha256" to sha256(bytes))
            }
        )
    }

    fun verifyAar(file: File) {
        ZipFile(file).use { zip ->
            val entries = zip.entries().asSequence().toList()
            require(entries.map { it.name }.distinct().size == entries.size) { "Duplicate AAR ZIP entries" }
            val libraries = entries.filter { it.name.matches(Regex("jni/[^/]+/libdjiunity\\.so")) }
                .associate { it.name.split('/')[1] to zip.getInputStream(it).use { stream -> stream.readBytes() } }
            val entry = zip.getEntry("assets/$manifestPath") ?: error("Missing bridge feature manifest")
            val actual = zip.getInputStream(entry).use { JsonSlurper().parse(it) }
            require(actual == manifest(libraries)) { "Bridge feature manifest does not match final packaged native libraries" }
        }
    }
}

abstract class GenerateBridgeFeatures : DefaultTask() {
    @get:InputFiles
    @get:PathSensitive(PathSensitivity.RELATIVE)
    abstract val mergedNativeLibraries: ConfigurableFileCollection

    @get:OutputDirectory
    abstract val outputDirectory: DirectoryProperty

    @TaskAction
    fun generate() {
        val files = mergedNativeLibraries.files.flatMap { root ->
            if (root.isDirectory) root.walkTopDown().filter { it.isFile && it.name == "libdjiunity.so" }.toList()
            else listOf(root).filter { it.name == "libdjiunity.so" }
        }
        require(files.map { it.parentFile.name }.distinct().size == files.size) { "Duplicate merged bridge ABI" }
        val manifest = BridgeFeatureProvenance.manifest(files.associate { it.parentFile.name to it.readBytes() })
        val output = outputDirectory.file(BridgeFeatureProvenance.manifestPath).get().asFile
        output.parentFile.mkdirs()
        output.writeText(JsonOutput.prettyPrint(JsonOutput.toJson(manifest)) + "\n", Charsets.UTF_8)
    }
}

abstract class VerifyBridgeFeatures : DefaultTask() {
    @get:InputFile
    @get:PathSensitive(PathSensitivity.NONE)
    abstract val aarFile: RegularFileProperty

    @TaskAction
    fun verify() {
        BridgeFeatureProvenance.verifyAar(aarFile.get().asFile)
        logger.lifecycle("Verified bridge feature manifest, all four ABI hashes and compiled JNI getters.")
    }
}

androidComponents {
    onVariants(selector().withBuildType("release")) { variant ->
        val generate = tasks.register<GenerateBridgeFeatures>("generateReleaseBridgeFeatures") {
            // Consume this invocation's merge-task outputs, not a guessed .cxx cache directory.
            dependsOn("mergeReleaseNativeLibs")
            mergedNativeLibraries.from(provider { tasks.getByName("mergeReleaseNativeLibs").outputs.files })
            outputDirectory.set(layout.buildDirectory.dir("generated/bridgeFeatures/${variant.name}/assets"))
        }
        variant.sources.assets?.addGeneratedSourceDirectory(generate, GenerateBridgeFeatures::outputDirectory)
        tasks.withType<Zip>().matching { it.name == "bundleReleaseAar" }.configureEach {
            // Preserve !* asset exclusion; explicitly package only the verified feature manifest.
            from(generate.flatMap { it.outputDirectory }) {
                include(BridgeFeatureProvenance.manifestPath)
                into("assets")
            }
        }
    }
}

val verifyReleaseBridgeFeatures = tasks.register<VerifyBridgeFeatures>("verifyReleaseBridgeFeatures") {
    dependsOn("bundleReleaseAar")
    aarFile.set(layout.buildDirectory.file("outputs/aar/DJIUnityBridge.aar"))
}
tasks.matching { it.name == "bundleReleaseAar" }.configureEach { finalizedBy(verifyReleaseBridgeFeatures) }
tasks.register("rebuildRelease") {
    group = "build"
    description = "Build and verify the standard shipping AAR; use --rerun-tasks --no-build-cache for a fresh rebuild."
    dependsOn(verifyReleaseBridgeFeatures)
}

dependencies {
    compileOnly(files("libs/unity-classes.jar"))

    // Prefab package used by the native bridge for the calibrated IPPE square PnP solver.
    implementation("org.opencv:opencv:4.11.0")

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
