#include <jni.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <android/log.h>
#include <ctime>
#include <cstdint>
#include <cstring>

#ifndef UNITY_INTERFACE_API
#define UNITY_INTERFACE_API
#endif

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "DJIUnity", __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, "DJIUnity", __VA_ARGS__)

static JavaVM* g_vm = nullptr;

static GLuint g_texId = 0;
static jobject g_surfaceTexture = nullptr;
static jmethodID g_updateTexImage = nullptr;
static jmethodID g_getTransformMatrix = nullptr;
static float g_texTransform[16] = {
    1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f
};

static uint64_t g_updates = 0;
static uint64_t g_prevUpdateMs = 0;
static uint64_t g_lastHeartbeatMs = 0;

static uint64_t NowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static bool GetEnvScoped(JNIEnv** outEnv) {
    *outEnv = nullptr;
    if (!g_vm) return true;

    JNIEnv* env = nullptr;
    jint res = g_vm->GetEnv((void**)&env, JNI_VERSION_1_6);
    if (res == JNI_OK && env) {
        *outEnv = env;
        return true;
    }

    if (g_vm->AttachCurrentThread(&env, nullptr) != JNI_OK || !env) {
        *outEnv = nullptr;
        return true;
    }

    *outEnv = env;
    return false;
}

static void DetachIfNeeded(bool alreadyAttached) {
    if (!alreadyAttached && g_vm) {
        g_vm->DetachCurrentThread();
    }
}

static void LogGlError(const char* where) {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        LOGW("DJIBridgeNative - GL error at %s: 0x%x", where, err);
    }
}

static void ResetTransformMatrix() {
    const float identity[16] = {
        1.f, 0.f, 0.f, 0.f,
        0.f, 1.f, 0.f, 0.f,
        0.f, 0.f, 1.f, 0.f,
        0.f, 0.f, 0.f, 1.f
    };
    std::memcpy(g_texTransform, identity, sizeof(identity));
}

static void ClearJavaRefs(JNIEnv* env) {
    if (!env) return;
    if (g_surfaceTexture) {
        env->DeleteGlobalRef(g_surfaceTexture);
        g_surfaceTexture = nullptr;
    }
    g_updateTexImage = nullptr;
    g_getTransformMatrix = nullptr;
    ResetTransformMatrix();
}

jint JNI_OnLoad(JavaVM* vm, void*) {
    g_vm = vm;
    LOGI("DJIBridgeNative - JNI_OnLoad called, VM stored.");
    return JNI_VERSION_1_6;
}

extern "C" {

JNIEXPORT void JNICALL DJI_SetSurfaceTexture(void* stPtr) {
    JNIEnv* env = nullptr;
    bool alreadyAttached = GetEnvScoped(&env);
    if (!env) {
        LOGW("DJIBridgeNative - DJI_SetSurfaceTexture: no JNIEnv");
        return;
    }

    jobject localObj = (jobject)stPtr;
    if (!localObj) {
        LOGW("DJIBridgeNative - DJI_SetSurfaceTexture: stPtr is null");
        DetachIfNeeded(alreadyAttached);
        return;
    }

    ClearJavaRefs(env);
    g_surfaceTexture = env->NewGlobalRef(localObj);
    if (!g_surfaceTexture) {
        LOGW("DJIBridgeNative - DJI_SetSurfaceTexture: NewGlobalRef failed");
        DetachIfNeeded(alreadyAttached);
        return;
    }

    jclass stClass = env->GetObjectClass(g_surfaceTexture);
    if (!stClass) {
        LOGW("DJIBridgeNative - DJI_SetSurfaceTexture: GetObjectClass failed");
        ClearJavaRefs(env);
        DetachIfNeeded(alreadyAttached);
        return;
    }

    g_updateTexImage = env->GetMethodID(stClass, "updateTexImage", "()V");
    g_getTransformMatrix = env->GetMethodID(stClass, "getTransformMatrix", "([F)V");
    env->DeleteLocalRef(stClass);

    if (!g_updateTexImage) {
        LOGW("DJIBridgeNative - DJI_SetSurfaceTexture: updateTexImage method not found");
    }
    if (!g_getTransformMatrix) {
        LOGW("DJIBridgeNative - DJI_SetSurfaceTexture: getTransformMatrix method not found");
    }

    g_updates = 0;
    g_prevUpdateMs = 0;
    g_lastHeartbeatMs = 0;

    LOGI("DJIBridgeNative - DJI_SetSurfaceTexture globalref=%p", g_surfaceTexture);

    DetachIfNeeded(alreadyAttached);
}

JNIEXPORT void JNICALL DJI_ClearSurfaceTexture() {
    JNIEnv* env = nullptr;
    bool alreadyAttached = GetEnvScoped(&env);
    if (!env) {
        LOGW("DJIBridgeNative - DJI_ClearSurfaceTexture: no JNIEnv");
        return;
    }

    ClearJavaRefs(env);

    g_updates = 0;
    g_prevUpdateMs = 0;
    g_lastHeartbeatMs = 0;

    LOGI("DJIBridgeNative - DJI_ClearSurfaceTexture done");

    DetachIfNeeded(alreadyAttached);
}

JNIEXPORT void JNICALL DJI_BeginCreateOESTexture(int reqW, int reqH) {
    LOGI("DJIBridgeNative - DJI_BeginCreateOESTexture called: %dx%d", reqW, reqH);
}

static void UNITY_INTERFACE_API OnRenderEvent(int eventID) {
    if (eventID == 1) {
        if (g_texId != 0) {
            GLuint old = g_texId;
            glDeleteTextures(1, &old);
            g_texId = 0;
            LogGlError("glDeleteTextures(create)");
        }

        glGenTextures(1, &g_texId);
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, g_texId);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        GLenum err = glGetError();
        LOGI("DJIBridgeNative - Created OES tex id=%u err=0x%x", (unsigned)g_texId, err);
        return;
    }

    if (eventID == 2) {
        if (!g_surfaceTexture || !g_updateTexImage) return;

        if (g_texId != 0) {
            glBindTexture(GL_TEXTURE_EXTERNAL_OES, g_texId);
            LogGlError("glBindTexture(update)");
        }

        JNIEnv* env = nullptr;
        bool alreadyAttached = GetEnvScoped(&env);
        if (!env) return;

        env->CallVoidMethod(g_surfaceTexture, g_updateTexImage);
        if (env->ExceptionCheck()) {
            env->ExceptionClear();
            LOGW("DJIBridgeNative - updateTexImage threw exception");
            DetachIfNeeded(alreadyAttached);
            return;
        }

        if (g_getTransformMatrix) {
            jfloatArray matrixArray = env->NewFloatArray(16);
            if (matrixArray) {
                env->CallVoidMethod(g_surfaceTexture, g_getTransformMatrix, matrixArray);
                if (env->ExceptionCheck()) {
                    env->ExceptionClear();
                    LOGW("DJIBridgeNative - getTransformMatrix threw exception");
                    ResetTransformMatrix();
                } else {
                    env->GetFloatArrayRegion(matrixArray, 0, 16, g_texTransform);
                }
                env->DeleteLocalRef(matrixArray);
            }
        }

        const uint64_t now = NowMs();
        const uint64_t gap = (g_prevUpdateMs == 0) ? 0 : (now - g_prevUpdateMs);
        g_prevUpdateMs = now;
        g_updates++;

        if (g_lastHeartbeatMs == 0 || (now - g_lastHeartbeatMs) >= 1000) {
            g_lastHeartbeatMs = now;
            LOGI("DJIBridgeNative - updateTexImage ok updates=%llu gapMs=%llu tex=%u st=%p",
                 (unsigned long long)g_updates,
                 (unsigned long long)gap,
                 (unsigned)g_texId,
                 g_surfaceTexture);
        }

        DetachIfNeeded(alreadyAttached);
        return;
    }

    if (eventID == 3) {
        if (g_texId != 0) {
            GLuint old = g_texId;
            glDeleteTextures(1, &old);
            g_texId = 0;

            GLenum err = glGetError();
            LOGI("DJIBridgeNative - Destroyed OES tex err=0x%x", err);
        }
        return;
    }
}

typedef void (UNITY_INTERFACE_API *RenderEventFunc)(int);

JNIEXPORT RenderEventFunc JNICALL DJI_GetRenderEventFunc() { return OnRenderEvent; }
JNIEXPORT int JNICALL DJI_GetTextureId() { return (int)g_texId; }
JNIEXPORT void JNICALL DJI_GetSurfaceTextureTransform(float* out16) {
    if (!out16) return;
    std::memcpy(out16, g_texTransform, sizeof(g_texTransform));
}

} // extern "C"
