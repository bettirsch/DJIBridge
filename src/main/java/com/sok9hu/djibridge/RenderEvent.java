package com.sok9hu.djibridge;

import android.graphics.SurfaceTexture;
import android.view.Surface;

import kotlin.jvm.JvmStatic;

/**
 * Bridge class that creates an Android {@link Surface} from a Unity texture
 * for DJI video decoder rendering.
 */
public class RenderEvent {
    private static SurfaceTexture st;
    private static Surface surface;

    @JvmStatic
    public static void create(int texId) {
        st = new SurfaceTexture(texId);
        surface = new Surface(st);
    }

    @JvmStatic
    public static Surface getSurface() {
        return surface;
    }
}
