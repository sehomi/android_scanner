package com.example.android_scanner;

import android.app.Application;
import android.content.Context;

import com.secneo.sdk.Helper;

public class MApplication extends Application {

    private CameraApplication cameraApplication;
    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);
        Helper.install(MApplication.this);
        if (cameraApplication == null) {
            cameraApplication = new CameraApplication();
            cameraApplication.setContext(this);
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
        cameraApplication.onCreate();
    }

}