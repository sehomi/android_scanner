package com.example.android_scanner;

import android.graphics.Bitmap;

public class InfoWindowData {

    public Bitmap img;
    public String type;
    public double lat;
    public double lng;
    public double dist;

    public InfoWindowData(Bitmap bitmap, String tp, double lt, double lg, double ds)
    {
        img = bitmap;
        type = tp;
        lat = lt;
        lng = lg;
        dist = ds;
    }
};
