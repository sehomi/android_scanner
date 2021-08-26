package com.example.android_scanner;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.LinearLayout;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.model.Marker;

public class ObjectInfoWindow extends LinearLayout implements GoogleMap.InfoWindowAdapter {

    private final View mWindow;

    public ObjectInfoWindow(Context context) {
        super(context);

        LayoutInflater inflater = (LayoutInflater) context
                .getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        mWindow = inflater.inflate(R.layout.object_info_window_view, null);
    }


    @Nullable
    @Override
    public View getInfoWindow(@NonNull Marker marker) {
        return mWindow;
    }

    @Nullable
    @Override
    public View getInfoContents(@NonNull Marker marker) {
        return null;
    }
}
