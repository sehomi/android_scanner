package com.example.android_scanner;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.model.Marker;


public class ObjectInfoWindow extends LinearLayout implements GoogleMap.InfoWindowAdapter {

    private View mWindow;
    private ImageView imageView;
    private TextView objTypeTxt;
    private TextView latTxt;
    private TextView lngTxt;
    private TextView distTxt;

    public ObjectInfoWindow(Context context) {
        super(context);

        LayoutInflater inflater = (LayoutInflater) context
                .getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        mWindow = inflater.inflate(R.layout.object_info_window_view, null);
        imageView = mWindow.findViewById(R.id.imageView);
        objTypeTxt = mWindow.findViewById(R.id.objTypeTxt);
        latTxt = mWindow.findViewById(R.id.latitudeTxt);
        lngTxt = mWindow.findViewById(R.id.longitudeTxt);
        distTxt = mWindow.findViewById(R.id.distanceTxt);
    }


    @Nullable
    @Override
    public View getInfoWindow(@NonNull Marker marker) {

        InfoWindowData data = (InfoWindowData) marker.getTag();

        imageView.setImageBitmap(data.img);
        objTypeTxt.setText(data.type);
        latTxt.setText(String.format("%.7f",data.lat));
        lngTxt.setText(String.format("%.7f",data.lng));
        distTxt.setText(String.format("%.1f",data.dist));

        return mWindow;
    }

    @Nullable
    @Override
    public View getInfoContents(@NonNull Marker marker) {
        return null;
    }
}
