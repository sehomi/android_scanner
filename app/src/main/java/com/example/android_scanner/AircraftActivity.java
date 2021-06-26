package com.example.android_scanner;

// TODO: adding descriptions and comments
// TODO: manage string.xml

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import android.Manifest;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;

import com.codemonkeylabs.fpslibrary.TinyDancer;
import com.example.android_scanner.databinding.ActivityAircraftBinding;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.time.Instant;

public class AircraftActivity extends AppCompatActivity implements OnMapReadyCallback {


    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    private ActivityAircraftBinding binding;
    private Bitmap srcBitmap;
    private Bitmap dstBitmap;

    LocationManager locationManager;

    private static final String TAG = AircraftActivity.class.getName();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityAircraftBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        TinyDancer.create()
                .show(this);

        // Get the SupportMapFragment and request notification when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        srcBitmap = BitmapFactory.decodeResource(this.getResources(), R.drawable.mountain);
        dstBitmap = srcBitmap.copy(srcBitmap.getConfig(), true);

        // TODO: move assets copying to connection activity
        String assetsDir = copyAssets();
        createScanner(assetsDir);

        // Example of a call to a native method
        ImageView iv = binding.imageView2;
        iv.setImageBitmap(dstBitmap);

        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        LocationListener locationListener = new MyLocationListener();

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 5000, 10, locationListener);

    }

    private class MyLocationListener implements LocationListener {

        @Override
        public void onLocationChanged(Location loc) {

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                Instant ins = Instant.now() ;
            }
            String longitude = "Longitude: " + loc.getLongitude();
            Log.v(TAG, longitude);
            String latitude = "Latitude: " + loc.getLatitude();
            Log.v(TAG, latitude);

            String s = longitude + "\n" + latitude ;

            double lat = loc.getLatitude();
            double lng = loc.getLongitude();

            binding.textView2.setText(s);
        }

        @Override
        public void onProviderDisabled(String provider) {
            // TODO: disable distance calculation
        }

        @Override
        public void onProviderEnabled(String provider) {
            // TODO: enable distance calculation
        }

        @Override
        public void onStatusChanged(String provider, int status, Bundle extras) {
            // TODO: check if distance to home calculation is operable in new status
        }
    }

//    private void decodeBytesToImage(){
//        if(!isImgBytesReady)
//            return;
//
//        Camera.Parameters parameters = mCamera.getParameters();
//        int width = parameters.getPreviewSize().width;
//        int height = parameters.getPreviewSize().height;
//
//        YuvImage yuv = new YuvImage(imgSet.imgBytes, parameters.getPreviewFormat(), width, height, null);
////        YuvImage yuv = new YuvImage(imgBytes, parameters.getPreviewFormat(), width, height, null);
//
//        ByteArrayOutputStream out = new ByteArrayOutputStream();
//        yuv.compressToJpeg(new Rect(0, 0, width, height), 50, out);
//
//        byte[] bts = out.toByteArray();
//        final Bitmap bitmap = BitmapFactory.decodeByteArray(bts, 0, bts.length);
//        int x =0;
//
//        Bitmap bitmap1 = bitmap.copy(bitmap.getConfig(), true);
//        detect(bitmap, bitmap1);
//        MainActivity.this.runOnUiThread(new Runnable() {
//
//            @Override
//            public void run() {
////                ((ImageView) findViewById(R.id.loopback)).setImageBitmap(bitmap1);
//                binding.imageView2.setImageBitmap(bitmap1);
//            }
//        });
////        binding.imageView2.setImageBitmap(bitmap1);
//
//    }


    private String copyAssets() {
        AssetManager assetManager = getAssets();
        String[] files = null;
        try {
            files = assetManager.list("");
        } catch (IOException e) {
            Log.e("tag", "Failed to get asset file list.", e);
        }
        for(String filename : files) {
            InputStream in = null;
            OutputStream out = null;
            try {
                in = assetManager.open(filename);
                File outFile = new File(getExternalFilesDir(null), filename);
                out = new FileOutputStream(outFile);
                copyFile(in, out);
                in.close();
                in = null;
                out.flush();
                out.close();
                out = null;
            } catch(IOException e) {
                Log.e("tag", "Failed to copy asset file: " + filename, e);
            }
        }

        return getExternalFilesDir(null).getAbsolutePath();
    }

    private void copyFile(InputStream in, OutputStream out) throws IOException {
        byte[] buffer = new byte[1024];
        int read;
        while((read = in.read(buffer)) != -1){
            out.write(buffer, 0, read);
        }
    }

    @Override
    public void onMapReady(@NonNull GoogleMap googleMap) {
        LatLng sydney = new LatLng(-33.852, 151.211);
        googleMap.addMarker(new MarkerOptions()
                .position(sydney)
                .title("Marker in Sydney"));

    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native void createScanner(String assets);
    public native void detect(Bitmap bitmapIn, Bitmap bitmapOut);
    public native void setImage(Bitmap bitmap, double time);
    public native void setLocation(double lat, double lng, double time);
    public native void setOrientation(double roll, double pitch, double azimuth, double time);

    @Override
    public void onResume() {
        super.onResume();
    }

}