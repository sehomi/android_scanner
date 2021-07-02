package com.example.android_scanner;

// TODO: adding descriptions and comments
// TODO: manage string.xml
// TODO: add navigation marker for activities
// TODO: attitude frequency increase

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
import android.graphics.SurfaceTexture;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.ImageView;
import android.widget.Toast;

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

import dji.common.flightcontroller.Attitude;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.gimbal.GimbalState;
import dji.common.product.Model;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;

public class AircraftActivity extends AppCompatActivity implements OnMapReadyCallback, TextureView.SurfaceTextureListener {


    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    private ActivityAircraftBinding binding;
    private Bitmap srcBitmap;
    private Bitmap dstBitmap;
    private Boolean isProcessing = false;

    LocationManager locationManager;

    private static final String TAG = AircraftActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    protected DJICodecManager mCodecManager = null;
    protected DJICodecManager.OnGetBitmapListener bitmapListener = null;


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

        createScanner(getIntent().getStringExtra("Assets"));

        // Example of a call to a native method
        ImageView iv = binding.imageView2;
        iv.setImageBitmap(dstBitmap);

        // The callback for receiving the raw H264 video data for camera live view
        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if (mCodecManager != null) {
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };

        if (binding.cameraPreview != null) {
            binding.cameraPreview.setSurfaceTextureListener(this);
        }

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

    @Override
    public void onSurfaceTextureAvailable(@NonNull SurfaceTexture surfaceTexture, int i, int i1) {
        Log.e(TAG, "onSurfaceTextureAvailable");
        if (mCodecManager == null) {
            mCodecManager = new DJICodecManager(this, surfaceTexture, i, i1);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(@NonNull SurfaceTexture surfaceTexture, int i, int i1) {
        Log.e(TAG, "onSurfaceTextureSizeChanged");
    }

    @Override
    public boolean onSurfaceTextureDestroyed(@NonNull SurfaceTexture surfaceTexture) {
        Log.e(TAG,"onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }

        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(@NonNull SurfaceTexture surfaceTexture) {

        if (!isProcessing) {
            Thread thread = new Thread() {
                @Override
                public void run() {
                    isProcessing = true;
                    processBitmap(binding.cameraPreview.getBitmap());
                    isProcessing = false;
                }
            };

            thread.start();
        }

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

//            binding.textView2.setText(s);
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

    private void processBitmap(Bitmap bitmap){

        Bitmap bitmap1 = bitmap.copy(bitmap.getConfig(), true);
        detect(bitmap, bitmap1);

        this.runOnUiThread(new Runnable() {

            @Override
            public void run() {
                binding.imageView2.setImageBitmap(bitmap1);
            }
        });

    }


    @Override
    public void onMapReady(@NonNull GoogleMap googleMap) {
//        LatLng sydney = new LatLng(-33.852, 151.211);
//        googleMap.addMarker(new MarkerOptions()
//                .position(sydney)
//                .title("Marker in Sydney"));
        googleMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);
    }

    protected void onProductChange() {
        initPreviewer();
        initState();
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initPreviewer();
        onProductChange();

        if(binding.cameraPreview == null) {
            Log.e(TAG, "mVideoSurface is null");
        }
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        uninitPreviewer();
        super.onPause();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    public void onReturn(View view){
        Log.e(TAG, "onReturn");
        this.finish();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        uninitPreviewer();
        super.onDestroy();
    }

    private void initState() {
        FlightController flightController = CameraApplication.getFlightControllerInstance();
        Gimbal gimbal = CameraApplication.getGimbalInstance();

        if (flightController != null){

            FlightControllerState.Callback fcallback = new FlightControllerState.Callback() {
                @Override
                public void onUpdate(@NonNull FlightControllerState flightControllerState) {
//                    Log.i(TAG, "FlightController state is updated.");

                    Attitude attitude = flightControllerState.getAttitude();

                    double roll = attitude.roll;
                    double pitch = attitude.pitch;
                    double yaw = attitude.yaw;

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            binding.textView.setText("\nroll: "+String.valueOf(roll)+"\npitch: "+String.valueOf(pitch)+"\nyaw: "+String.valueOf(yaw));
                        }
                    });

                    LocationCoordinate3D location = flightControllerState.getAircraftLocation();
                    double lat = location.getLatitude();
                    double lon = location.getLongitude();
                    float alt = location.getAltitude();

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            binding.textView2.setText("Longitude: " + String.valueOf(lon) + "\nLatitude: " + String.valueOf(lat) + "\nAltitude: " + String.valueOf(alt));
                        }
                    });
                }
            };

            flightController.setStateCallback(fcallback);
        }
        else {
            Log.e(TAG, "FlightController is null.");
        }

        if (gimbal != null){
            GimbalState.Callback gcallback = new GimbalState.Callback() {
                @Override
                public void onUpdate(@NonNull GimbalState gimbalState) {
                    Log.i(TAG, "GimbalState is updated.");

                    double groll = gimbalState.getAttitudeInDegrees().getRoll();
                    double gpitch = gimbalState.getAttitudeInDegrees().getPitch();
                    double gyaw = gimbalState.getAttitudeInDegrees().getYaw();

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            binding.textView3.setText("\ngimbal roll: "+String.valueOf(groll)+"\ngimbal pitch: "+String.valueOf(gpitch)+"\ngimbal yaw: "+String.valueOf(gyaw));
                        }
                    });
                }
            };

            gimbal.setStateCallback(gcallback);
        }
        else{
            Log.e(TAG, "Gimbal is null.");
        }
    }

    private void initPreviewer() {

        BaseProduct product = CameraApplication.getProductInstance();

        if (product == null || !product.isConnected()) {
            showToast(getString(R.string.disconnected));
        } else {
            if (binding.cameraPreview != null) {
                binding.cameraPreview.setSurfaceTextureListener(this);
            }
            if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
            }
        }
    }

    private void uninitPreviewer() {
        Camera camera = CameraApplication.getCameraInstance();
        if (camera != null){
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    private void showToast(final String toastMsg) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), toastMsg, Toast.LENGTH_LONG).show();

            }
        });
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native void createScanner(String assets);
    public native void detect(Bitmap bitmapIn, Bitmap bitmapOut);
//    public native void setImage(Bitmap bitmap, double time);
//    public native void setLocation(double lat, double lng, double time);
//    public native void setOrientation(double roll, double pitch, double azimuth, double time);


}