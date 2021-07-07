package com.example.android_scanner;

// TODO: ask for location to be enabled
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.annotation.SuppressLint;
import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Address;
import android.location.Geocoder;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.hardware.usb.UsbManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.codemonkeylabs.fpslibrary.TinyDancer;
import com.example.android_scanner.databinding.ActivityMainBinding;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.time.Instant;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.time.format.FormatStyle;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;

public class MainActivity extends AppCompatActivity implements OnMapReadyCallback, Camera.PreviewCallback, SurfaceHolder.Callback {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    private ActivityMainBinding binding;
    private Bitmap srcBitmap;
    private Bitmap dstBitmap;
    private Camera mCamera;
    private byte[] imgBytes;
    private boolean isImgBytesReady = false;
    private SensorManager sensorManager;
    private SensorManager mSensorManager;
    Sensor accelerometer;
    Sensor magnetometer;

    // Gravity rotational data
    private float gravity[];
    // Magnetic rotational data
    private float magnetic[]; //for magnetic rotational data
    private float accels[] = new float[3];
    private float mags[] = new float[3];
    private float[] values = new float[3];

    // azimuth, pitch and roll
    private float azimuth;
    private float pitch;
    private float roll;

    private EditText editLocation = null;
    private ProgressBar pb = null;

    LocationManager locationManager;
//    LocationListener locationListener;

    private class imageSet
    {
        public byte[] imgBytes;
        public double image_time;      // = Instant.now() ;
        public Instant ins;
//        public double nano;
    };

    private imageSet imgSet = new imageSet();

    double orn_time;
    double loc_time;

//    DateTimeFormatter formatter =
//            DateTimeFormatter.ofLocalizedDateTime( FormatStyle.SHORT )
//                    .withLocale( Locale.UK )
//                    .withZone( ZoneId.systemDefault() );
    private static final String TAG = MainActivity.class.getName();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Log.v(TAG, "---------1");

        binding = ActivityMainBinding.inflate(getLayoutInflater());
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

        // Example of a call to a native method
        ImageView iv = binding.imageView2;
        iv.setImageBitmap(dstBitmap);

        SeekBar sb = binding.seekBar;
        sb.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                doBlur();
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        Log.v(TAG, "---------4");
        Context context = this;
        boolean camDetected = checkCameraHardware(context);

        // Create an instance of Camera


        // Create our Preview view and set it as the content of our activity.
//        mPreview = new CameraPreview(this, mCamera);
        SurfaceView preview = (SurfaceView) findViewById(R.id.camera_preview);
        int type = SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS;
        preview.getHolder().setType(type);//REQUIRED:API10
        preview.getHolder().addCallback(this);
//        preview.addView(mPreview);

        mCamera = getCameraInstance();
        float hva = mCamera.getParameters().getHorizontalViewAngle();
        createScanner(getIntent().getStringExtra("Assets"), getIntent().getStringExtra("Log"), getIntent().getIntExtra("Algorithm", 0), hva);

        Thread thread = new Thread() {
            @Override
            public void run() {
                try {
                    while (true) {
                        sleep(500);
                        decodeBytesToImage();
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };

        thread.start();

        Log.v(TAG, "---------5");
        Thread scan_thread = new Thread() {
            @Override
            public void run() {
                try {
                    while (true) {
                        sleep(500);
                        doScan();
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };

//        scan_thread.start();

        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null) {
            // Success! There's a magnetometer.
            int x = 0;
        } else {
            int x = 1;
            // Failure! No magnetometer.
        }

        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

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

        Log.v(TAG, "---------6");
    }

    private void doScan()
    {
        scan();
    }

    private class MyLocationListener implements LocationListener {

        @Override
        public void onLocationChanged(Location loc) {
            Log.v(TAG, "---------7");
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                Instant ins = Instant.now() ;
                loc_time = ins.getEpochSecond() + (ins.getNano()/1e9);
            }
            String longitude = "Longitude: " + loc.getLongitude();
            Log.v(TAG, longitude);
            String latitude = "Latitude: " + loc.getLatitude();
            Log.v(TAG, latitude);

            String s = longitude + "\n" + latitude ;

            double lat = loc.getLatitude();
            double lng = loc.getLongitude();
            double alt = loc.getAltitude();
            setLocation(lat, lng, alt, loc_time);
            // TODO: Check if altitude in measured ASL or AGL
            binding.textView2.setText(s);
            Log.v(TAG, "---------8");
        }

        @Override
        public void onProviderDisabled(String provider) {}

        @Override
        public void onProviderEnabled(String provider) {}

        @Override
        public void onStatusChanged(String provider, int status, Bundle extras) {}
    }

    private SensorEventListener mySensorEventListener = new SensorEventListener() {
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        public void onSensorChanged(SensorEvent event) {
            Log.v(TAG, "---------9");
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
//                ornSet.orn_time = Instant.now();
                Instant ins = Instant.now() ;
                orn_time = ins.getEpochSecond() + (ins.getNano()/1e9);
            }

            switch (event.sensor.getType()) {
                case Sensor.TYPE_MAGNETIC_FIELD:
                    mags = event.values.clone();
                    break;
                case Sensor.TYPE_ACCELEROMETER:
                    accels = event.values.clone();
                    break;
            }

            if (mags != null && accels != null) {
                gravity = new float[9];
                magnetic = new float[9];
                SensorManager.getRotationMatrix(gravity, magnetic, accels, mags);
                float[] outGravity = new float[9];
                SensorManager.remapCoordinateSystem(gravity, SensorManager.AXIS_X,SensorManager.AXIS_Z, outGravity);
                SensorManager.getOrientation(outGravity, values);

                azimuth = values[0] * 57.2957795f;
                pitch = values[1] * 57.2957795f;
                roll = values[2] * 57.2957795f;
                mags = null;
                accels = null;
                binding.textView.setText("\nroll: "+String.valueOf(roll)+"\npitch: "+String.valueOf(pitch)+"\nazimuth: "+String.valueOf(azimuth));
//                Double[][] fov = {{}};
//                boolean success = setOrientation(roll, pitch, azimuth, orn_time, fov);
                double[][] fov = setOrientation(roll, pitch, azimuth, orn_time);
                if (fov != null)
                {

                    Log.v(TAG, "********** fov:");

                    Log.v(TAG, String.valueOf(fov[0][0]) + " " + String.valueOf(fov[0][1]) + " " + String.valueOf(fov[0][2]));
                    Log.v(TAG, String.valueOf(fov[1][0]) + " " + String.valueOf(fov[1][1]) + " " + String.valueOf(fov[1][2]));
                    Log.v(TAG, String.valueOf(fov[2][0]) + " " + String.valueOf(fov[2][1]) + " " + String.valueOf(fov[2][2]));
                    Log.v(TAG, String.valueOf(fov[3][0]) + " " + String.valueOf(fov[3][1]) + " " + String.valueOf(fov[3][2]));


                }
                else
                {
                    Log.v(TAG, "\n********** fov failed: \n");
                }
                int x = 0;
            }
            Log.v(TAG, "---------10");
        }
    };

    private void decodeBytesToImage(){
        Log.v(TAG, "---------11");
        if(!isImgBytesReady)
            return;

        Camera.Parameters parameters = mCamera.getParameters();
//        float fl = parameters.getFocalLength();
//        Log.v(TAG, "focal length:   "+String.valueOf(fl));
        int width = parameters.getPreviewSize().width;
        int height = parameters.getPreviewSize().height;

//        double fl = (0.5 * width * (1.0 / Math.tan(Math.toRadians(hva/2))));
//        Log.v(TAG, "focal length:   "+String.valueOf(fl));

        YuvImage yuv = new YuvImage(imgSet.imgBytes, parameters.getPreviewFormat(), width, height, null);
//        YuvImage yuv = new YuvImage(imgBytes, parameters.getPreviewFormat(), width, height, null);

        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuv.compressToJpeg(new Rect(0, 0, width, height), 50, out);

        byte[] bts = out.toByteArray();
        final Bitmap bitmap = BitmapFactory.decodeByteArray(bts, 0, bts.length);
//        int x =0;
        setImage(bitmap, imgSet.image_time);
        Bitmap bitmap1 = bitmap.copy(bitmap.getConfig(), true);
        detect(bitmap, bitmap1);
        MainActivity.this.runOnUiThread(new Runnable() {

            @Override
            public void run() {
//                ((ImageView) findViewById(R.id.loopback)).setImageBitmap(bitmap1);
                binding.imageView2.setImageBitmap(bitmap1);
            }
        });
//        binding.imageView2.setImageBitmap(bitmap1);
        Log.v(TAG, "---------12");
    }

    public void doFlip(View view){
//        flip(srcBitmap,srcBitmap);
//        doBlur();
        detect(srcBitmap, dstBitmap);
        binding.imageView2.setImageBitmap(dstBitmap);
    }

    public void doBlur(){

        float sigma = (float) (binding.seekBar.getProgress() / 10.0);
        if(sigma<0.1){
            sigma= (float) 0.1;
        }

        blur(srcBitmap, dstBitmap, sigma);
        binding.imageView2.setImageBitmap(dstBitmap);
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
    public native String stringFromJNI();
    public native void createScanner(String assets, String logs, int method, float hva);
    public native void flip(Bitmap bitmapIn, Bitmap bitmapOut);
    public native void blur(Bitmap bitmapIn, Bitmap bitmapOut, float sigma);
    public native void detect(Bitmap bitmapIn, Bitmap bitmapOut);
    public native void scan();
    public native void setImage(Bitmap bitmap, double time);
    public native void setLocation(double lat, double lng, double alt, double time);
//    public native boolean setOrientation(double roll, double pitch, double azimuth, double time, Double[][] oa);
    public native double[][] setOrientation(double roll, double pitch, double azimuth, double time);

    @Override
    public void onResume() {
        super.onResume();
        mSensorManager.registerListener(mySensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(mySensorEventListener, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
//        getCameraInstance();

    }

    private boolean checkCameraHardware(Context context) {
        if (context.getPackageManager().hasSystemFeature(PackageManager.FEATURE_CAMERA)){
            // this device has a camera
            return true;
        } else {
            // no camera on this device
            return false;
        }
    }

    public static Camera getCameraInstance(){
        Camera c = null;
        try {
            c = Camera.open(); // attempt to get a Camera instance
        }
        catch (Exception e){
            // Camera is not available (in use or does not exist)
            int x = 1;
        }
        return c; // returns null if camera is unavailable
    }


    @Override
    public void onPreviewFrame(byte[] bytes, Camera camera) {
//        imgSet.image_time = Calendar.getInstance().getTime();
        long inst;
        int x;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
//            inst = Instant.now() ;
            imgSet.ins = Instant.now();
            imgSet.image_time = imgSet.ins.getEpochSecond() + (imgSet.ins.getNano()/1e9);
        }
        isImgBytesReady = false;
        imgSet.imgBytes = bytes;
//        imgBytes = bytes;
        isImgBytesReady = true;
    }

    @Override
    public void surfaceCreated(@NonNull SurfaceHolder surfaceHolder) {
        try
        {

            mCamera.setPreviewDisplay(surfaceHolder);
            mCamera.setPreviewCallback(this);
            mCamera.startPreview();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    @Override
    public void surfaceChanged(@NonNull SurfaceHolder surfaceHolder, int i, int i1, int i2) {
        try {
            mCamera.stopPreview();
//            Size s = mCamera.getParameters().getPreviewSize();
//            LayoutParams params = surface.getLayoutParams();
//            params.height = w*s.width/s.height; // portrait mode only
//            surface.setLayoutParams(params);
            mCamera.setPreviewDisplay(surfaceHolder);
            mCamera.setPreviewCallback(this);
            mCamera.startPreview();
        } catch (Exception ex) {
//            showException(ex);
        }
    }

    @Override
    public void surfaceDestroyed(@NonNull SurfaceHolder surfaceHolder) {

    }

}


