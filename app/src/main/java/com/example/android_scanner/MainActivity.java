package com.example.android_scanner;

// TODO: ask for location to be enabled
// TODO: elevations are to be fixed
// TODO: motion detection should be separated in main activity

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
import android.graphics.Color;
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
import android.view.WindowManager;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.codemonkeylabs.fpslibrary.TinyDancer;
import com.example.android_scanner.databinding.ActivityMainBinding;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

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
import java.util.Arrays;
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

    private class GroundLocation
    {
        public double lat = 0.0;
        public double lng = 0.0;
        public double elev = 0.0;
    };

    private class MarkerInfo
    {
        public double lat = 0.0;
        public double lng = 0.0;
        public double type = 0.0;
        public double dist = 0.0;
    };


    private imageSet imgSet = new imageSet();

    double orn_time;
    double loc_time;

    Polyline polyline = null;
    Polygon fov_polygon = null;
    Polygon sweep_polygon = null;
    GoogleMap googleMap = null;
    List<Marker> AllMarkers = new ArrayList<Marker>();
    List<MarkerInfo> AllMarkersInfo = new ArrayList<MarkerInfo>();

//    boolean readMode = false;

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

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR_LANDSCAPE);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

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
        Log.v(TAG, "---------41");
        createScanner(getIntent().getStringExtra("Assets"), getIntent().getStringExtra("Log"), getIntent().getIntExtra("Log Mode",2), getIntent().getIntExtra("Algorithm", 0), hva);
        Log.v(TAG, "---------42");
        Thread thread = new Thread() {
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

        thread.start();

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

//            if (readMode)
//                return;

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
//                double[][] fov = null;
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
//                    Log.v(TAG, "\n********** fov failed! \n");
                }
            }
            Log.v(TAG, "---------10");
        }
    };

    private void doScan(){
        Log.v(TAG, "---------11");
        if(!isImgBytesReady)// || readMode)
            return;

        Camera.Parameters parameters = mCamera.getParameters();

        int width = parameters.getPreviewSize().width;
        int height = parameters.getPreviewSize().height;

        YuvImage yuv = new YuvImage(imgSet.imgBytes, parameters.getPreviewFormat(), width, height, null);

        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuv.compressToJpeg(new Rect(0, 0, width, height), 50, out);

        byte[] bts = out.toByteArray();
        final Bitmap bitmap = BitmapFactory.decodeByteArray(bts, 0, bts.length);

        setImage(bitmap, imgSet.image_time);
        Bitmap bitmap1 = bitmap.copy(bitmap.getConfig(), true);
//        detect(bitmap, bitmap1);
        scan(bitmap1);
        MainActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Log.v(TAG, "---------65");
                binding.imageView2.setImageBitmap(bitmap1);
            }
        });
        Log.v(TAG, "---------12");
    }

    public void playLog(View view){
        Log.v(TAG, "---------this is playlog");
        MainActivity.this.runOnUiThread(new Runnable() {

            @Override
            public void run() {
                mCamera.stopPreview();
                isImgBytesReady = false;
//                mCamera = null;
            }
        });

        Thread read_thread = new Thread() {
            @Override
            public void run() {
//                Log.v(TAG, "---------this is read thread run");
                try {
//                    Log.v(TAG, "---------this is before try");
                    while (true) {
                        isImgBytesReady = false;
//                        Log.v(TAG, "---------this is while");
                        sleep(2);

                        Bitmap bitmap, processedBitmap, movingsBitmap;
                        bitmap = BitmapFactory.decodeResource(getResources(), R.drawable.mountain);
                        processedBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.mountain);
                        movingsBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.mountain);

                        double stamp = -1;
                        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
                            Instant ins = Instant.now();
                            stamp = ins.getEpochSecond() + (ins.getNano()/1e9);
                        }
                        else
                        {
                            Log.e(TAG, "run: Error. time stamp could not be calculated due to sdk version.");
                        }
                        GroundLocation elev = new GroundLocation();

                        // TODO: action should be considered for markers
                        double[][] fov = readLog(bitmap, processedBitmap, movingsBitmap, stamp, elev);
                        Bitmap[] objImages = getImages();

                        if (objImages != null)
                            Log.v(TAG, "objImages size: " + String.valueOf(objImages.length));

                        runOnUiThread(new Runnable() {

                            @Override
                            public void run() {
                                if (fov != null && googleMap != null) {
                                    binding.imageView3.setImageBitmap(movingsBitmap);
                                    binding.imageView2.setImageBitmap(processedBitmap);

                                    if (fov_polygon == null)
                                        googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(fov[0][0], fov[0][1]), 18));
//                                    if (polyline == null) {

//                                    googleMap.clear();

                                    for (Marker mLocationMarker: AllMarkers) {
                                        mLocationMarker.remove();
                                    }
                                    AllMarkers.clear();
                                    AllMarkersInfo.clear();

                                    if (fov_polygon != null)
                                        fov_polygon.remove();
                                    if (sweep_polygon != null)
                                        sweep_polygon.remove();

                                    PolygonOptions fov_polygon_opt = new PolygonOptions();
                                    PolygonOptions sweep_polygon_opt = new PolygonOptions();

                                    for(int i=0; i<fov.length; i++) {
//                                        Log.v(TAG, String.valueOf(fov[i][0]) + " " + String.valueOf(fov[i][1]) + " " + String.valueOf(fov[i][3]));
                                        // TODO: A new marker must be assigned to moving objects (in which fov[i][3] == 4) in read-log mode
                                        if (fov[i][3] == 0) {
//                                        if (fov[i][3] == 0 || fov[i][3] == 4) {
                                            LatLng per = new LatLng(fov[i][0], fov[i][1]);
                                            MarkerOptions locMarker = new MarkerOptions();
                                            locMarker.position(per);
                                            locMarker.anchor(0.5f,0.5f);
                                            locMarker.icon(BitmapDescriptorFactory.fromResource(R.drawable.red_circle_icon));
//                                            locMarker.title("Person");
                                            Marker mm = googleMap.addMarker(locMarker);
                                            AllMarkers.add(mm);
                                            MarkerInfo mi = new MarkerInfo();
                                            mi.lat = fov[i][0]; mi.lng = fov[i][1]; mi.type = fov[i][3]; mi.dist = fov[i][4];
                                            AllMarkersInfo.add(mi);
                                            Log.v(TAG, "as person");
                                        }
                                        else if(fov[i][3] == 1)
                                        {
                                            LatLng per = new LatLng(fov[i][0], fov[i][1]);
                                            MarkerOptions locMarker = new MarkerOptions();
                                            locMarker.position(per);
                                            locMarker.anchor(0.5f,0.5f);
                                            locMarker.icon(BitmapDescriptorFactory.fromResource(R.drawable.brown_rect_icon));
//                                            locMarker.title("Person");
                                            Marker mm = googleMap.addMarker(locMarker);

                                            AllMarkers.add(mm);
                                            MarkerInfo mi = new MarkerInfo();
                                            mi.lat = fov[i][0]; mi.lng = fov[i][1]; mi.type = fov[i][3]; mi.dist = fov[i][4];
                                            AllMarkersInfo.add(mi);
                                            Log.v(TAG, "as car");
                                        }
                                        else if(fov[i][3] == 2)
                                        {
                                            fov_polygon_opt.add(new LatLng(fov[i][0], fov[i][1]));
                                        }
                                        else if(fov[i][3] == 3)
                                        {
                                            sweep_polygon_opt.add(new LatLng(fov[i][0], fov[i][1]));
                                        }
                                    }

                                    fov_polygon_opt.add(new LatLng(fov[0][0], fov[0][1]));
//                                    sweep_polygon_opt.add(new LatLng(fov[4][0], fov[4][1]));

                                    sweep_polygon_opt.fillColor(Color.argb(150, 100, 100, 100));
                                    sweep_polygon_opt.strokeColor(Color.argb(255, 255, 255, 255));
                                    fov_polygon_opt.fillColor(Color.argb(100, 255, 255, 255));
                                    fov_polygon_opt.strokeColor(Color.BLACK);

                                    binding.textView7.setText(String.valueOf(sweep_polygon_opt.getPoints().size()));

                                    binding.textView9.setText(String.valueOf(elev.elev));
                                    if (elev.elev == 0) {
                                        binding.textView9.setTextColor(Color.BLACK);
                                    }
                                    else if (elev.elev == -32768){
                                        binding.textView9.setTextColor(Color.RED);
                                        binding.textView9.setText("No File");
                                    }
                                    else{
                                        binding.textView9.setTextColor(Color.GREEN);
                                    }

                                    if (sweep_polygon_opt.getPoints().size() > 0) {
                                        sweep_polygon = googleMap.addPolygon(sweep_polygon_opt);
                                    }
                                    if (fov_polygon_opt.getPoints().size() > 0) {
                                        fov_polygon = googleMap.addPolygon(fov_polygon_opt);
                                    }

                                }
                            }
                        });
                    }
                } catch (InterruptedException e) {
                    Log.e(TAG, "\n********** readlog run failed! \n"+e.getMessage());
                    e.printStackTrace();
                }
            }
        };

        read_thread.start();

    }


    @Override
    public void onMapReady(@NonNull GoogleMap ggleMap) {

        googleMap = ggleMap;
        googleMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);

//        ObjectInfoWindow oiw = new ObjectInfoWindow(this);
//        googleMap.setInfoWindowAdapter(oiw);
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
    public native void createScanner(String assets, String logs, int log_mode, int method, float hva);
//    public native void detect(Bitmap bitmapIn, Bitmap bitmapOut);
    public native void scan(Bitmap detections);
    public native void setImage(Bitmap bitmap, double time);
    public native void setLocation(double lat, double lng, double alt, double time);
//    public native boolean setOrientation(double roll, double pitch, double azimuth, double time, Double[][] oa);
    public native double[][] setOrientation(double roll, double pitch, double azithmu, double time);
    public native double[][] readLog(Bitmap bitmap, Bitmap processedBitmap, Bitmap movingsBitmap, double stamp, GroundLocation elev);
    public native Bitmap[] getImages();

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


