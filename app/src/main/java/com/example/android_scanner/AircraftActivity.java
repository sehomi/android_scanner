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
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
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
import android.widget.RadioGroup;
import android.widget.Toast;

import com.codemonkeylabs.fpslibrary.TinyDancer;
import com.example.android_scanner.databinding.ActivityAircraftBinding;
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

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

import dji.common.flightcontroller.Attitude;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.GPSSignalLevel;
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
    Polygon fov_polygon = null;
    Polygon sweep_polygon = null;
    private GoogleMap googleMap = null;

    List<Marker> AllMarkers = new ArrayList<Marker>();

    private Bitmap srcBitmap;
    private Bitmap dstBitmap;
    private Marker aircraft = null;
    private Marker user = null;
    private Boolean isProcessing = false;

    LocationManager locationManager;

    private static final String TAG = AircraftActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    protected DJICodecManager mCodecManager = null;
    protected DJICodecManager.OnGetBitmapListener bitmapListener = null;

    private double aircraftYaw = 0.0;
    private RadioGroup r_group;
    private String detMode = "OBJECT_DETECTION";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.v(TAG, "-------///1");
        super.onCreate(savedInstanceState);
        Log.v(TAG, "-------///2");
        binding = ActivityAircraftBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        initUI();
        Log.v(TAG, "-------///3");
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        TinyDancer.create()
                .show(this);

        // Get the SupportMapFragment and request notification when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        srcBitmap = BitmapFactory.decodeResource(this.getResources(), R.drawable.mountain);
        dstBitmap = srcBitmap.copy(srcBitmap.getConfig(), true);
        Log.v(TAG, "-------///4");
        createScanner(getIntent().getStringExtra("Assets"), getIntent().getStringExtra("Log"), getIntent().getBooleanExtra("Log Mode", false), (float) 66.0, getIntent().getIntExtra("Algorithm", 0));
//        createScanner(getIntent().getStringExtra("Assets"), "", getIntent().getBooleanExtra("Log Mode", false), (float) 66.0, getIntent().getIntExtra("Algorithm", 0));

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
        Log.v(TAG, "-------///5");
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

                    double curr_time = -1;
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                        Instant ins = Instant.now() ;
                        curr_time = ins.getEpochSecond() + (ins.getNano()/1e9);
                    }

                    Bitmap bitmap = binding.cameraPreview.getBitmap();
                    // TODO: set image time to it's exact message arrival
                    setImage(bitmap, curr_time);
                    processBitmap(bitmap);

                    isProcessing = false;
                }
            };

            thread.start();
        }

    }

    public synchronized AircraftActivity getInstance(){
        return this;
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

            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    if (googleMap != null) {
                        if (user == null) {
                            LatLng user_pos = new LatLng(lat, lng);
                            user = googleMap.addMarker(new MarkerOptions()
                                    .position(user_pos)
                                    .anchor(0.5f,0.5f)
                                    .title("User Position")
                                    .icon(BitmapDescriptorFactory.fromResource(R.drawable.blue_circle_icon)));
                            googleMap.animateCamera(CameraUpdateFactory.newLatLngZoom(user_pos, 18));
                        }
                        else
                        {
                            LatLng user_pos = new LatLng(lat, lng);
                            user.setPosition(user_pos);
                        }
                    }

                    binding.textView6.setText(s);
                }
            });

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
        Bitmap bitmap2 = bitmap.copy(bitmap.getConfig(), true);
//        detect(bitmap, bitmap1);
        double[][] object_poses;
        if (r_group.getCheckedRadioButtonId() == R.id.objDetMode)
            object_poses = scan(bitmap1, bitmap2, 0);
        else
            object_poses = scan(bitmap1, bitmap2, 1);

        visualize(object_poses, bitmap2, bitmap1, false);

        this.runOnUiThread(new Runnable() {

            @Override
            public void run() {
                binding.imageView2.setImageBitmap(bitmap1);
                binding.motionImageView.setImageBitmap(bitmap2);
            }
        });

    }


    @Override
    public void onMapReady(@NonNull GoogleMap gMap) {
        googleMap = gMap;
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

    private void initUI()
    {
        r_group = (RadioGroup) findViewById(R.id.detectionMode);
    }

//    public void onRgClick(View v)
//    {
//        switch (v.getId()) {
//
//            case R.id.btn_open: {
//
//                if (r_group.getCheckedRadioButtonId() == R.id.objDetMode){
//                    detMode = "OBJECT_DETECTION";
//                }
//                else if (r_group.getCheckedRadioButtonId() == R.id.motDetMode){
//                    detMode = "MOTION_DETECTION";
//                }
//                break;
//            }
//            default:
//                break;
//        }
//    }

    private void initState() {
        FlightController flightController = CameraApplication.getFlightControllerInstance();
        Gimbal gimbal = CameraApplication.getGimbalInstance();

        if (flightController != null){

            FlightControllerState.Callback fcallback = new FlightControllerState.Callback() {
                @Override
                public void onUpdate(@NonNull FlightControllerState flightControllerState) {
//                    Log.i(TAG, "FlightController state is updated.");
                    double curr_time = -1;
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                        Instant ins = Instant.now() ;
                        curr_time = ins.getEpochSecond() + (ins.getNano()/1e9);
                    }

                    Attitude attitude = flightControllerState.getAttitude();

                    double roll = attitude.roll;
                    double pitch = attitude.pitch;
                    double yaw = attitude.yaw;
                    aircraftYaw = yaw;
//                    double[][] fov = setOrientation(roll, pitch, yaw, curr_time);

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

                    setLocation(lat, lon, alt, curr_time);

                    GPSSignalLevel gpsLevel = flightControllerState.getGPSSignalLevel();

                    if (gpsLevel == GPSSignalLevel.LEVEL_3 || gpsLevel == GPSSignalLevel.LEVEL_4 || gpsLevel == GPSSignalLevel.LEVEL_5){

                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                if (googleMap != null) {
                                    if (aircraft == null) {
                                        LatLng ac_pos = new LatLng(lat, lon);
                                        aircraft = googleMap.addMarker(new MarkerOptions()
                                                .position(ac_pos)
                                                .anchor(0.5f,0.5f)
                                                .title("Aircraft Position")
                                                .rotation((float) yaw)
                                                .icon(BitmapDescriptorFactory.fromResource(R.drawable.aircraft)));
                                        googleMap.animateCamera(CameraUpdateFactory.newLatLngZoom(ac_pos, 18));
                                    }
                                    else
                                    {
                                        LatLng ac_pos = new LatLng(lat, lon);
                                        aircraft.setPosition(ac_pos);
                                        aircraft.setRotation((float) yaw);
                                    }
                                }

                                binding.textView2.setText("Longitude: " + String.valueOf(lon) + "\nLatitude: " + String.valueOf(lat) + "\nAltitude: " + String.valueOf(alt));
                            }
                        });
                    }
                    else
                    {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                binding.textView2.setText("GPS Weak or Unavailabe.");
                            }
                        });
                    }

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

                    double curr_time = -1;
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                        Instant ins = Instant.now() ;
                        curr_time = ins.getEpochSecond() + (ins.getNano()/1e9);
                    }

                    gimbalState.getYawRelativeToAircraftHeading();
                    double groll = gimbalState.getAttitudeInDegrees().getRoll();
                    double gpitch = gimbalState.getAttitudeInDegrees().getPitch();
//                    double gyaw = gimbalState.getAttitudeInDegrees().getYaw() + gimbalState.getYawRelativeToAircraftHeading();
                    double gyaw = aircraftYaw;

                    double[][] fov = setOrientation(groll, gpitch, gyaw, curr_time);
                    visualize(fov, null, null, true);
                    Log.e(TAG, "onResume");
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

    private void visualize(double[][] markers, Bitmap movingsBitmap, Bitmap processedBitmap, boolean fovCall)
    {
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                if (!fovCall) {
                    binding.motionImageView.setImageBitmap(movingsBitmap);
                    binding.imageView2.setImageBitmap(processedBitmap);
                    Log.v(TAG, "vis -- obj call");
                }
                else
                {
                    Log.v(TAG, "vis -- fov call");
                }

                if (markers != null && googleMap != null) {
                    if (fovCall)
                    {
                        if ( fov_polygon == null)
                            googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(markers[0][0], markers[0][1]), 18));
                        else{
                            fov_polygon.remove();
//                            sweep_polygon.remove();
                        }
                    }
                    else
                    {
                        for (Marker mLocationMarker: AllMarkers) {
                            mLocationMarker.remove();
                        }
                        AllMarkers.clear();
                    }

                    PolygonOptions fov_polygon_opt = new PolygonOptions();
                    PolygonOptions sweep_polygon_opt = new PolygonOptions();

                    for (double[] marker : markers) {
                        Log.v(TAG, String.valueOf(marker[0]) + " " + String.valueOf(marker[1]) + " " + String.valueOf(marker[3]));
                        // TODO: A new marker must be assigned to moving objects (in which marker[3] == 4)
                        if (marker[3] == 0 || marker[3] == 4) {

                            LatLng per = new LatLng(marker[0], marker[1]);
                            MarkerOptions locMarker = new MarkerOptions();
                            locMarker.position(per);
                            locMarker.anchor(0.5f,0.5f);
                            locMarker.icon(BitmapDescriptorFactory.fromResource(R.drawable.red_circle_icon));
                            locMarker.title("Person");
                            Marker mm = googleMap.addMarker(locMarker);
                            AllMarkers.add(mm);
                            Log.v(TAG, "as person");
                        } else if (marker[3] == 1) {
                            Log.v(TAG, "as car");
                        } else if (marker[3] == 2) {
                            Log.v(TAG, "as fov");
                            fov_polygon_opt.add(new LatLng(marker[0], marker[1]));
                        } else if (marker[3] == 3) {
                            Log.v(TAG, "as sweeped");
                            sweep_polygon_opt.add(new LatLng(marker[0], marker[1]));
                        }
                    }

                    if (fovCall)
                    {
                        fov_polygon_opt.add(new LatLng(markers[0][0], markers[0][1]));
                        sweep_polygon_opt.add(new LatLng(markers[4][0], markers[4][1]));

                        sweep_polygon_opt.fillColor(Color.argb(150, 255, 0, 0));
                        sweep_polygon_opt.strokeColor(Color.argb(255, 150, 150, 150));
                        fov_polygon_opt.strokeColor(Color.BLUE);

//                        sweep_polygon = googleMap.addPolygon(sweep_polygon_opt);
                        fov_polygon = googleMap.addPolygon(fov_polygon_opt);
                    }

                }
            }
        });
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
    public native void createScanner(String assets, String logs, boolean log_mode, float hva, int method);
    public native void detect(Bitmap bitmapIn, Bitmap bitmapOut);
    public native double[][] scan(Bitmap detections, Bitmap movings_img, int detMode);
    public native void setImage(Bitmap bitmap, double time);
    public native void setLocation(double lat, double lng, double alt, double time);
    public native double[][] setOrientation(double roll, double pitch, double azimuth, double time);


}