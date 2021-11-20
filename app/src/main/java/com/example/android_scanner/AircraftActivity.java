package com.example.android_scanner;

// TODO: adding descriptions and comments
// TODO: manage string.xml
// TODO: add navigation marker for activities
// TODO: attitude frequency increase

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
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
import android.view.WindowManager;
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

import dji.common.airlink.SignalQualityCallback;
import dji.common.battery.BatteryState;
import dji.common.flightcontroller.Attitude;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.GPSSignalLevel;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.gimbal.GimbalState;
import dji.common.model.LocationCoordinate2D;
import dji.common.product.Model;
import dji.sdk.airlink.AirLink;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

public class AircraftActivity extends AppCompatActivity implements OnMapReadyCallback, TextureView.SurfaceTextureListener {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    private class GroundLocation
    {
        public double lat = 0.0;
        public double lng = 0.0;
        public double elev = 0.0;
    };

    private ActivityAircraftBinding binding;
    Polygon fov_polygon = null;
    Polygon sweep_polygon = null;
    private GoogleMap googleMap = null;

//    List<Marker> AllMarkers = new ArrayList<Marker>();
    class MarkerSet {
        double time = 0.0;
        Marker marker;
    }
    List<MarkerSet> AllMarkers = new ArrayList<MarkerSet>();
    float markerShowTime = 20.0f;

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
    private boolean firstLoc = true;

    private boolean isMoving = false;
    private boolean isRotating = false;
    private float velLimit = 0.1f;

    private double groll_old = 0.0;
    private double gpitch_old = 0.0;
    private double gyaw_old = 0.0;
    private double groll_dif_lim = 1.0;
    private double gpitch_dif_lim = 1.0;
    private double gyaw_dif_lim = 1.0;

    double movementTime = 0.0;
    double img_time = 0.0;
    double motionDetectionDelay = 1.0;

    Bitmap[] objImages = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
//        Log.v(TAG, "-------///1");
        super.onCreate(savedInstanceState);
//        Log.v(TAG, "-------///2");
        binding = ActivityAircraftBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        initUI();
//        Log.v(TAG, "-------///3");
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
//        Log.v(TAG, "-------///4");
        createScanner(getIntent().getStringExtra("Assets"), getIntent().getStringExtra("Log"), getIntent().getIntExtra("Log Mode", 2), (float) 66.0, getIntent().getIntExtra("Algorithm", 0));

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
//        Log.v(TAG, "-------///5");
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

        changeMarkers();
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

                    img_time = -1;
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                        Instant ins = Instant.now() ;
                        img_time = ins.getEpochSecond() + (ins.getNano()/1e9);
                    }

                    Bitmap bitmap = binding.cameraPreview.getBitmap();
                    // TODO: set image time to it's exact message arrival
                    setImage(bitmap, img_time);
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
//            Log.v(TAG, longitude);
            String latitude = "Latitude: " + loc.getLatitude();
//            Log.v(TAG, latitude);

            String s = longitude + "\n" + latitude ;

            double lat = loc.getLatitude();
            double lng = loc.getLongitude();
            setUserLocation(lat, lng);

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

    @RequiresApi(api = Build.VERSION_CODES.O)
    private void processBitmap(Bitmap bitmap) {

        Bitmap bitmap1 = bitmap.copy(bitmap.getConfig(), true);
        Bitmap bitmap2 = bitmap.copy(bitmap.getConfig(), true);
        double[][] object_poses = null;
        if (r_group.getCheckedRadioButtonId() == R.id.objDetMode)
        {
//            Log.e(TAG, "object detection set");
            object_poses = scan(bitmap1, bitmap2, 0, false);
        }
        else {
//            Log.e(TAG, "motion detection set 1: *-*- "+String.valueOf(abs(movementTime-img_time)));
//            Log.e(TAG, "motion detection set 2: *-*- "+String.valueOf(abs(motionDetectionDelay)));

            if ((!isMoving && !isRotating) && abs(movementTime-img_time)>motionDetectionDelay)
            {
                object_poses = scan(bitmap1, bitmap2, 1, true);
//                Log.e(TAG, "motion - stable");
            }
            else {
                object_poses = scan(bitmap1, bitmap2, 1, false);
            }
            if (isMoving || isRotating)
            {
                movementTime = img_time;
//                Log.e(TAG, "motion - is moving");
            }
        }
//        Log.e(TAG, "---- ter before getimages");
        objImages = getImages();
//        Log.e(TAG, "---- ter before visualize obj call");
        visualize(object_poses, bitmap2, bitmap1, false);

        this.runOnUiThread(new Runnable() {

            @Override
            public void run() {
                binding.isMoving.setText(String.valueOf(isMoving)+","+String.valueOf(isRotating));
                binding.imageView2.setImageBitmap(bitmap1);
                binding.motionImageView.setImageBitmap(bitmap2);
            }
        });
    }

    @Override
    public void onMapReady(@NonNull GoogleMap gMap) {
        googleMap = gMap;
        googleMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);

        ObjectInfoWindow oiw = new ObjectInfoWindow(this);
        googleMap.setInfoWindowAdapter(oiw);
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

    private void initState() {
        FlightController flightController = CameraApplication.getFlightControllerInstance();
        Gimbal gimbal = CameraApplication.getGimbalInstance();
        Battery battery = CameraApplication.getBatteryInstance();
        AirLink link = CameraApplication.getLinkInstance();

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

//                    flightControllerState.getAttitude()
                    Attitude attitude = flightControllerState.getAttitude();
//                    String fms = flightControllerState.getFlightModeString();
                    int sat_cnt = flightControllerState.getSatelliteCount();
                    int goHomeHeight = flightControllerState.getGoHomeHeight();
//                    flightControllerState

                    float velX = flightControllerState.getVelocityX();
                    float velY = flightControllerState.getVelocityY();
                    float velZ = flightControllerState.getVelocityZ();
                    isMoving = abs(velX)>velLimit || abs(velY)>velLimit || abs(velZ)>velLimit;
//                    Log.e(TAG, "velocity z: --- "+String.valueOf(velZ));
//                    Log.e(TAG, "velocity z: --- "+String.valueOf(velY));
                    double vel = sqrt((velX*velX)+(velY*velY)+(velZ*velZ));
                    LocationCoordinate2D hom_loc = flightControllerState.getHomeLocation();
                    double home_lat = hom_loc.getLatitude();
                    double home_lon = hom_loc.getLongitude();

                    double roll = attitude.roll;
                    double pitch = attitude.pitch;
                    double yaw = attitude.yaw;
//                    Log.e(TAG, "rdf z: --- "+String.valueOf(yaw));
                    aircraftYaw = yaw;
//                    double[][] fov = setOrientation(roll, pitch, yaw, curr_time);

                    LocationCoordinate3D location = flightControllerState.getAircraftLocation();
                    double lat = location.getLatitude();
                    double lon = location.getLongitude();
                    float alt = location.getAltitude();

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            binding.textView.setText("\nroll: "+String.valueOf(roll)+"\npitch: "+String.valueOf(pitch)+"\nyaw: "+String.valueOf(yaw));
                            binding.numSatState.setText(String.valueOf(sat_cnt));
                            binding.heightState.setText(String.valueOf(alt) + " m");
                            binding.speedState.setText(String.format("%.2f",vel) + " m/s");
                        }
                    });

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
                                binding.latState.setText(String.format("%.4f",lat));
                                binding.lngState.setText(String.format("%.4f",lat));
                                if (firstLoc && googleMap != null) {
                                    googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(lat, lon), 18));
                                    firstLoc = false;
                                }
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

//                        gimbalState.y
//                    isRotating = !(gimbalState.isYawAtStop() && gimbalState.isRollAtStop() && gimbalState.isPitchAtStop());

                    gimbalState.getYawRelativeToAircraftHeading();
                    double groll = gimbalState.getAttitudeInDegrees().getRoll();
                    double gpitch = gimbalState.getAttitudeInDegrees().getPitch();
//                    double gyaw = gimbalState.getAttitudeInDegrees().getYaw() + gimbalState.getYawRelativeToAircraftHeading();
                    double gyaw = aircraftYaw;
                    GroundLocation elev = new GroundLocation();

                    double[][] fov = setOrientation(groll, gpitch, gyaw, curr_time, elev);

                    isRotating = abs(groll-groll_old)>groll_dif_lim || abs(gpitch-gpitch_old)>gpitch_dif_lim || abs(gyaw - gyaw_old)>gyaw_dif_lim;
                    groll_old = groll;
                    gpitch_old = gpitch;
                    gyaw_old = gyaw;


//                    Log.e(TAG, "---- ter before visualize fov call");
                    visualize(fov, null, null, true);
//                    Log.e(TAG, "onResume");
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            binding.textView3.setText("\ngimbal roll: "+String.valueOf(groll)+"\ngimbal pitch: "+String.valueOf(gpitch)+"\ngimbal yaw: "+String.valueOf(gyaw));

                            binding.textView11.setText(String.valueOf(elev.elev));
                            if (elev.elev == 0) {
                                binding.textView11.setTextColor(Color.BLACK);
                            }
                            else if (elev.elev == -32768){
                                binding.textView11.setTextColor(Color.RED);
                                binding.textView11.setText("No File");
                            }
                            else{
                                binding.textView11.setTextColor(Color.GREEN);
                            }
                        }
                    });
                }
            };

            gimbal.setStateCallback(gcallback);
        }
        else{
            Log.e(TAG, "Gimbal is null.");
        }

        if (battery != null)
        {
            BatteryState.Callback bcallback = new BatteryState.Callback() {
                @Override
                public void onUpdate(BatteryState batteryState) {
                    int bat_rem_charge = batteryState.getChargeRemainingInPercent();
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                        binding.batteryState.setText(String.valueOf(bat_rem_charge)+" %");
                        }
                    });
                }
            };

            battery.setStateCallback(bcallback);
        }
        else
        {
            Log.e(TAG, "Battery is null.");
        }

        if (link != null)
        {
            SignalQualityCallback scallback = new SignalQualityCallback() {
                @Override
                public void onUpdate(int i) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            binding.linkState.setText(String.valueOf(i)+" %");
                        }
                    });
                }
            };

            link.setDownlinkSignalQualityCallback(scallback);
        }
        else
        {
            Log.e(TAG, "AirLink is null.");
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
//                    Log.v(TAG, "vis -- obj call");
                }
                else
                {
//                    Log.v(TAG, "vis -- fov call");
                }

                if (markers != null && googleMap != null) {
                    if (fovCall)
                    {
//                        if ( fov_polygon == null)
//                            googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(markers[0][0], markers[0][1]), 18));

                        if (fov_polygon != null)
                            fov_polygon.remove();
                        if (sweep_polygon != null)
                            sweep_polygon.remove();
                    }
//                    else
//                    {

//                        for (Marker mLocationMarker: AllMarkers) {
//                            mLocationMarker.remove();
//                        }
//                        AllMarkers.clear();
//                    }
//                    Log.e(TAG, "---- ter 1.5");

                    PolygonOptions fov_polygon_opt = new PolygonOptions();
                    PolygonOptions sweep_polygon_opt = new PolygonOptions();

                    List<MarkerSet> newMarkers = new ArrayList<MarkerSet>();
//                    Log.e(TAG, "---- ter 1.75");

                    int i = -1;
                    for (double[] marker : markers) {
                        i += 1;
                        if (marker[3] == 0)
                        {
//                            Log.e(TAG, "---- ter 2");

                            if (marker[5] == 0) {
//                                Log.e(TAG, "---- ter 3");
                                continue;
                            }
                            LatLng per = new LatLng(marker[0], marker[1]);
                            MarkerOptions locMarker = new MarkerOptions();
                            locMarker.position(per);
//                            Log.e(TAG, "---- ter 4");

                            locMarker.anchor(0.5f,0.5f);
                            locMarker.icon(BitmapDescriptorFactory.fromResource(R.drawable.red_circle_icon));

                            MarkerSet mm = new MarkerSet();
                            mm.marker = googleMap.addMarker(locMarker);
//                            Log.e(TAG, "---- ter 6");

//                            mm.setTag(new InfoWindowData(BitmapFactory.decodeResource(getResources(), R.drawable.mountain) , "person", marker[0], marker[1], marker[4]));
                            mm.marker.setTag(new InfoWindowData(objImages[i] , "person", marker[0], marker[1], marker[4]));
//                            Log.e(TAG, "---- ter 7");

                            if (marker[5] == 1)
                            {
                                newMarkers.add(mm);
                                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                                    Instant ins = Instant.now();
                                    mm.time = ins.getEpochSecond() + (ins.getNano()/1e9);
                                }
                            }
                            else if (marker[5] == 2)
                            {
//                                Log.e(TAG, "---- ter 8");
                                int idx = (int)marker[6];
                                AllMarkers.get(idx).marker.remove();
                                AllMarkers.set(idx, mm);
                                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                                    Instant ins = Instant.now();
                                    mm.time = ins.getEpochSecond() + (ins.getNano()/1e9);
                                }
//                                Log.e(TAG, "---- ter 9");
                            }
                        }
                        else if (marker[3] == 1)
                        {
                            if (marker[5] == 0) continue;

                            LatLng per = new LatLng(marker[0], marker[1]);
                            MarkerOptions locMarker = new MarkerOptions();
                            locMarker.position(per);
                            locMarker.anchor(0.5f,0.5f);
                            locMarker.icon(BitmapDescriptorFactory.fromResource(R.drawable.brown_rect_icon));
//                                            locMarker.title("Person");
                            MarkerSet mm = new MarkerSet();
                            mm.marker = googleMap.addMarker(locMarker);
                            mm.marker.setTag(new InfoWindowData(objImages[i] , "car", marker[0], marker[1], marker[4]));

                            if (marker[5] == 1)
                            {
                                newMarkers.add(mm);
                                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                                    Instant ins = Instant.now();
                                    mm.time = ins.getEpochSecond() + (ins.getNano()/1e9);
                                }
                            }
                            else if (marker[5] == 2)
                            {
                                int idx = (int)marker[6];
                                AllMarkers.get(idx).marker.remove();
                                AllMarkers.set(idx, mm);
                                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                                    Instant ins = Instant.now();
                                    mm.time = ins.getEpochSecond() + (ins.getNano()/1e9);
                                }
                            }
                        }
                        else if (marker[3] == 2)
                        {
                            fov_polygon_opt.add(new LatLng(marker[0], marker[1]));
                        }
                        else if (marker[3] == 3)
                        {
                            sweep_polygon_opt.add(new LatLng(marker[0], marker[1]));
                        }
                        else if (marker[3] == 4)
                        {
                            if (marker[5] == 0)
                                continue;

                            LatLng mov = new LatLng(marker[0], marker[1]);
                            MarkerOptions locMarker = new MarkerOptions();
                            locMarker.position(mov);
//                            Log.e(TAG, "---- ter 4");
//
                            locMarker.anchor(0.5f,0.5f);
                            locMarker.icon(BitmapDescriptorFactory.fromResource(R.drawable.yellow_arrow));
                            locMarker.rotation((float) (aircraftYaw + marker[7]));
//                                            locMarker.title("Person");
//                            Log.e(TAG, "---- ter 5");
//
                            MarkerSet mm = new MarkerSet();
                            mm.marker = googleMap.addMarker(locMarker);
//                            Log.e(TAG, "---- ter 6");
//
                            mm.marker.setTag(new InfoWindowData(objImages[i] , "moving", marker[0], marker[1], marker[4]));
//                            Log.e(TAG, "---- ter 7");
//
                            if (marker[5] == 1)
                            {
                                newMarkers.add(mm);
                                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                                    Instant ins = Instant.now();
                                    mm.time = ins.getEpochSecond() + (ins.getNano()/1e9);
                                }
                            }
                            else if (marker[5] == 2)
                            {
//                                Log.e(TAG, "---- ter 8");
                                int idx = (int)marker[6];
                                AllMarkers.get(idx).marker.remove();
                                AllMarkers.set(idx, mm);
                                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                                    Instant ins = Instant.now();
                                    mm.time = ins.getEpochSecond() + (ins.getNano()/1e9);
                                }
//                                Log.e(TAG, "---- ter 9");
                            }
//                            Log.e(TAG, "---- ter 10");

//                            if (marker[5] == 0) continue;
//                            if (marker[5] == 1)
//                            {
//                                newMarkers.add(mm);
//                            }
//                            else if (marker[5] == 2)
//                            {
//                                int idx = (int)marker[6];
//                                AllMarkers.get(idx).remove();
//                                AllMarkers.set(idx, null);
//                            }
//                            Log.e(TAG, "---- ter 11");
                        }
//                        Log.e(TAG, "---- ter 12");
                    }

                    if (fovCall)
                    {
//                        Log.e(TAG, "---- ter 15");
                        fov_polygon_opt.add(new LatLng(markers[0][0], markers[0][1]));
//                        sweep_polygon_opt.add(new LatLng(markers[4][0], markers[4][1]));

                        sweep_polygon_opt.fillColor(Color.argb(150, 100, 100, 100));
                        sweep_polygon_opt.strokeColor(Color.argb(255, 255, 255, 255));
                        fov_polygon_opt.fillColor(Color.argb(100, 255, 255, 255));
                        fov_polygon_opt.strokeColor(Color.BLACK);

                        if (sweep_polygon_opt.getPoints().size() > 0) {
                            sweep_polygon = googleMap.addPolygon(sweep_polygon_opt);
                        }

                        if (fov_polygon_opt.getPoints().size() > 0) {
                            fov_polygon = googleMap.addPolygon(fov_polygon_opt);
                        }
//                        Log.e(TAG, "---- ter 16");
                    }
                    else
                    {
//                        Log.e(TAG, "---- ter 13");
                        AllMarkers.addAll(newMarkers);
//                        Log.e(TAG, "---- ter 14");
                    }

                }
            }
        });
    }

    public void changeMarkers() {
        Thread markers_thread = new Thread() {
            @Override
            public void run() {
                try {
                    while (true) {
                        sleep(500);
                        if (AllMarkers.isEmpty())
                            continue;

                        double now = 1e8;
                        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                            Instant ins = Instant.now();
                            now = ins.getEpochSecond() + (ins.getNano() / 1e9);
                        }

                        for (int counter = 0; counter < AllMarkers.size(); counter++) {
                            float markerOpacity = Math.max(0.0f, 1.0f - ((float) (now - AllMarkers.get(counter).time) / markerShowTime));

                            int finalCounter = counter;
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    AllMarkers.get(finalCounter).marker.setAlpha(markerOpacity);
                                }
                            });
                        }
//                            AllMarkers.get(counter);
                    }

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };
        markers_thread.start();
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
    public native void createScanner(String assets, String logs, int log_mode, float hva, int method);
    public native double[][] scan(Bitmap detections, Bitmap movings_img, int detMode, boolean isFix);
    public native void setImage(Bitmap bitmap, double time);
    public native void setLocation(double lat, double lng, double alt, double time);
    public native void setUserLocation(double lat, double lng);
    public native double[][] setOrientation(double roll, double pitch, double azimuth, double time, GroundLocation elev);
    public native Bitmap[] getImages();

}