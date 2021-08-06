package com.example.android_scanner;

import android.Manifest;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.RadioGroup;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.log.DJILog;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;

public class ConnectionActivity extends Activity {

    private static final String TAG = ConnectionActivity.class.getName();

    private TextView mTextConnectionStatus;
    private TextView mTextProduct;
    private TextView mVersionTv;
    private Button mBtnOpen;
    private Button mBtnOpenPhone;
    private ProgressBar pbar;
    private TextView fileTextView;
    private RadioGroup rgroup;
    private Switch sw;

    private static final String[] REQUIRED_PERMISSION_LIST = new String[]{
            Manifest.permission.VIBRATE,
            Manifest.permission.INTERNET,
            Manifest.permission.ACCESS_WIFI_STATE,
            Manifest.permission.WAKE_LOCK,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_NETWORK_STATE,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.CHANGE_WIFI_STATE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.READ_PHONE_STATE,
            Manifest.permission.CAMERA,
    };
    private List<String> missingPermission = new ArrayList<>();
    private AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private static final int REQUEST_PERMISSION_CODE = 12345;
    private String assetsDir;
    private String logDir;

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        checkAndRequestPermissions();
        setContentView(R.layout.activity_connection);
        initUI();

        Thread thread = new Thread() {
            @Override
            public void run() {

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        pbar.setProgress(0);
                        pbar.setVisibility(View.VISIBLE);
                        fileTextView.setText("Starting to copy files ...");
                        fileTextView.setVisibility(View.VISIBLE);
                    }
                });

                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                assetsDir = copyAssets();

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        pbar.setProgress(100);
                        fileTextView.setText("Finished Copying Files.");
                    }
                });

                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        pbar.setVisibility(View.INVISIBLE);
                        fileTextView.setVisibility(View.INVISIBLE);
                        mBtnOpenPhone.setEnabled(true);
                    }
                });
            }
        };
        thread.start();

        // Register the broadcast receiver for receiving the device connection's changes.
        IntentFilter filter = new IntentFilter();
        filter.addAction(CameraApplication.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        // TODO: modify this section. bug occured in nokia device
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");
        LocalDateTime now = LocalDateTime.now();
        String folderName = "log_" + dtf.format(now);
        logDir = getStorageDir(folderName);
//        logDir = "";
        Log.v(TAG, "logDir: ........................ "+logDir);

    }

    public String getStorageDir(String fn)
    {
        //create folder
        File file = new File(Environment.getExternalStorageDirectory() + "/LogFolder");
        if (!file.exists()) {
            if (!file.mkdirs()) {
                Log.v(TAG, "Unable to create the folder! ...");
                return null;
            }
        }

        File file1 = new File(Environment.getExternalStorageDirectory() + "/LogFolder/" + fn);

        if (!file1.exists()) {
            if (!file1.mkdirs()) {
                Log.v(TAG, "Unable to create the folder! ...");
                return null;
            }
        }
        return file1.getAbsolutePath() + File.separator;
    }

    /**
     * Checks if there is any missing permissions, and
     * requests runtime permission if needed.
     */
    private void checkAndRequestPermissions() {
        // Check for permissions
        for (String eachPermission : REQUIRED_PERMISSION_LIST) {
            if (ContextCompat.checkSelfPermission(this, eachPermission) != PackageManager.PERMISSION_GRANTED) {
                missingPermission.add(eachPermission);
            }
        }
        // Request for missing permissions
        if (!missingPermission.isEmpty() && Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this,
                    missingPermission.toArray(new String[missingPermission.size()]),
                    REQUEST_PERMISSION_CODE);
        }

    }

    /**
     * Result of runtime permission request
     */
    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE) {
            for (int i = grantResults.length - 1; i >= 0; i--) {
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED) {
                    missingPermission.remove(permissions[i]);
                }
            }
        }
        // If there is enough permission, we will start the registration
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else {
            showToast("Missing permissions!!!");
        }
    }

    private void startSDKRegistration() {
        if (isRegistrationInProgress.compareAndSet(false, true)) {
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    showToast( "registering, pls wait...");
                    DJISDKManager.getInstance().registerApp(getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                        @Override
                        public void onRegister(DJIError djiError) {
                            if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                                DJILog.e("App registration", DJISDKError.REGISTRATION_SUCCESS.getDescription());
                                DJISDKManager.getInstance().startConnectionToProduct();
                                showToast("Register Success");
                            } else {
                                showToast( "Register sdk fails, check network is available");
                            }
                            Log.v(TAG, djiError.getDescription());
                        }

                        @Override
                        public void onProductDisconnect() {
                            Log.d(TAG, "onProductDisconnect");
                            showToast("Product Disconnected");

                        }
                        @Override
                        public void onProductConnect(BaseProduct baseProduct) {
                            Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                            showToast("Product Connected");

                        }

                        @Override
                        public void onProductChanged(BaseProduct baseProduct) {

                        }

                        @Override
                        public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent,
                                                      BaseComponent newComponent) {

                            if (newComponent != null) {
                                newComponent.setComponentListener(new BaseComponent.ComponentListener() {

                                    @Override
                                    public void onConnectivityChange(boolean isConnected) {
                                        Log.d(TAG, "onComponentConnectivityChanged: " + isConnected);
                                    }
                                });
                            }
                            Log.d(TAG,
                                    String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s",
                                            componentKey,
                                            oldComponent,
                                            newComponent));

                        }
                        @Override
                        public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {

                        }

                        @Override
                        public void onDatabaseDownloadProgress(long l, long l1) {

                        }
                    });
                }
            });
        }
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
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


    private void initUI() {
        mTextConnectionStatus = (TextView) findViewById(R.id.text_connection_status);
        mTextProduct = (TextView) findViewById(R.id.text_product_info);

        mVersionTv = (TextView) findViewById(R.id.textView2);
        mVersionTv.setText(getResources().getString(R.string.sdk_version, DJISDKManager.getInstance().getSDKVersion()));

        mBtnOpen = (Button) findViewById(R.id.btn_open);
        mBtnOpen.setEnabled(false);

        mBtnOpenPhone = (Button) findViewById(R.id.btn_open_phone);
        mBtnOpenPhone.setEnabled(false);

        pbar = (ProgressBar) findViewById(R.id.progressBar2);
        fileTextView = (TextView) findViewById(R.id.textView4);

        pbar.setVisibility(View.INVISIBLE);
        fileTextView.setVisibility(View.INVISIBLE);

        sw = (Switch) findViewById(R.id.switch1);

        rgroup = (RadioGroup) findViewById(R.id.radioGroup2);
    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            refreshSDKRelativeUI();
        }
    };

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        unregisterReceiver(mReceiver);
        super.onDestroy();
    }

    private void refreshSDKRelativeUI() {
        BaseProduct mProduct = CameraApplication.getProductInstance();

        if (null != mProduct && mProduct.isConnected()) {
            Log.v(TAG, "refreshSDK: True");
            mBtnOpen.setEnabled(true);

            String str = mProduct instanceof Aircraft ? "DJIAircraft" : "DJIHandHeld";
            mTextConnectionStatus.setText("Status: " + str + " connected");

            if (null != mProduct.getModel()) {
                mTextProduct.setText("" + mProduct.getModel().getDisplayName());
            } else {
                mTextProduct.setText(R.string.product_information);
            }

        } else {
            Log.v(TAG, "refreshSDK: False");
            mBtnOpen.setEnabled(false);

            mTextProduct.setText(R.string.product_information);
            mTextConnectionStatus.setText(R.string.connection_loose);
        }
    }

    public void onClick(View v) {
        switch (v.getId()) {

            case R.id.btn_open: {
                Intent intent = new Intent(this, AircraftActivity.class);
                intent.putExtra("Assets", assetsDir);
                intent.putExtra("Log", logDir);
                intent.putExtra("Log Mode", sw.isChecked());

                int algorithm = 0;
                if (rgroup.getCheckedRadioButtonId() == R.id.yolov3Button){
                    algorithm = 0;
                }
                else if (rgroup.getCheckedRadioButtonId() == R.id.yolov3TinyButton){
                    algorithm = 1;
                }
                else if (rgroup.getCheckedRadioButtonId() == R.id.mnSsdButton){
                    algorithm = 2;
                }
                intent.putExtra("Algorithm", algorithm);

                startActivity(intent);
                break;
            }
            default:
                break;
        }
    }

    public void onClickPhone(View v) {
        switch (v.getId()) {

            case R.id.btn_open_phone: {
                Intent intent = new Intent(this, MainActivity.class);
                intent.putExtra("Assets", assetsDir);
                intent.putExtra("Log", logDir);
                intent.putExtra("Log Mode", sw.isChecked());

                int algorithm = 0;
                if (rgroup.getCheckedRadioButtonId() == R.id.yolov3Button){
                    algorithm = 0;
                }
                else if (rgroup.getCheckedRadioButtonId() == R.id.yolov3TinyButton){
                    algorithm = 1;
                }
                else if (rgroup.getCheckedRadioButtonId() == R.id.mnSsdButton){
                    algorithm = 2;
                }
                intent.putExtra("Algorithm", algorithm);

                startActivity(intent);
                break;
            }
            default:
                break;
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

    private String copyAssets() {
        AssetManager assetManager = getAssets();
        String[] files = null;
        try {
            files = assetManager.list("");
        } catch (IOException e) {
            // TODO: handle assets copying failure
            Log.e("tag", "Failed to get asset file list.", e);
        }

        int count = 0;
        for(String filename : files) {
            InputStream in = null;
            OutputStream out = null;
            try {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        fileTextView.setText("Copying file: " + filename);
                    }
                });

                File outFile = new File(getExternalFilesDir(null), filename);

                if (!outFile.exists()) {
                    in = assetManager.open(filename);
                    out = new FileOutputStream(outFile);
                    copyFile(in, out);
                    in.close();
                    in = null;
                    out.flush();
                    out.close();
                    out = null;
                }

                int percentage = (int)( ((float)count)/ files.length ) * 100;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        pbar.setProgress(percentage);
                    }
                });

            } catch(IOException e) {
                // TODO: handle assets copying failure
                Log.e("tag", "Failed to copy asset file: " + filename, e);
            }

            count += 1;
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
}