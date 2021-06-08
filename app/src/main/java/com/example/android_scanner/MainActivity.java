package com.example.android_scanner;

import androidx.appcompat.app.AppCompatActivity;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.View;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.TextView;

import com.example.android_scanner.databinding.ActivityMainBinding;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    private ActivityMainBinding binding;
    private Bitmap srcBitmap;
    private Bitmap dstBitmap;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        srcBitmap = BitmapFactory.decodeResource(this.getResources(),R.drawable.mountain);

        // Example of a call to a native method
        ImageView iv = binding.imageView2;
        iv.setImageBitmap(srcBitmap);

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
    }

    public void doFlip(View view){
        flip(srcBitmap,srcBitmap);
        doBlur();
    }

    public void doBlur(){

        float sigma = (float) (binding.seekBar.getProgress() / 10.0);
        if(sigma<0.1){
            sigma= (float) 0.1;
        }

        blur(srcBitmap, dstBitmap, sigma);
        binding.imageView2.setImageBitmap(dstBitmap);
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
    public native void flip(Bitmap bitmapIn, Bitmap bitmapOut);
    public native void blur(Bitmap bitmapIn, Bitmap bitmapOut, float sigma);
}