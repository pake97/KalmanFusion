/**
 * KalmanFusion by Amedeo Pachera
 * Last version 11/07/2019
 * info: amedeopachera@gmail.com
 *
 * StartActivity.java: this activity coolects datas, waits for altitude coordinates (wich often comes later then otehrs)
 * and send to MainActivity the first position used for coordinates conversion.
 */
package com.bignerd.android.sensor;


import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.hardware.SensorEventListener;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.Sensor;
import android.content.Context;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;



import static android.hardware.Sensor.TYPE_GYROSCOPE;
import static android.hardware.Sensor.TYPE_LINEAR_ACCELERATION;



public class StartActivity extends AppCompatActivity {
    private static final String TAG="StartActivity";

    private static final int REQUEST_LOCATION_PERMISSION =1 ;
    private SensorManager sensorManager;
    private Sensor sensorG;
    private Sensor sensorA;
    private SensorEventListener eventListener;
    private float[] data = new float[10];
    public static float[] data0=new float[10]; //static variable used in MainActivity

    @Override
    protected void onCreate(Bundle savedInstanceState) throws SecurityException {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_start);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        // request permission for location
        if (ActivityCompat.checkSelfPermission(this.getApplicationContext(), android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED){
            ActivityCompat.requestPermissions(this, new String[] {android.Manifest.permission.ACCESS_FINE_LOCATION}, REQUEST_LOCATION_PERMISSION);
        }
        // Acquire a reference to the system Location Manager
        final LocationManager locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

        // Define a listener that responds to location updates
        final LocationListener locationListener = new LocationListener() {
            public void onLocationChanged(Location location) {
                //implementa
                //makeUseOfNewLocation(location);
            }

            public void onStatusChanged(String provider, int status, Bundle extras) {
                Log.d(TAG,"stato provider cambiato");
            }

            public void onProviderEnabled(String provider) {
                Log.d(TAG,"provider non disponibile");
            }

            public void onProviderDisabled(String provider) {
                Log.d(TAG,"provider disattivato");
            }
        };

        final String locationProvider = LocationManager.GPS_PROVIDER;


        locationManager.requestLocationUpdates(locationProvider, 0, 0, locationListener);
        // Gestore degli eventi
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        sensorG = sensorManager.getDefaultSensor(TYPE_GYROSCOPE);
        sensorA = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        eventListener = new SensorEventListener()  {
            @Override
            public void onSensorChanged(SensorEvent event) throws SecurityException {
                int tipo = event.sensor.getType();
                data[0] = event.timestamp;
                Location lastKnownLocation = locationManager.getLastKnownLocation(locationProvider);
                if(lastKnownLocation!=null) {
                    data[7] = (float) lastKnownLocation.getLatitude();
                    data[8] = (float) lastKnownLocation.getLongitude();
                    data[9] = (float) lastKnownLocation.getAltitude();
                }
                switch (tipo) {
                    // Event came from the light sensor.
                    case TYPE_LINEAR_ACCELERATION:
                        // Handle light sensor
                        data[4] = event.values[0];
                        data[5] = event.values[1];
                        data[6] = event.values[2];
                        break;
                    case TYPE_GYROSCOPE:
                        data[1] = event.values[0]; // X
                        data[2] = event.values[1]; // Y
                        data[3] = event.values[2]; // Z
                        break;

                    default:
                        // do nothing

                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
                Log.i("MainActivity","Precisione del sensore cambiata");
            }
        };

        sensorManager.registerListener(eventListener, sensorG, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(eventListener, sensorA, SensorManager.SENSOR_DELAY_FASTEST);

    }

    @Override
    protected void onResume(){
        super.onResume();
        sensorManager.registerListener(eventListener, sensorG, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(eventListener, sensorA, SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    protected void onPause(){
        super.onPause();
        //locationManager.removeUpdates(locationListener);
        sensorManager.unregisterListener(eventListener);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        switch (requestCode) {
            case REQUEST_LOCATION_PERMISSION:

                // If the permission is granted, get the location, otherwise,
                // show a Toast
                if (grantResults.length > 0
                        && grantResults[0]
                        == PackageManager.PERMISSION_GRANTED) {
                    //
                } else {
                    Toast.makeText(this,
                            R.string.location_permission_denied,
                            Toast.LENGTH_SHORT).show();
                    System.exit(0);
                }
                break;
        }
    }


    public void startAct(View view) {
        Log.e(TAG,"BOTTONE PREMUTO!");
        if(data[9]==0.0) // if altitude is aviable
        {
            Toast.makeText(this,"Wait for GPS-calibration",Toast.LENGTH_LONG).show();
        }
        else{
        Intent myIntent = new Intent(view.getContext(), MainActivity.class);
        data0=data.clone();
        startActivityForResult(myIntent, 0);}
    }
}
