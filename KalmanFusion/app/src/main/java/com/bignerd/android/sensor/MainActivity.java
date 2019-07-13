/**
* KalmanFusion by Amedeo Pachera
* Last version 11/07/2019
* info: amedeopachera@gmail.com
*
* MainActivity.java: this activity handles massage from and to the other services: it recieves data (vector with sensor datas and gps datas) synchronized from SyncService
 * and sends them to KalmanFusion wich returns the estimated position
 * This activity also collects datas with SensorManager and LocationManager, and then sends them asynchronusly to SyncService
 * This activity contains also methods for coordinates conversion: LLA (gps , WGS84) <--> ECEF <--> NED (kalman filter position)
 * To extends the coordinates conversion for long distances reset the ecefRef position every 2 km.
*/
package com.bignerd.android.sensor;


import android.annotation.SuppressLint;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.pm.ActivityInfo;
import android.hardware.SensorEventListener;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.SeekBar;
import android.widget.TextView;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.Sensor;
import android.content.Context;
import java.text.SimpleDateFormat;
import java.util.Date;
import static android.hardware.Sensor.TYPE_GYROSCOPE;
import static android.hardware.Sensor.TYPE_LINEAR_ACCELERATION;
import static com.bignerd.android.sensor.StartActivity.data0; // coordinates received from StartActivity used as ref position for ecef-ned conversion


public class MainActivity extends AppCompatActivity {
    private static final String TAG1 = MainActivity.class.getSimpleName();
    private SensorManager sensorManager;
    private Sensor sensorG;
    private Sensor sensorA;
    private SensorEventListener eventListener;
    private float[] data = new float[10]; //collector of sensor and gps datas
    private float[] datarcv = new float[10]; //data received from SyncService
    private Intent i,j; //to bound with services
    public static Animation bounce;
    public static LocationManager locationManager;
    public static LocationListener locationListener;
    private Messenger mService = null;
    private Messenger mService2 = null;
    private Messenger inMessenger;
    /**
     * Handle Message from and to Services
     */
    @SuppressLint("HandlerLeak")
    private Handler incomingMessages = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case SyncService.MSG_SERtoACT: //message from SyncService
                    Log.i(TAG1,"Sync data arrived from SyncService");
                    datarcv = (float[]) msg.obj; //message arrived
                    SimpleDateFormat sdf = new SimpleDateFormat("H:m:s:S"); //datarrcv[0] contains timestamps
                    Date resultdate = new Date();
                    t.setText(sdf.format(resultdate));
                    if (datarcv != null) {
                        wx.setText(String.format("Wx: %1$.6f", datarcv[1])); //angular velocity x
                        wy.setText(String.format("Wy: %1$.6f", datarcv[2])); //angular velocity y
                        wz.setText(String.format("Wz: %1$.6f", datarcv[3])); //angular velocity z
                        ax.setText(String.format("Ax: %1$.6f", datarcv[4])); //acceleration velocity x
                        ay.setText(String.format("Ay: %1$.6f", datarcv[5])); //acceleration velocity y
                        az.setText(String.format("Az: %1$.6f", datarcv[6])); //acceleration velocity z
                        g1.setText(String.format("lat GPS: %1$.6f", datarcv[7])); //latitude wgs84
                        g2.setText(String.format("long GPS: %1$.6f", datarcv[8])); //longitude wgs84
                        g3.setText(String.format("alt GPS: %1$.6f", datarcv[9]));  //altitude wgs84
                        try {
                                mService2.send(Message.obtain(null, KalmanService.MSG_ACTtoRAN,0,0,datarcv));  //send sync data to KalmanService
                        } catch (RemoteException e) {
                            e.printStackTrace();
                        }
                    }
                    break;
                case KalmanService.MSG_RANtoACT: //message from KalmanService
                    double[] posK = (double[]) msg.obj; //dimension 3 : x,y,z position on NED frame
                    double[] posKalman=ecef2lla(nedToEcef(posK)); // convert NED to ECEF to LLA
                    k1.setText(String.format("Lat Kalman: %1$.8f", posKalman[0])); //latitude estimated by Kalman filter
                    k2.setText(String.format("Long Kalman: %1$.8f", posKalman[1])); //longitude estimated by Kalman filter
                    k3.setText(String.format("Alt Kalman: %1$.8f", posKalman[2])); //altitude estimated by Kalman filter
                    break;
                default:
                    super.handleMessage(msg);
            }
        }
    };


    // booleans for connections
    boolean bound, bound2;

    /**
     * Menage the connection between Activity and Service
     */
    private ServiceConnection mConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            mService = new Messenger(service);
            bound = true;
        }

        public void onServiceDisconnected(ComponentName className) {
            mService = null;
            bound = false;
        }
    };
    private ServiceConnection mConnection2 = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            mService2 = new Messenger(service);
            bound2 = true;
        }

        public void onServiceDisconnected(ComponentName className) {
            mService2 = null;
            bound2 = false;
        }
    };

    //UI elements
    private TextView wx,wy,wz,ax,ay,az,g1,g2,g3,t,k1,k2,k3; //text with datas
    public static TextView sicuro,medio,allarme; //text over seekbar
    public static SeekBar s;
    //static elements for coordinate conversion
    public static double[] posizione0={data0[7],data0[8],data0[9]};
    public static double[] ecefRef=llaToECEF(posizione0);
    public static Matrice rotEcef=new Matrice(3,3);
    @Override
    protected void onCreate(Bundle savedInstanceState) throws SecurityException {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        bounce= AnimationUtils.loadAnimation(this,R.anim.bounce);
        wx = findViewById(R.id.textView);
        wy = findViewById(R.id.textView2);
        wz = findViewById(R.id.textView3);
        ax = findViewById(R.id.textView5);
        ay = findViewById(R.id.textView6);
        az = findViewById(R.id.textView7);
        g1 = findViewById(R.id.textView8);
        g2 = findViewById(R.id.textView9);
        g3=findViewById(R.id.textView10);
        t = findViewById(R.id.textView4);
        k1=findViewById(R.id.textView11);
        k2=findViewById(R.id.textView12);
        k3=findViewById(R.id.textView16);
        s=findViewById(R.id.seekBar);
        sicuro=findViewById(R.id.textView22);
        medio=findViewById(R.id.textView21);
        allarme=findViewById(R.id.textView23);
        wx.setText("Wx:");
        wy.setText("Wy:");
        wz.setText("Wz:");
        ax.setText("Ax:");
        ay.setText("Ay:");
        az.setText("Az:");
        g1.setText("lat:");
        g2.setText("long");
        g3.setText("Alt:");
        k1.setText("Lat Kalman:");
        k2.setText("Long Kalman:");
        k3.setText("Alt Kalman:");
        s.setProgress(0);

        // Acquire a reference to the system Location Manager
        locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

        // Define a listener that responds to location updates
         locationListener = new LocationListener() {
            public void onLocationChanged(Location location) {
                //implements
                //makeUseOfNewLocation(location);
            }

            public void onStatusChanged(String provider, int status, Bundle extras) {
                Log.d(TAG1,"stato provider cambiato");
            }

            public void onProviderEnabled(String provider) {
                Log.d(TAG1,"provider non disponibile");
            }

            public void onProviderDisabled(String provider) {
                Log.d(TAG1,"provider disattivato");
            }
        };
        final String locationProvider = LocationManager.GPS_PROVIDER;

        locationManager.requestLocationUpdates(locationProvider, 0, 0, locationListener);

        // acquire a reference to SensorManager
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        sensorG = sensorManager.getDefaultSensor(TYPE_GYROSCOPE);
        sensorA = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        // event listener
        eventListener = new SensorEventListener()  {
            @Override
            public void onSensorChanged(SensorEvent event) throws SecurityException {
                int tipo = event.sensor.getType();
                data[0] = event.timestamp;
                Location lastKnownLocation = locationManager.getLastKnownLocation(locationProvider); //get location sync with sensor datas
                if(lastKnownLocation!=null)
                {
                    data[7]=(float)lastKnownLocation.getLatitude();
                    data[8]=(float)lastKnownLocation.getLongitude();
                    data[9]=(float)lastKnownLocation.getAltitude();

                    //if reference position from StartActivity wasn't set, set now
                    if((posizione0[0]==0)&&(posizione0[1]==0)&&(posizione0[2]==0)) {
                        posizione0[0] = lastKnownLocation.getLatitude();
                        posizione0[1] = lastKnownLocation.getLongitude();
                        posizione0[2] = lastKnownLocation.getAltitude();
                        ecefRef = llaToECEF(posizione0);
                        rotEcef = getRotEcef(posizione0);
                    }

                }

                switch (tipo) {
                    // Event came from the light sensor.
                    case TYPE_LINEAR_ACCELERATION:
                        data[4] = event.values[0]; // X
                        data[5] = event.values[1]; // Y
                        data[6] = event.values[2]; // Z
                        break;
                    case TYPE_GYROSCOPE:
                        data[1] = event.values[0]; // X
                        data[2] = event.values[1]; // Y
                        data[3] = event.values[2]; // Z
                        break;
                    default:
                        // do nothing

                }
                // Send values to SyncService
                if (!bound) return;
                Log.i(TAG1, "sendins Asynch data");
                Message msg = Message.obtain(null, SyncService.MSG_ACTtoSER, 0, 0, data);
                try {
                    mService.send(msg);
                } catch (RemoteException e) {
                    e.printStackTrace();
                }


            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
                Log.i("MainActivity","Precisione del sensore cambiata");
            }
        };
        //regist sensors to eventListener
        sensorManager.registerListener(eventListener, sensorG, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(eventListener, sensorA, SensorManager.SENSOR_DELAY_FASTEST);

        // start services
        i = new Intent(this,SyncService.class);
        startService(i);

        j = new Intent(this, KalmanService.class);
        startService(j);

    }

    /**
     * when app starts connect Activity and Service:
     * send reference of Messengers
     */
    @Override
    protected void onStart() {
        super.onStart();
        //Context ctx = getApplicationContext();

        i = new Intent(this,SyncService.class);
        inMessenger = new Messenger(incomingMessages);
        i.putExtra("messenger",inMessenger);

        bindService(i, mConnection,
                Context.BIND_AUTO_CREATE);

        j = new Intent(this, KalmanService.class);
        j.putExtra("messenger",inMessenger);

        bindService(j, mConnection2,
                Context.BIND_AUTO_CREATE);
    }

    /**
     * app close, delete the connection
     */
    @Override
    protected void onStop() {
        super.onStop();
        // Unbind from the service
        if (bound) {
            unbindService(mConnection);
            bound = false;
        }
        if (bound2) {
            unbindService(mConnection2);
            bound2 = false;
        }
    }


    /**
     * open app, collect datas
     */
    @Override
    protected void onResume(){
        super.onResume();
        sensorManager.registerListener(eventListener, sensorG, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(eventListener, sensorA, SensorManager.SENSOR_DELAY_FASTEST);
    }

    /**
     * app in background, stop reading datas
     */
    @Override
    protected void onPause(){
        super.onPause();
        sensorManager.unregisterListener(eventListener);
    }


    /**
     * 3d vector sum
     * @param a vector 1
     * @param b vector 2
     * @return vector [a1+b1, a2+b2, a3+b3]
     */
    public static double[] sum(double[] a, double[] b)
    {
        double[] c=new double[3];
        c[0]=a[0]+b[0];
        c[1]=a[1]+b[1];
        c[2]=a[2]+b[2];
        return c;
    }
    /**
     * 3d vector sub
     * @param a vector 1
     * @param b vector 2
     * @return vector [a1-b1, a2-b2, a3-b3]
     */
    public static double[] sub(double[] a, double[] b)
    {
        double[] c=new double[3];
        c[0]=a[0]-b[0];
        c[1]=a[1]-b[1];
        c[2]=a[2]-b[2];
        return c;
    }


    /**
     * conversion from ECEF frame to NED frame
     * @param ecef 3d coordinates in ECEF frame
     * @return 3d coordinates in NED frame
     */
    public static double[] ecefToNED(double[] ecef)
    {
        rotEcef=getRotEcef(posizione0);
        double[] pNed=rotEcef.mvp(sub(ecef,ecefRef));
        return pNed;
    }

    /**
     * Method for coordinate conversion
     * @param lla GPS position corrisponding to ECEF reference position for ECEF-NED conversion
     * @return rotation matrix for ECEF-NED conversion
     */
    public static Matrice getRotEcef(double[] lla)
    {
        Matrice rot=new Matrice(3,3);
        rot.mat[0][0]=-Math.sin(lla[0])*Math.cos(lla[1]);
        rot.mat[0][1]=-Math.sin(lla[1]);
        rot.mat[0][2]=-Math.cos(lla[0])*Math.cos(lla[1]);
        rot.mat[1][0]=-Math.sin(lla[0])*Math.sin(lla[1]);
        rot.mat[1][1]=Math.cos(lla[1]);
        rot.mat[1][2]=-Math.cos(lla[0])*Math.sin(lla[1]);
        rot.mat[2][0]=Math.cos(lla[0]);
        rot.mat[2][1]=0;
        rot.mat[2][2]=-Math.sin(lla[0]);
        return rot;
    }

    /**
     * conversion from NED frame to ECEF frame
     * @param ned 3d coordinates in NED frame
     * @return 3d coordinates in ECEF frame
     */
    public static double[] nedToEcef(double[] ned)
    {
        //Log.e("CONVERSIONE",String.valueOf(ned[0])+","+String.valueOf(ned[1])+","+String.valueOf(ned[2]));
        rotEcef=getRotEcef(posizione0);
        Matrice rot=rotEcef.inversa();
        //rot.logMatrice();
        double[] appo=rot.mvp(ned);
        double[] ecef=sum(ecefRef,appo);
        return ecef;
    }

    /**
     * conversion from ECEF frame to lla frame (gps, WGS84)
     * @param ecef 3d coordinates in ECEF frame
     * @return latitude,longitude,altitude
     */

   public static double[] ecef2lla(double[] ecef){
    double[] lla=new double[3];
    double A_EARTH = 6378137.0;
    double flattening = 1.0 / 298.257223563;
    double NAV_E2 = (2.0 - flattening) * flattening; 
    double rad2deg = 180.0 / Math.PI;

    if ((ecef[0] == 0.0)&&(ecef[1]==0.0)){
        lla[1]=0.0;
    } else {
        lla[1] =Math.atan2(ecef[1],ecef[0]) * rad2deg;
    }
    
    // Make initial lat and alt guesses based on spherical earth.
    double rhosqrd = ecef[0] * ecef[0] + ecef[1] * ecef[1];
    double rho = Math.sqrt(rhosqrd);
    double templat = Math.atan2(ecef[2], rho);
    double tempalt = Math.sqrt(rhosqrd + ecef[2] * ecef[2]) - A_EARTH;
    double rhoerror = 1000.0;
    double zerror = 1000.0;
    int iter = 0; // number of iterations

        //      %  Newton's method iteration on templat and tempalt makes
        //      %   the residuals on rho and z progressively smaller.  Loop
        //      %   is implemented as a 'while' instead of a 'do' to simplify
        //      %   porting to MATLAB

        while ((Math.abs(rhoerror) > 1e-6)||(Math.abs(zerror) > 1e-6)){
            double slat = Math.sin(templat);
            double clat = Math.cos(templat);
            double qu = 1.0 - NAV_E2 * slat*slat;
            double r_n = A_EARTH /Math.sqrt(qu);
            double drdl = r_n * NAV_E2 * slat * clat / qu; // d(r_n)/d(latitutde)
            rhoerror = (r_n + tempalt) * clat - rho;
            zerror = (r_n * (1.0 - NAV_E2) + tempalt) * slat - ecef[2];
            double aa = drdl * clat - (r_n + tempalt) * slat;
            double bb = clat;
            double cc = (1.0 - NAV_E2)*(drdl * slat + r_n * clat);
            double dd = slat;
            double invdet = 1.0 / (aa * dd - bb * cc);
            templat = templat - invdet * (+dd * rhoerror - bb * zerror);
            tempalt = tempalt - invdet * (-cc * rhoerror + aa * zerror);
            iter++;
        }
        lla[0] = templat*rad2deg;
        lla[2] = tempalt;
    return lla;
}

    /**
     * conversion from LLA frame to ECEF frame
     * @param lla latitude, longitude, altitude
     * @return 3d coordinates in ECEF frame
     */
    public static double[] llaToECEF(double[] lla){
    double[] ecef=new double[3];
	double A_EARTH = 6378137.0;
	double flattening = 1.0/298.257223563;
	double NAV_E2 = (2.0-flattening)*flattening; // also e^2
	double deg2rad = Math.PI/180.0;

	double slat = Math.sin(lla[0]*deg2rad);
	double clat = Math.cos(lla[0]*deg2rad);
	double r_n = A_EARTH/Math.sqrt(1.0 - NAV_E2*slat*slat);
	ecef[0] = (r_n + lla[2])*clat*Math.cos(lla[1]*deg2rad);  
	ecef[1] = (r_n + lla[2])*clat*Math.sin(lla[1]*deg2rad);  
	ecef[2] = (r_n*(1.0 - NAV_E2) + lla[2])*slat;

  	return ecef;
}
}


