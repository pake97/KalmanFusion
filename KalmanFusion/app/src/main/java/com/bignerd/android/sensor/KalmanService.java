package com.bignerd.android.sensor;

/**
 * KalmanFusion by Amedeo Pachera
 * Last version 11/07/2019
 * info: amedeopachera@gmail.com
 *
 * KalmanService.java: this service applies Kalman filter on datas received from MainActivity and return the estimate position
 */


import android.app.IntentService;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.TextView;

import static com.bignerd.android.sensor.MainActivity.bounce;
import static com.bignerd.android.sensor.MainActivity.s;
import static com.bignerd.android.sensor.MainActivity.sicuro;
import static com.bignerd.android.sensor.MainActivity.medio;
import static com.bignerd.android.sensor.MainActivity.allarme;
import static com.bignerd.android.sensor.StartActivity.data0;

public class KalmanService extends IntentService {
    public static double[] pos={0,0,0}; //first position in NED frame
    public static double[] biasG={0,0,0}; //gyroscope bias
    public static double[] biasA={-0.00040629576841369475,0.001835710851103186,0.1207805039747979}; //accelerometer bias
    public static double[] vel={0,0,0}; //first velocity in NED frame
    public static Quaternion quat=new Quaternion(1,0,0,0); //first quaternion in NED frame
    public static Matrice cova=new Matrice(13,13); //or use identity(13); //first states covariance matrix
    public static State prev=new State(pos,vel,quat,cova); //first state
    public static State post; //other state used in filtering
    private float[] data; //vector with datas
    private boolean RO=false; //control
    public static double distance; //distance between kalman estimated position and GPS position
    //ref position for coordinates conversion
    public static double[] posizione0={data0[7],data0[8],data0[9]};
    public static double[] ecefRef=llaToECEF(posizione0);
    public static Matrice rotEcef=new Matrice(3,3);
    /**
     * Variabiles for messages controllo
     */
    static final int MSG_ACTtoRAN = 3; // Messagge from Activity to Service
    static final int MSG_RANtoACT = 4; // Messagge from Service to Activity
    private Messenger mMessenger; // Messenger in
    private Messenger outMessanger; // Messenger out

    /**
     * Handler of incoming messages
     */
    class IncomingHandler extends Handler {
        private Context applicationContext;
        IncomingHandler(Context context) {
            applicationContext = context.getApplicationContext();
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case MSG_ACTtoRAN:
                    data= (float[]) msg.obj;
                    //Log.e("SENSOR",String.valueOf(data[1])+String.valueOf(data[2])+String.valueOf(data[3])+String.valueOf(data[4])+String.valueOf(data[5])+String.valueOf(data[6])+String.valueOf(data[7])+String.valueOf(data[8])+String.valueOf(data[9]));
                    RO=true;
                    break;
                default:
                    super.handleMessage(msg);
            }
        }
    }
    //builder
    public KalmanService(){
        super("KalmanService");
    }

    @Override
    protected void onHandleIntent(Intent i) {
        while(true)
        {   //if datas arrived
            if(RO==true){
            post=kalmanFilter(prev,data); //apply kalman filter
            prev=new State(post); //update state
            post=null;
            //compute distance
            double[] appo={data[7],data[8],data[9]};
            double[] temp=ecefToNED(llaToECEF(appo));
            distance=Math.sqrt((prev.position[0]-temp[0])*(prev.position[0]-temp[0])+(prev.position[1]-temp[1])*(prev.position[1]-temp[1])+(prev.position[2]-temp[2])*(prev.position[2]-temp[2]));
            //log info
            Log.e("DISTANCE",String.valueOf(distance));
            Log.e("BIAS",String.valueOf(biasG[0])+","+String.valueOf(biasG[1])+","+String.valueOf(biasG[2]));
            Log.e("VELOCITY",String.valueOf(prev.velocity[0])+","+String.valueOf(prev.velocity[1])+","+String.valueOf(prev.velocity[2]));
            //seekbar and animation
            int seek=(int)((distance/0.40)*100);
            s.setProgress(seek);
            animate(seek);
            try {
                outMessanger.send(Message.obtain(null, MSG_RANtoACT,0,0,prev.position));
            } catch (RemoteException e) {
                e.printStackTrace();
            }
            RO=false;
            }
        }
    }


    /**
     * when activity bind with service it returns an intefrace for communication
     * @param i intent between Activity and Service
     */
    @Override
    public IBinder onBind(Intent i) {
        Log.e("KalmanService","Binding...");
        mMessenger = new Messenger(new KalmanService.IncomingHandler(this));
        outMessanger = (Messenger) i.getExtras().get("messenger");
        return mMessenger.getBinder();
    }

    /**
     * when destroy (app close) log info
     */
    @Override
    public void onDestroy()
    {
        Log.i("KalmanService", "Distruzione del servizio");
    }



    // d = standard deviation (ex dp is the standard deviation for position)
    public static double dp=0.01; //positon
    public static double dw=0.01; //angular velocity
    public static double da=0.057; //acceleration
    public static double db=0.005; //gyroscope bias
    public static Matrice Q=createQ(); //Matrix Q (covariance of measures)
    public static Matrice H=createH(); //Matrix H (jacobian of observation)
    public static Matrice R=createR(); //Matrix R (covariance of observation)
    public static State kalmanFilter(State s1, float[] sensorData) {
        double[] p,v;
        double t=1.0; //trigger time, if you change this you need to change also the INTERVAL variable in SyncService
        double[] w={sensorData[1],sensorData[2],sensorData[3]}; //angular velocity
        double[] a={sensorData[4],sensorData[5],sensorData[6]}; //acceleration
        double[] pos={sensorData[7],sensorData[8],sensorData[9]}; // x,y,z in NED frame
        double[] gravity={0.0,0.0,0.0}; //if don't have linear_acceleration sensor you need to use this variable with gravity sensor
        Quaternion q; //quaternion
        Matrice Rnb = quatToMatrix(s1.quaternion); //rotation matrix corresponding to the quaternion
        Matrice cov; //covariance Matrix
        //TIME UPDATE
        //state update
        p=sum(sum(s1.position,sc(s1.velocity,t)),sc(sum(Rnb.mvp(sub(a,biasA)),gravity),t*t*0.5)); // p1 = p0 + v0*t + 1/2 * t^2 * R * (a - bias)
        v=sum(s1.velocity,sc(sum(Rnb.mvp(sub(a,biasA)),gravity),t)); // v1 = v0 * t + t * R * (a - bias)
        if((w[0]==0)&&(w[1]==0)&&(w[2]==0)) // if not angular velocity don't update quaternion
        {q=new Quaternion(s1.quaternion.x0,s1.quaternion.x1,s1.quaternion.x2,s1.quaternion.x3);}
        else
        {q=s1.quaternion.times((exp((sc(sub(w,biasG),(t*0.5))))));}  // dot product of quaternion with exp map of angular velocity
        //covariance update
        Matrice F=createF(a,w,t,s1.quaternion); // Matrix F (jacobian of state update)
        Matrice G=createG(t,s1.quaternion,w); //Matrix G (control of covariance update)

        cov=((F.mmp(s1.covariance)).mmp(F.trasposta())).sum((G.mmp(Q)).mmp(G.trasposta()));

        //MEASUREMENT UPDATE

        Matrice S=((H.mmp(cov)).mmp(H.trasposta())).sum(R); // Matrix S (residual of covariance)
        Matrice E=createE(pos,p); //Matrix E (residual of mesurement)
        Matrice K=(cov.mmp(H.trasposta())).mmp(S.inversa()); // Matrix K (Kalman Gain)

        Matrice status=createStatus(p,v,q,biasG).sum(K.mmp(E));
        //Matrice covarianza=cov.sub((K.mmp(S)).mmp(K.trasposta()));
        Matrice covarianza=cov.sub((K.mmp(H)).mmp(cov));

        // update state
        double[] pf={status.mat[0][0],status.mat[1][0],status.mat[2][0]};
        double[] vf={status.mat[3][0],status.mat[4][0],status.mat[5][0]};
        //normalize quaternion
        double norm=Math.sqrt(status.mat[6][0]*status.mat[6][0] + status.mat[7][0]*status.mat[7][0] +status.mat[8][0]*status.mat[8][0]+ status.mat[9][0]*status.mat[9][0]);
        Quaternion qf=new Quaternion(status.mat[6][0]/norm,status.mat[7][0]/norm,status.mat[8][0]/norm,status.mat[9][0]/norm);
        // normalize quaternion's covariance
        Matrice J= createJ(qf,qf);
        Matrice covarianzaf = (J.mmp(covarianza)).mmp(J.trasposta());
        State s2=new State(pf,vf,qf,covarianzaf);
        //covarianza.logMatrice(); //log covariance Matrix
        //update bias
        double[] bias={status.mat[10][0],status.mat[11][0],status.mat[12][0]};
        biasG=null;
        biasG=bias.clone();
        return s2;

    }



    public static double[] sum(double[] a, double[] b)
    {
        double[] c=new double[3];
        c[0]=a[0]+b[0];
        c[1]=a[1]+b[1];
        c[2]=a[2]+b[2];
        return c;
    }
    public static double[] sub(double[] a, double[] b)
    {
        double[] c=new double[3];
        c[0]=a[0]-b[0];
        c[1]=a[1]-b[1];
        c[2]=a[2]-b[2];
        return c;
    }


    public static double[] sc(double[] a, double b)
    {
        double[] c=new double[3];
        c[0]=a[0]*b;
        c[1]=a[1]*b;
        c[2]=a[2]*b;
        return c;
    }
    //exponential map of quaternion
    public static Quaternion exp(double[] a)
    {
        double norm=Math.sqrt((a[0]*a[0]+a[1]*a[1]+a[2]*a[2]));

        double w=Math.cos(norm);
        if(norm!=0)
        {
            double x=a[0]*(Math.sin(norm)/norm);
            double y=a[1]*(Math.sin(norm)/norm);
            double z=a[2]*(Math.sin(norm)/norm);
            return new Quaternion(w,x,y,z);
        }
        else return new Quaternion(1,0,0,0);
    }
    // conversion of a quaternion into rotation matrix
    public static Matrice quatToMatrix(Quaternion a)
    {
        Matrice result=new Matrice(3,3);
        result.mat[0][0]=2*a.x0*a.x0+2*a.x1*a.x1-1;
        result.mat[0][1]=2*a.x1*a.x2-2*a.x0*a.x3;
        result.mat[0][2]=2*a.x1*a.x3+2*a.x0*a.x2;
        result.mat[1][0]=2*a.x1*a.x2+2*a.x0*a.x3;
        result.mat[1][1]=2*a.x0*a.x0+2*a.x2*a.x2-1;
        result.mat[1][2]=2*a.x2*a.x3-2*a.x0*a.x1;
        result.mat[2][0]=2*a.x1*a.x3-2*a.x0*a.x2;
        result.mat[2][1]=2*a.x2*a.x3+2*a.x0*a.x1;
        result.mat[2][2]=2*a.x0*a.x0+2*a.x3*a.x3-1;
        return result;
    }
    //create identity matrix
    public static Matrice identity(int a)
    {
        Matrice result= new Matrice(a,a);
        for(int i=0;i<a;i++)
            result.mat[i][i]=1;
        return result;
    }

    public static Matrice createQ()
    {
        Matrice appo=new Matrice(12,12);
        appo.mat[0][0]=da*da;
        appo.mat[1][1]=da*da;
        appo.mat[2][2]=da*da;
        appo.mat[3][3]=da*da;
        appo.mat[4][4]=da*da;
        appo.mat[5][5]=da*da;
        appo.mat[6][6]=dw*dw;
        appo.mat[7][7]=dw*dw;
        appo.mat[8][8]=dw*dw;
        appo.mat[9][9]=db*db;
        appo.mat[10][10]=db*db;
        appo.mat[11][11]=db*db;

        return appo;
    }
    public static Matrice createF(double[] a,double[] w,double t,Quaternion q)
    {
        Matrice appo=new Matrice(13,13);
        Quaternion ap=exp(sc(sub(w,biasG),(t/2)));
        Matrice f=rProduct(ap);
        Matrice rrr=derivate(q,sub(a,biasA));
        Matrice ew=createEwd(biasG);
        Matrice g=((lProduct(q)).mmp(ew)).scalar((-t/2));
        appo.mat[0][0]=1;
        appo.mat[0][3]=t;
        appo.mat[1][4]=t;
        appo.mat[2][5]=t;
        appo.mat[1][1]=1;
        appo.mat[2][2]=1;
        appo.mat[3][3]=1;
        appo.mat[4][4]=1;
        appo.mat[5][5]=1;
        appo.mat[10][10]=1;
        appo.mat[11][11]=1;
        appo.mat[12][12]=1;
        for(int i=0;i<3;i++)
            for(int j=6;j<10;j++)
            {appo.mat[i][j]=rrr.mat[i][j-6]*t*t/2;
                appo.mat[i+3][j]=rrr.mat[i][j-6]*t;
            }
        for(int i=6;i<10;i++)
            for(int j=6;j<10;j++)
            {appo.mat[i][j]=f.mat[i-6][j-6];}

        for(int i=6;i<10;i++)
            for(int j=10;j<13;j++)
            {
                appo.mat[i][j]=g.mat[i-6][j-10];
            }
        return appo;
    }
    public static Matrice createG(double t,Quaternion q,double[] w)
    {
        Matrice appo=new Matrice(13,12);
        Matrice ew=createEw();
        Matrice g=((lProduct(q)).mmp(ew)).scalar((-t/2));
        appo.mat[0][0]=1;
        appo.mat[1][1]=1;
        appo.mat[2][2]=1;
        appo.mat[3][3]=1;
        appo.mat[4][4]=1;
        appo.mat[5][5]=1;
        appo.mat[10][9]=1;
        appo.mat[11][10]=1;
        appo.mat[12][11]=1;
        for(int i=6;i<10;i++)
            for(int j=6;j<9;j++)
                appo.mat[i][j]=g.mat[i-6][j-6];
        return appo;
    }

    public static Matrice createEw()
    {
        Matrice appo=new Matrice(4,3);
        appo.mat[0][0]=0;
        appo.mat[0][1]=0;
        appo.mat[0][2]=0;
        appo.mat[1][0]=1;
        appo.mat[1][1]=0;
        appo.mat[1][2]=0;
        appo.mat[2][0]=0;
        appo.mat[2][1]=1;
        appo.mat[2][2]=0;
        appo.mat[3][0]=0;
        appo.mat[3][1]=0;
        appo.mat[3][2]=1;
        return appo;
    }
    public static Matrice createEwd(double[] w)
    {
        double norm=Math.sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);
        if(norm!=0){
        Matrice appo = new Matrice(4,3);
        appo.mat[0][0]=(-Math.sin(norm)/norm)*w[0];
        appo.mat[0][1]=(-Math.sin(norm)/norm)*w[1];
        appo.mat[0][2]=(-Math.sin(norm)/norm)*w[2];
        appo.mat[1][0]=((norm*norm-w[0]*w[0])/(norm*norm*norm))*Math.sin(norm)+(w[0]/(norm*norm))*Math.cos(norm)*w[0];
        appo.mat[1][1]=0;
        appo.mat[1][2]=0;
        appo.mat[2][0]=0;
        appo.mat[2][1]=((norm*norm-w[1]*w[1])/(norm*norm*norm))*Math.sin(norm)+(w[1]/(norm*norm))*Math.cos(norm)*w[1];
        appo.mat[2][2]=0;
        appo.mat[3][0]=0;
        appo.mat[3][1]=0;
        appo.mat[3][2]=((norm*norm-w[2]*w[2])/(norm*norm*norm))*Math.sin(norm)+(w[2]/(norm*norm))*Math.cos(norm)*w[2];
        return appo;}
        else return createEw();
    }
    // R product of quaternion
    public static Matrice rProduct(Quaternion a)
    {
        Matrice result=new Matrice(4,4);
        result.mat[0][0]=a.x0;
        result.mat[0][1]=-a.x1;
        result.mat[0][2]=-a.x2;
        result.mat[0][3]=-a.x3;
        result.mat[1][0]=a.x1;
        result.mat[1][1]=a.x0;
        result.mat[1][2]=a.x3;
        result.mat[1][3]=-a.x2;
        result.mat[2][0]=a.x2;
        result.mat[2][1]=-a.x3;
        result.mat[2][2]=a.x0;
        result.mat[2][3]=a.x1;
        result.mat[3][0]=a.x3;
        result.mat[3][1]=a.x2;
        result.mat[3][2]=-a.x1;
        result.mat[3][3]=a.x0;
        return result;
    }

/*
    public static Matrice derivate(Quaternion a,double[] ac)
    {
        Matrice result=new Matrice(3,3);
        Matrice a1=new Matrice(3,3);
        a1.mat[0][0]=2*a.x0*a.x0-1.0;
        a1.mat[0][1]=0;
        a1.mat[0][2]=0;
        a1.mat[1][0]=0;
        a1.mat[1][1]=2*a.x0*a.x0-1.0;
        a1.mat[1][2]=0;
        a1.mat[2][0]=0;
        a1.mat[2][1]=0;
        a1.mat[2][2]=2*a.x0*a.x0-1.0;

        Matrice a2=new Matrice(3,3);
        a2.mat[0][0]=0;
        a2.mat[0][1]=-2*a.x0*a.x3;
        a2.mat[0][2]=2*a.x0*a.x2;
        a2.mat[1][0]=2*a.x0*a.x3;
        a2.mat[1][1]=0;
        a2.mat[1][2]=-2*a.x0*a.x1;
        a2.mat[2][0]=-2*a.x0*a.x2;
        a2.mat[2][1]=2*a.x0*a.x1;
        a2.mat[2][2]=0;
        result=a1.sum(a2);

        Matrice a3=new Matrice(3,3);
        result.mat[0][0]=a.x1*a.x1;
        result.mat[0][1]=a.x1*a.x2;
        result.mat[0][2]=a.x1*a.x3;
        result.mat[1][0]=a.x2*a.x1;
        result.mat[1][1]=a.x2*a.x2;
        result.mat[1][2]=a.x2*a.x3;
        result.mat[2][0]=a.x3*a.x1;
        result.mat[2][1]=a.x3*a.x2;
        result.mat[2][2]=a.x3*a.x3;
        result= result.sum(a3.scalar(2f));
        return (result.trasposta()).scalar(-1);
    }*/

    // derivate of a quaternion
    public static Matrice derivate(Quaternion a,double[] ac)
    {
        Matrice result=new Matrice(3,4);
        result.mat[0][0]=4*a.x0*ac[0]-2*a.x3*ac[1]+2*a.x2*ac[2];
        result.mat[0][1]=4*a.x1*ac[0]+2*a.x2*ac[1]+2*a.x3*ac[2];
        result.mat[0][2]=2*a.x1*ac[1]+2*a.x0*ac[2];
        result.mat[0][3]=-2*a.x0*ac[1]+2*a.x1*ac[2];
        result.mat[1][0]=2*a.x3*ac[0]+4*a.x0*ac[1]+2*a.x1*ac[2];
        result.mat[1][1]=2*a.x2*ac[0]+2*a.x0*ac[2];
        result.mat[1][2]=2*a.x1*ac[0]+4*a.x2*ac[1]+2*a.x3*ac[2];
        result.mat[1][3]=2*a.x0*ac[0]+2*a.x2*ac[2];
        result.mat[2][0]=-2*a.x2*ac[0]+2*a.x1*ac[1]+4*a.x0*ac[2];
        result.mat[2][1]=4*a.x3*ac[0]+2*a.x0*ac[1];
        result.mat[2][2]=-2*a.x0*ac[0]+2*a.x3*ac[1];
        result.mat[2][3]=2*a.x1*ac[0]+2*a.x2*ac[1]+4*a.x3*ac[2];
        return result;
    }

    public static Matrice createH()
    {
        Matrice appo=new Matrice(3,13);
        appo.mat[0][0]=1;
        appo.mat[1][1]=1;
        appo.mat[2][2]=1;
        return appo;
    }

    public static Matrice lProduct(Quaternion a)
    {
        Matrice result=new Matrice(4,4);
        result.mat[0][0]=a.x0;
        result.mat[0][1]=-a.x1;
        result.mat[0][2]=-a.x2;
        result.mat[0][3]=-a.x3;
        result.mat[1][0]=a.x1;
        result.mat[1][1]=a.x0;
        result.mat[1][2]=-a.x3;
        result.mat[1][3]=a.x2;
        result.mat[2][0]=a.x2;
        result.mat[2][1]=a.x3;
        result.mat[2][2]=a.x0;
        result.mat[2][3]=-a.x1;
        result.mat[3][0]=a.x3;
        result.mat[3][1]=-a.x2;
        result.mat[3][2]=a.x1;
        result.mat[3][3]=a.x0;
        return result;
    }
    public static Matrice createR()
    {
        Matrice result=new Matrice(3,3);
        result.mat[0][0]=dp*dp;
        result.mat[1][1]=dp*dp;
        result.mat[2][2]=dp*dp;
        return result;
    }
    public static Matrice createE(double[] pt,double[] p)
    {
        Matrice appo=new Matrice(3,1);
        double[] a =sub(ecefToNED(llaToECEF(pt)),p);
        appo.mat[0][0]=a[0];
        appo.mat[1][0]=a[1];
        appo.mat[2][0]=a[2];
        return appo;
    }
    // vector write as a matrix for better moltiplication
    public static Matrice createStatus(double[] p,double[] v, Quaternion q,double[] g)
    {
        Matrice result=new Matrice(13,1);
        result.mat[0][0]=p[0];
        result.mat[1][0]=p[1];
        result.mat[2][0]=p[2];
        result.mat[3][0]=v[0];
        result.mat[4][0]=v[1];
        result.mat[5][0]=v[2];
        result.mat[6][0]=q.x0;
        result.mat[7][0]=q.x1;
        result.mat[8][0]=q.x2;
        result.mat[9][0]=q.x3;
        result.mat[10][0]=g[0];
        result.mat[11][0]=g[1];
        result.mat[12][0]=g[2];
        return result;
    }

    public static Matrice createJ(Quaternion a, Quaternion b)
    {
        Matrice result = new Matrice(13,13);
        double norm=a.norm();
        result.mat[0][0]=1;
        result.mat[1][1]=1;
        result.mat[2][2]=1;
        result.mat[3][3]=1;
        result.mat[4][4]=1;
        result.mat[5][5]=1;
        result.mat[10][10]=1;
        result.mat[11][11]=1;
        result.mat[12][12]=1;
        result.mat[6][6]=a.x0*b.x0/(norm*norm*norm);
        result.mat[6][7]=a.x0*b.x1/(norm*norm*norm);
        result.mat[6][8]=a.x0*b.x2/(norm*norm*norm);
        result.mat[6][9]=a.x0*b.x3/(norm*norm*norm);
        result.mat[7][6]=a.x1*b.x0/(norm*norm*norm);
        result.mat[7][7]=a.x1*b.x1/(norm*norm*norm);
        result.mat[7][8]=a.x1*b.x2/(norm*norm*norm);
        result.mat[7][9]=a.x1*b.x3/(norm*norm*norm);
        result.mat[8][6]=a.x2*b.x0/(norm*norm*norm);
        result.mat[8][7]=a.x2*b.x1/(norm*norm*norm);
        result.mat[8][8]=a.x2*b.x2/(norm*norm*norm);
        result.mat[8][9]=a.x2*b.x3/(norm*norm*norm);
        result.mat[9][6]=a.x3*b.x0/(norm*norm*norm);
        result.mat[9][7]=a.x3*b.x1/(norm*norm*norm);
        result.mat[9][8]=a.x3*b.x2/(norm*norm*norm);
        result.mat[9][9]=a.x3*b.x3/(norm*norm*norm);
        return result;

    }



    public static double[] ecefToNED(double[] ecef)
    {
        rotEcef=getRotEcef(posizione0);
        double[] pNed=rotEcef.mvp(sub(ecef,ecefRef));
        return pNed;
    }

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
        while ((Math.abs(rhoerror) > 1e-6)||(Math.abs(zerror) > 1e-6)){
            double slat = Math.sin(templat);
            double clat = Math.cos(templat);
            double qu = 1.0 - NAV_E2 * slat*slat;
            double r_n = A_EARTH /Math.sqrt(qu);
            double drdl = r_n * NAV_E2 * slat * clat / qu;
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
    public static void animate(int seek)
    {
        sicuro.clearAnimation();
        medio.clearAnimation();
        allarme.clearAnimation();
        if(seek<40)
            sicuro.startAnimation(bounce);
        if((seek>=40)&&(seek<90))
            medio.startAnimation(bounce);
        if(seek>=90)
            allarme.startAnimation(bounce);
    }
}
