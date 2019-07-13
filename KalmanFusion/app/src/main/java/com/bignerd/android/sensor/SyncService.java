/**
 * KalmanFusion by Amedeo Pachera
 * Last version 11/07/2019
 * info: amedeopachera@gmail.com
 *
 * SyncService.java : this service receive asynch data from MainActivity and return Synch data every INTERVAL milliseconds
 */
package com.bignerd.android.sensor;


import android.app.IntentService;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;
import java.util.LinkedList;

public class SyncService extends IntentService {

    /**
     * Datas
     */
    public static LinkedList<float[]> database = new LinkedList<>();
    float[] data;

    /**
     * Variables for control
     */
    private final long INTERVAL = 1000;
    private long lastTime = 0;
    private long actualTime = 0;

    /**
     * Variable for messages control
     */
    static final int MSG_ACTtoSER = 1; // Messagge from Activity to Service
    static final int MSG_SERtoACT = 2; // Messagge from Service to Service
    private Messenger mMessenger; // Messenger in
    private Messenger outMessanger; // Messenger out
    /**
     * Handler of incoming messagges
     */
    class IncomingHandler extends Handler {
        private Context applicationContext;

        IncomingHandler(Context context) {
            applicationContext = context.getApplicationContext();
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case MSG_ACTtoSER:
                    Log.i("SyncService","Nuovi dati asincroni ricevuti");
                    float[] atm = (float[]) msg.obj;
                    float[] tem = atm.clone();
                    database.addLast(tem); //add datas to a list
                    break;
                default:
                    super.handleMessage(msg);
            }
        }
    }

    /**
     * Metodo costruttore
     */
    public SyncService(){
        super("SyncService");
    }

    /**
     * Ciclo di vita del servizio, ad ogni intervallo di tempo prende i dati salvati
     * e li restituisce alla Activity tramite messaggio
     * @param i intent tra le Activity e Service
     */
    @Override
    protected void onHandleIntent(Intent i) {
        while(true)
        {
            actualTime = System.currentTimeMillis();
            if ((actualTime - lastTime)>= INTERVAL) { //every INTERVAL milliseconds sends synch data to MainAcvivity
                if(outMessanger != null) {
                    try {
                        data = getDataSync();
                        outMessanger.send(Message.obtain(null, MSG_SERtoACT, 0, 0,data));
                    } catch (RemoteException e) {
                        e.printStackTrace();
                    }
                }


                lastTime = actualTime;
            }
        }
    }

    /**
     * Return the last data in list
     * @return last measurements
     */
    private float[] getDataSync(){
        if (database.size() == 0) return null;
        float[] syncdata = database.getLast();
        database.clear();

        return syncdata;
    }

    /**
     * when bind return the interfce for communication
     * @param i intent between Activity and Service
     */
    @Override
    public IBinder onBind(Intent i) {
        Log.e("SyncService","Binding...");
        mMessenger = new Messenger(new IncomingHandler(this));
        outMessanger = (Messenger) i.getExtras().get("messenger");
        return mMessenger.getBinder();
    }

    /**
     * Alla distruzione del servizio informa l'utente
     */
    @Override
    public void onDestroy()
    {
        Log.i("SyncService", "Distruzione del servizio");
    }
}
