/**
 * KalmanFusion by Amedeo Pachera
 * Last version 11/07/2019
 * info: amedeopachera@gmail.com
 *
 * State.java : java class to manage Kalman filter's state
 */
package com.bignerd.android.sensor;

public class State {
    //IN NED FRAME
    public double[] position;
    public double[] velocity;
    public Quaternion quaternion;
    public Matrice covariance;

    public State(double[] p, double[] v, Quaternion q,Matrice cov)
    {
        position =p;
        velocity=v;
        quaternion=q;
        covariance = new Matrice(13,13);
        for(int i=0;i<13;i++)
            for(int j=0;j<13;j++)
                covariance.mat[i][j]=cov.mat[i][j];
    }
    public State(State s) //copy costructor
    {
        this.position=new double[3];
        this.velocity=new double[3];
        this.quaternion=new Quaternion(s.quaternion.x0,s.quaternion.x1,s.quaternion.x2,s.quaternion.x3);
        this.covariance=new Matrice(13,13);
        for(int i=0;i<3;i++)
        {
            this.position[i]=s.position[i];
            this.velocity[i]=s.velocity[i];
        }

        for(int i=0;i<13;i++)
            for(int j=0;j<13;j++)
                this.covariance.mat[i][j]=s.covariance.mat[i][j];

    }

}
