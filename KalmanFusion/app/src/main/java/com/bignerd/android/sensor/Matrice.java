/**
 * KalmanFusion by Amedeo Pachera
 * Last version 11/07/2019
 * info: amedeopachera@gmail.com
 *
 * Matrice.java : java class to manage Matrix operations
 */
package com.bignerd.android.sensor;

import android.util.Log;

public class Matrice {

    public double[][] mat;
    public int righe; //row
    public int colonne; //column

    public Matrice(int r, int c)
    {
        mat = new double[r][c];
        righe=r;
        colonne=c;
    }

    //matix * matrix
    public Matrice mmp(Matrice b)
    {
        Matrice result= new Matrice(this.righe,b.colonne);
        double appo=0;
        for(int i=0;i<this.righe;i++) //righe matrice A
        {
            for(int j=0;j<b.colonne;j++) //colonne mat B
            {
                for(int k=0;k<this.colonne;k++)
                {
                    appo=appo+this.mat[i][k]*b.mat[k][j];
                }
                result.mat[i][j]=appo;
                appo=0;
            }
        }

        return result;
    }

    // matrix * vector
    public double[] mvp(double[] a)
    {
        double[] result=new double[this.righe];
        double appo=0;
        for(int i=0;i<this.righe;i++)
        {
            for(int j=0;j<this.colonne;j++)
            {
                appo=appo+this.mat[i][j]*a[j];
            }
            result[i]=appo;
            appo=0;
        }
        return result;
    }

    //vector * matrix
    public double[] vmp(double[] a)
    {
        double[] result=new double[this.colonne];
        double appo=0;
        for(int i=0;i<this.colonne;i++)
        {
            for(int j=0;j<this.righe;j++)
            {
                appo=appo+a[j]*this.mat[j][i];
            }
            result[i]=appo;
            appo=0;
        }
        return result;
    }

    //matrix trasposition
    public Matrice trasposta()
    {
        Matrice t=new Matrice(this.colonne,this.righe);
        {
            for(int i=0;i<this.righe;i++)
                for(int j=0;j<this.colonne;j++)
                    t.mat[j][i]=this.mat[i][j];
        }
        return t;
    }

    //log matrix diagonal
    public void logMatrice()
    {
        Log.e("MATRICE:",String.valueOf(this.mat[0][0])+","+String.valueOf(this.mat[1][1])+","+String.valueOf(this.mat[2][2])+","+String.valueOf(this.mat[3][3])+","+String.valueOf(this.mat[4][4])+","+String.valueOf(this.mat[5][5])+","+String.valueOf(this.mat[6][6])+","+String.valueOf(this.mat[7][7])+","+String.valueOf(this.mat[8][8])+","+String.valueOf(this.mat[9][9])+","+String.valueOf(this.mat[10][10])+","+String.valueOf(this.mat[11][11])+","+String.valueOf(this.mat[12][12]));
    }

    // Matrix A - Matrix B
    public Matrice sub(Matrice a)
    {
        Matrice result=new Matrice(a.righe,a.colonne);
        for(int i=0;i<this.righe;i++)
        {
            for(int j=0;j<this.colonne;j++)
            {
                result.mat[i][j]=this.mat[i][j]-a.mat[i][j];
            }
        }
        return result;
    }

    // Matrix A + Matrix B
    public Matrice sum(Matrice a)
    {
        Matrice result=new Matrice(a.righe,a.colonne);
        for(int i=0;i<this.righe;i++)
        {
            for(int j=0;j<this.colonne;j++)
            {
                result.mat[i][j]=this.mat[i][j]+a.mat[i][j];
            }
        }
        return result;
    }

    // Matrix A * scalar
    public Matrice scalar(double a)
    {
        Matrice result=new Matrice(this.righe,this.colonne);
        for(int i=0;i<this.righe;i++)
            for(int j=0;j<this.colonne;j++)
                result.mat[i][j]=this.mat[i][j]*a;
        return result;
    }

    //matrix determinant
    public double determinante()
    {

        double a=this.mat[1][1]*this.mat[2][2]-this.mat[1][2]*this.mat[2][1];
        double b=this.mat[1][0]*this.mat[2][2]-this.mat[1][2]*this.mat[2][0];
        double c=this.mat[1][0]*this.mat[2][1]-this.mat[1][1]*this.mat[2][0];
        double det=this.mat[0][0]*(a)-this.mat[0][1]*(b)+this.mat[0][2]*(c);
        return det;
    }


    //matrix inversion (3x3)
    public Matrice inversa()
    {
        double det=this.determinante();
        Matrice result=new Matrice(3,3);
        result.mat[0][0]=(this.mat[1][1]*this.mat[2][2]-this.mat[1][2]*this.mat[2][1]);
        result.mat[0][1]=(-this.mat[1][0]*this.mat[2][2]+this.mat[1][2]*this.mat[2][0]);
        result.mat[0][2]=(this.mat[1][0]*this.mat[2][1]-this.mat[1][1]*this.mat[2][0]);
        result.mat[1][0]=(-this.mat[0][1]*this.mat[2][2]+this.mat[0][2]*this.mat[2][1]);
        result.mat[1][1]=(this.mat[0][0]*this.mat[2][2]-this.mat[0][2]*this.mat[2][0]);
        result.mat[1][2]=(-this.mat[0][0]*this.mat[2][1]+this.mat[0][1]*this.mat[2][0]);
        result.mat[2][0]=(this.mat[0][1]*this.mat[1][2]-this.mat[0][2]*this.mat[1][1]);
        result.mat[2][1]=(-this.mat[0][0]*this.mat[1][2]+this.mat[0][2]*this.mat[1][0]);
        result.mat[2][2]=(this.mat[0][0]*this.mat[1][1]-this.mat[0][1]*this.mat[1][0]);
        return (result.trasposta()).scalar(1/det);
    }



}
