#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"
#include <Eigen/Dense>

class LSM9DS1printCallback : public LSM9DS1callback {

   
    const float om_ie_z = 7.2921150e-5; //the sideral angular rate of earth
    const float Lb = 10; //longitude of the begining point;
    const float lamdab =10; //lattitude of the begining point;

    Eigen::Matrix3d I {{1,0,0},{0,1,0},{0,0,1}}; // I: Unit Matrix.
    
    Eigen::Vector3d om_ie_e {{0,0,om_ie_z}}; 
    
    Eigen::Matrix3d Om_ie_e {{0,-om_ie_z,0},{om_ie_z,0,0},{0,0,0}};
    
    /*
    **initial value of velocity and position
    */

    Eigen::Vector3d vd_eb_e {{0,0,0}};
    Eigen::Vector3d vd_eb_e_last {{0,0,0}};
    Eigen::Vector3d v_eb_e {{0,0,0}};
    Eigen::Vector3d v_eb_e_last {{0,0,0}};
    //Eigen::Matrix3d Cbe {{cos(psi0), sin(psi0), 0},{-sin(psi0), cos(psi0), 0},{0, 0, 1}};
    Eigen::Vector3d r_eb_e {{0,0,0}};
    Eigen::Vector3d om_ib_b;
    Eigen::Vector3d f_ib_b;
    Eigen::Vector3d f_ib_e;
    
    Eigen::Matrix3d Cen {
                            {-sin(Lb)*cos(lamdab), -sin(Lb)*sin(lamdab), cos(Lb)},
                            {-sin(lamdab), cos(lamdab), 0},
                            {-cos(Lb)*cos(lamdab), -cos(Lb)*sin(lamdab), -sin(Lb)}
    };

    Eigen::Matrix3d Cbe = Cen.transpose();

    virtual void hasSample(LSM9DS1Sample s) {

        /*The code below try to transform data from LSM9DS1, such as angular rate (Gyroscope),

         specific force (Accelerometer) to attitude, acceleration respectively.

         Furthermore using attitude and acceleration to computing the velocity and displacement of a object.

         \para Cbe : coordinate transform matric from body to earth axis applied to the specific force measurement.
         */

        om_ib_b << s.gx,s.gy,s.gz; //data from Gyroscope;
        f_ib_b << s.ax, s.ay, s.az; //data from Accelerometer

        //transform specific force frame;
        f_ib_e = Cbe*f_ib_b; //specific force resolved in ECEF.
        vd_eb_e_last = vd_eb_e; //lastaccleration
        vd_eb_e = f_ib_e - 2*Om_ie_e*v_eb_e; //Accleration of system;

        

        Eigen::Vector3d alph = s.dt*(om_ib_b-om_ie_e); //vector of incremental angles;
        double alph_norm = alph.norm(); //normalised value of incremental angles.
        Eigen::Matrix3d alph_skew {{0,-alph(2),alph(1)},
                                   {alph(2),0,-alph(0)},
                                   {-alph(1),alph(0),0}
        }; //skew matrix of alph;
        
        //update attitude;

        Eigen::Matrix3d Cdcm = I + alph_skew*sin(alph_norm)/(alph_norm) + alph_skew*alph_skew*(1-cos(alph_norm))/(alph_norm*alph_norm);
        Cbe = Cbe*Cdcm;

        //update velocity;
        v_eb_e_last = v_eb_e;
        v_eb_e = v_eb_e + s.dt*(vd_eb_e + vd_eb_e_last)/2;

        //update position;
        r_eb_e = r_eb_e + s.dt*(v_eb_e + v_eb_e_last)/2;

        imudisplay();

    }

    /*method that provide the velocity;*/
    Eigen::Vector3d getvelocity(){

        return v_eb_e;

    }
    /*method that provide position;*/
    Eigen::Vector3d getposition(){
        
        return r_eb_e;

    }

    void imudisplay(){

   

        fprintf(stderr,"Velocity:\t%3.10f,\t%3.10f,\t%3.10f [m/s]\n", v_eb_e[0], v_eb_e[1], v_eb_e[2]);

        fprintf(stderr,"Position:\t%3.10f,\t%3.10f,\t%3.10f [m]\n", r_eb_e[0],r_eb_e[1],r_eb_e[2]);

        //fprintf(stderr,"Mag:\t%3.10f,\t%3.10f,\t%3.10f [gauss]\n", s.mx, s.my, s.mz);

        //fprintf(stderr,"Temperature: %f\n",s.temperature);

        fprintf(stderr,"\n");

   

    }

};


int main(int argc, char *argv[]) {
    fprintf(stderr,"Press <RETURN> any time to stop the acquisition.\n");
    LSM9DS1 imu;
    LSM9DS1printCallback callback;
    imu.setCallback(&callback);
    GyroSettings gyroSettings;
    gyroSettings.sampleRate = GyroSettings::G_ODR_14_9; // 14.9Hz for acc and gyr
    imu.begin(gyroSettings);
    do {
	sleep(1);
    } while (getchar() < 10);
    imu.end();
    exit(EXIT_SUCCESS);
}
