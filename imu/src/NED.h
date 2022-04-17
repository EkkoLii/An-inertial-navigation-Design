#ifndef __NED_H__
#define __NED_H__

#include <stdio.h>
#include <stdlib.h>
//#include <stdint.h>
#include "LSM9DS1.h"
#include "Eigen/Dense"

class NED : public LSM9DS1callback 
{
public:
    double wie = 7.2921150e-5;
    double e = 0.0818191908425;//
    double R0= 6378137;//
    double Logi; //Logitude of IMU
    double lati; //latitude
    double hb; //altitude

    double Logid_pri; //Logitude of IMU
    double latid_pri; //latitude
    double hbd_pri; //altitude
    double dt; //sampling interval

    /**
     * variables defined for coordinate transformation
     **/
    Eigen::Matrix3d Cbn;
    Eigen::Matrix3d Cpos_pri;
    Eigen::Matrix3d skew_alpha;
    Eigen::Matrix3d Om_ie_n;
    Eigen::Matrix3d Om_en_n;
    Eigen::Vector3d wib_b;
    Eigen::Vector3d wie_n;
    Eigen::Vector3d wen_n;
    Eigen::Vector3d fib_b;
    Eigen::Vector3d fib_n;
    Eigen::Vector3d alpha; //vector of incremental angles, alpha_bn_n = dt*wbn_n
    Eigen::Vector3d veb_n;
    Eigen::Vector3d veb_priori;
    Eigen::Vector3d vd;
    Eigen::Vector3d vd_priori;
    Eigen::Vector3d P_cur;
    Eigen::Vector3d P_cart;

    double norm_alpha;

    
    /**
     * The fuction is derived from LSM9DS1callback
     * it was used to obtain data from LSM9DS1
     **/
    virtual void hasSample(LSM9DS1Sample sample);
    void DCMUpdate();
    void ForceFrameTrans();
    void VelocityUpdate();
    void PositionUpdate();
    void PositionInCartesian();
    double RE(double Lb);
    double RN(double Lb);
    Eigen::Vector3d Getvelocity();
    /*method that provide position;*/
    Eigen::Vector3d GetpositionInCart();
};
#endif 
