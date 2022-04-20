#include "ECEF.h"


ECEF::ECEF(double Lb, double longi, double hb){
    Pos_0(0) = (RE(Lb)+hb)*cos(Lb*d2r)*cos(longi*d2r);//xeb_e;
    Pos_0(1) = (RE(Lb)+hb)*cos(Lb*d2r)*sin(longi)*d2r;//yeb_e;
    Pos_0(2) = ((1-e*e)*RE(Lb)+hb)*sin(longi*d2r);//zeb_e;
    r_eb_e = Pos_0;
    Cen << -sin(Lb*d2r)*cos(longi*d2r), -sin(Lb*d2r)*sin(longi*d2r), cos(Lb*d2r),
           -sin(longi*d2r), -cos(longi*d2r), 0,
           -cos(Lb*d2r)*cos(longi*d2r), -cos(Lb*d2r)*sin(longi*d2r), -sin(Lb*d2r);
    Cne = Cen.transpose();
    Cbe = Cne*Cbn;
    g_0 << 0,0,9.7803253359*(1 + 0.001931853*pow(sin(Lb*d2r),2))/sqrt(1-pow(e*sin(Lb*d2r),2));

}

void ECEF::hasSample(LSM9DS1Sample sample) {
    fib_b <<sample.ax, sample.ay, sample.az;
    wib_b <<sample.gx, sample.gy, sample.gz;
    fib_b *= 9.8;
    wib_b *= d2r;
    BiasRemove();
    //fprintf(stderr,"Gyro:\t%3.10f,\t%3.10f,\t%3.10f,Accel:\t%3.10f,\t%3.10f,\t%3.10f\n", wib_b(0), wib_b(1),wib_b(2),fib_b(0),fib_b(1),fib_b(2));
    dt = sample.dt;
    PositionUpdate();
    //fprintf(stderr,"Position: X = \t%3.10f, Y = \t%3.10f [m]\n",r_eb_e(0)-Pos_0(0),r_eb_e(1)-Pos_0(1));
}
void ECEF::DCMUpdate(){

    alpha = dt*(wie_e - Cbe*wib_b); //wbe_e = wie_e - Cbe*wib_b;
    norm_alpha = alpha.norm();
    skew_alpha << 0,-alpha(2),alpha(1),
                  alpha(2),0,-alpha(0),
                  -alpha(1),alpha(0),0; //skew matrix of alpha
    
    Cpos_pre = I + skew_alpha*sin(norm_alpha)/norm_alpha + skew_alpha*skew_alpha*(1-cos(norm_alpha))/pow(norm_alpha,2);
    Cbe = Cbe*Cpos_pre;
    
}

void ECEF::VelocityUpdate(){
    DCMUpdate();
    v_eb_e_ = v_eb_e;
    vd_eb_e_ = vd_eb_e;
    vd_eb_e = Cbe*fib_b + g_0- 2*Om_ie_e*v_eb_e;
    v_eb_e = v_eb_e + dt*(vd_eb_e + vd_eb_e_)/2;
   
}
void ECEF::PositionUpdate(){

    VelocityUpdate(); //update velocity for computing new NED.
    r_eb_e = r_eb_e + dt*(v_eb_e + v_eb_e_)/2;
   
}

double ECEF::RE(double Lb){
    double re = R0/sqrt(1-e*e*pow(Lb*d2r,2));
    return re;
}
double ECEF::RN(double Lb){
    double rn = (1-e*e)*R0/pow((1-e*e*pow(sin(Lb*d2r),2)),3/2);
    return rn;
}
Eigen::Vector3d ECEF::Getvelocity(){

        return v_eb_e;

}

void ECEF::PositionReady(){
    if (!callback) return;
    Pos = r_eb_e -Pos_0;
    callback->set_cur_pos(Pos[0],Pos[1]);
   


}
Eigen::Vector3d ECEF::GetPosition(){
    return r_eb_e -Pos_0;
}
void ECEF::BiasRemove(){
    //remove bias from gyro
    if(abs(wib_b(0))<bg(0)){
        
        wib_b(0)=0;
    }else{
        if(wib_b(0)>0){
            wib_b(0) -=bg(0);
        }else{
            wib_b(0) += bg(0);
        }
           
    }
   if(abs(wib_b(1))<bg(1)){
        
        wib_b(1)=0;
    }else{
        if(wib_b(1)>0){
            wib_b(1) -=bg(1);
        }else{
            wib_b(1) += bg(1);
        }
           
    }
    if(abs(wib_b(2))<bg(2)){
        
        wib_b(2)=0;
    }else{
        if(wib_b(2)>0){
            wib_b(2) -=bg(2);
        }else{
            wib_b(2) += bg(2);
        }
           
    }
    //remove bias from accel
    if(abs(fib_b(0))<ba(0)){
        
        fib_b(0)=0;
    }else{
        if(fib_b(0)>0){
            fib_b(0) -=ba(0);
        }else{
            fib_b(0) += ba(0);
        }
           
    }
   if(abs(fib_b(1))<ba(1)){
        
        fib_b(1)=0;
    }else{
        if(fib_b(1)>0){
            fib_b(1) -=ba(1);
        }else{
            fib_b(1) += ba(1);
        }
           
    }
    if(abs(fib_b(2))<ba(2)){
        
        fib_b(2)=0;
    }else{
        if(fib_b(2)>0){
            fib_b(2) -=ba(2);
        }else{
            fib_b(2) += ba(2);
        }
           
    }

}
