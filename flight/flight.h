#include "MPU9150_9Axis_MotionApps41.h"

#include "mraa.hpp"
#include "Matrix/matrix/math.hpp"

#include <librealsense/rs.hpp>

#include <chrono>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>

#include <unistd.h>
#include <inttypes.h>
#include <stdint.h>
#include <signal.h>
#include <chrono>
#include <fstream>
#include "/usr/include/upm/pca9685.h"

#include <stdio.h>
#include <string>

#include <termios.h>
#include <ctype.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

#include<cstdlib>
#include<cstdio>
#include<ctime>
#include<time.h>

float pi=M_PI;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t dmpIntStatus;   // holds actual interrupt status byte from DMP

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = e$
uint16_t packetSize=0;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion refQ;        // reference quaternion container
Quaternion corrQ;       // corrected quaternion container
Quaternion Q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 ar;
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            body gravity vector

float ypr[3],ypr_r[3]; // [psi, theta, phi]  Euler angle containers
int16_t gyro[3];
int16_t a_x,a_y,a_z; // used for correcting accel for rotation (due to IMU offset)

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

const int n_pts = 1400; // number of points to map

Quaternion fromEuler(float ypr[]) {
  float cosX2 = cos(ypr[2] / 2.0f);
  float sinX2 = sin(ypr[2] / 2.0f);
  float cosY2 = cos(ypr[1] / 2.0f);
  float sinY2 = sin(ypr[1] / 2.0f);
  float cosZ2 = cos(ypr[0] / 2.0f);
  float sinZ2 = sin(ypr[0] / 2.0f);

  float qa[4];
  qa[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
  qa[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
  qa[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
  qa[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;

  Quaternion q = Quaternion(qa[0],qa[1],qa[2],qa[3]);
  q.normalize();

  return q;
}

void dmpDataReady() {
  mpuInterrupt = true;
}

using namespace std;
using namespace matrix;

int sgn(float val) {
  if (val>0) {
    return 1;
  } else if (val<0) {
    return -1;
  } else {
    return 0;
  }
}

// linear function with deadzone
float sigma(float e_w,float ep) {
  if (abs(e_w)<ep/2) {return 0;}
  else {return e_w-sgn(e_w)*ep/2;}
}

char getUserChar() {
  int i;
  ioctl(0, FIONREAD, &i);
  if (i <= 0) return 0;
  return tolower(getchar());
}


float ElpsdRad(const Vector3f &P_o,float a,float b,float c) {
  float theta,phi,x,y,z;

  theta = atan((P_o(2)/c)/sqrt(pow(P_o(0),2)/(a*a)+pow(P_o(1),2)/(b*b)));
  phi = atan2(P_o(1)/b,P_o(0)/a);

  x=a*cos(theta)*cos(phi);
  y=b*cos(theta)*sin(phi);
  z=c*sin(theta);

  return sqrt(x*x+y*y+z*z); //radius of ellipsoid along p_i
}

// is object within ellipsoid
bool in_Elpsd(const Vector3f &P_o,float a,float b,float c) {
  if (P_o(0)==0||P_o(1)==0||P_o(2)==0) return false;
  return (pow(P_o(0)/a,2)+pow(P_o(1)/b,2)+pow(P_o(2)/c,2) <= 1);
}

// set all pwm values to val and close pwm connection
void setAllPWM(upm::PCA9685 *pwm, int val) {
  usleep(5000);
  //pwm->ledOffTime(PCA9685_ALL_LED, val);
  pwm->ledOffTime(15, val);
  pwm->ledOffTime(14, val);
  pwm->ledOffTime(1, val);
  pwm->ledOffTime(0, val);
  usleep(50000);
  printf(" - PWM off\n");
  pwm->setModeSleep(true);
  pwm->setModeSleep(false);
}

void write_Data(ofstream &file, float time[],float data[],int n) {
  if (file.is_open()) {
    for(int count = 0; count < n; count ++) {
	file << time[count] << "\t" << data[count] << "\n" ;
    }
    file << "\n";
  }
  else cout << "Unable to open file";
}

void write_Data(ofstream &file, float dat1[], float dat2[], float dat3[],int n) {
  if (file.is_open()) {
    for(int count = 0; count < n; count ++) {
	file << dat1[count] << "\t" << dat2[count] << "\t" << dat3[count] << "\n";
    }
    file << "\n";
  }
  else cout << "Unable to open file";
}
void write_Data(ofstream &file, const Matrix<float, 3, n_pts> &pts,int i) {
  if (file.is_open()) {
    for(int count = 0; count < i; count ++) {
      file << pts(0,count) << "\t" << pts(1,count) << "\t" << pts(2,count) << "\n";
    }
    file << "\n";
  }
  else cout << "Unable to open file";
}

void write_Data(ofstream &file, float time[], float dat1[], float dat2[],float dat3[],float dat4[],int n) {
  if (file.is_open()) {
    for(int count = 0; count < n; count ++) {
      file << time[count] << "\t" << dat1[count] << "\t" << dat2[count] << "\t" << dat3[count] << "\t" << dat4[count] << "\n";
    }
    file << "\n";
  }
  else cout << "Unable to open file";
}

void write_Position(int n,float time[],float x[], float y[],float z[],float z_o[]) {
  ofstream xy ("xy.txt"); ofstream xyz ("xyz.txt"); ofstream zz ("z_e.txt");
  ofstream xx ("x_e.txt"); ofstream yy ("y_e.txt");
  write_Data(xy,y,x,n); write_Data(xyz,x,y,z,n); write_Data(zz,time,z,n);
  write_Data(xx,time,x,n); write_Data(yy,time,y,n);
  write_Data(zz,time,z_o,n);
  xy.close(); xyz.close(); zz.close(); xx.close(); yy.close();
}

void write_Map(int i, const Matrix<float, 3, n_pts> &pts) {
  ofstream map ("map.txt");
  write_Data(map,pts,i);
  map.close();
}

void write_Velocity(int n,float time[],float u[],float v[],float w[],float u_c[],float v_c[],float w_c[]) {
  ofstream uu ("u_e.txt"); ofstream vv ("v_e.txt"); ofstream ww ("w_e.txt");
  write_Data(uu,time,u,n); write_Data(vv,time,v,n); write_Data(ww,time,w,n);
  write_Data(uu,time,u_c,n); write_Data(vv,time,v_c,n); write_Data(ww,time,w_c,n);
  uu.close(); vv.close(); ww.close();
}

void write_Vel_Int(int n,float time[],float Ie_u[],float Ie_v[],float Ie_w[]) {
  ofstream uu ("Ie_u.txt"); ofstream vv ("Ie_v.txt"); ofstream ww ("Ie_w.txt");
  write_Data(uu,time,Ie_u,n); write_Data(vv,time,Ie_v,n); write_Data(ww,time,Ie_w,n);
  uu.close(); vv.close(); ww.close();
}

void write_Acceleration(int n, float time[],float ud[],float vd[],float wd[]) {
  ofstream uu ("ud.txt"); ofstream vv ("vd.txt"); ofstream ww ("wd.txt");
  write_Data(uu,time,ud,n); write_Data(vv,time,vd,n); write_Data(ww,time,wd,n);
  uu.close(); vv.close(); ww.close();
}

void write_Angles(int n,float time[],float phi[],float theta[],float psi[],float phi_d[],float theta_d[],float psi_c[]) {
  ofstream Phi ("phi.txt"); ofstream Theta ("theta.txt"); ofstream Psi ("psi.txt");
  write_Data(Phi,time,phi,n); write_Data(Theta,time,theta,n); write_Data(Psi,time,psi,n);
  write_Data(Phi,time,phi_d,n); write_Data(Theta,time,theta_d,n); write_Data(Psi,time,psi_c,n);
  Phi.close(); Theta.close(); Psi.close();
}

void write_Ang_Int(int n,float time[],float Ie_ph[],float Ie_th[],float Ie_ps[]) {
  ofstream uu ("Ie_ph.txt"); ofstream vv ("Ie_th.txt"); ofstream ww ("Ie_ps.txt");
  write_Data(uu,time,Ie_ph,n); write_Data(vv,time,Ie_th,n); write_Data(ww,time,Ie_ps,n);
  uu.close(); vv.close(); ww.close();
}

void write_Angle_Errors(int n,float time[],float e_phi[],float e_the[],float e_psi[]) {
  ofstream eph ("eph.txt"); ofstream eth ("eth.txt"); ofstream eps ("eps.txt");
  write_Data(eph,time,e_phi,n); write_Data(eth,time,e_the,n); write_Data(eps,time,e_psi,n);
  eph.close(); eth.close(); eps.close();
}

void write_Rates(int n,float time[],float p[],float q[],float r[],float p_d[],float q_d[],float r_d[]) {     
  ofstream pp ("p.txt"); ofstream qq ("q.txt"); ofstream rr ("r.txt");
  write_Data(pp,time,p,n); write_Data(qq,time,q,n); write_Data(rr,time,r,n);
  write_Data(pp,time,p_d,n); write_Data(qq,time,q_d,n); write_Data(rr,time,r_d,n);
  pp.close(); qq.close(); rr.close();
}


void write_Momentum(int n,float time[],float hx[],float hy[],float hz[],float hp[]) {
  ofstream Hb ("Hb.txt"); ofstream Hp ("Hp.txt");
  write_Data(Hb,time,hx,n); write_Data(Hb,time,hy,n); write_Data(Hb,time,hz,n);
  write_Data(Hp,time,hp,n);
  Hb.close(); Hp.close();
}

void write_Rate_Errors(int n,float time[],float e_p[],float e_q[],float e_r[]) {
  ofstream ep ("ep.txt"); ofstream eq ("eq.txt"); ofstream er ("er.txt");
  write_Data(ep,time,e_p,n); write_Data(eq,time,e_q,n); write_Data(er,time,e_r,n);
  ep.close(); eq.close(); er.close();
}

void write_Inputs(int n,float time[],float T[],float t1[],float t2[],float
t3[],float om1[],float om2[],float om3[],float om4[]) {
  ofstream TT ("T.txt"); ofstream om ("om.txt");
  ofstream tt1 ("t1.txt"); ofstream tt2 ("t2.txt"); ofstream tt3 ("t3.txt");
  write_Data(TT,time,T,n); write_Data(om,time,om1,om2,om3,om4,n);
  write_Data(tt1,time,t1,n); write_Data(tt2,time,t2,n); write_Data(tt3,time,t3,n);
  TT.close(); om.close();
  tt1.close(); tt2.close(); tt3.close();
}

void write_Params(int n, float time[], float kp[],float bp[]) {
  ofstream kk ("k.txt"); ofstream bb ("b.txt");
  write_Data(kk,time,kp,n); write_Data(bb,time,bp,n);
  kk.close(); bb.close();
}

void write_Freq(int n, float time[],float clf[]) {
  ofstream freq ("freq.txt");
  write_Data(freq,time,clf,n);
  freq.close();
}

void write_Pos_Obj(int n,float time[],float px[], float py[],float pz[],float dist[]) {
  ofstream posx ("posx.txt"); ofstream posy ("posy.txt"); ofstream posz ("posz.txt");
  ofstream pts ("pts.txt");
  write_Data(posx,time,px,n); write_Data(posy,time,py,n); write_Data(posz,time,pz,n);
  write_Data(pts,time,dist,n);
  posx.close(); posy.close(); posz.close(); pts.close();
}

// Initialize all the data holders
uint8_t frame[22];
uint8_t int_frame[26];

// standard deviation height/velocity limits
const float h_min = 2.0f;
const float h_max = 8.0f;
const float v_min = 0.5f;
const float v_max = 1.0f;

// polynomial noise model, found using least squares fit
// h, h**2, v, v*h, v*h**2

const float P[5] = {0.04005232f, -0.00656446f, -0.26265873f,  0.13686658f,-0.00397357f};
float flow_vxy_stddev=0;

float flow_x_rad;
float flow_y_rad;
float dt_flow;
float gyro_x_rad = 0;
float gyro_y_rad = 0;
uint16_t f_c;
uint8_t qual;
float rot_sq=0;
float rotrate_sq=0;

matrix::Vector3f delta_b;
matrix::Vector3f delta_n;

float h=0;
float v=0;

// acceleration estimation matrices
matrix::Vector3f x_k;
matrix::Vector3f y_k;
matrix::SquareMatrix<float, 3> ph_k;
matrix::SquareMatrix<float, 3> C_k;
matrix::SquareMatrix<float, 3> P_k;
matrix::SquareMatrix<float, 3> Q_k;
matrix::SquareMatrix<float, 3> R_k;
matrix::SquareMatrix<float, 3> K_k;
matrix::Matrix<float,3,3> SI_k;

matrix::Vector<float,4> x;
matrix::Vector3f y;
matrix::SquareMatrix<float, 4> A;
matrix::Matrix<float, 4, 3> B;
matrix::Matrix<float, 3, 4> C;
matrix::SquareMatrix<float, 4> _P;
matrix::SquareMatrix<float, 4> _Q;
matrix::SquareMatrix<float, 3> R;
matrix::Matrix<float,4,3> K;
matrix::Matrix<float,3,3> S_I;

matrix::Vector<float,2> xk;
matrix::Vector<float,2> yk;
matrix::SquareMatrix<float, 2> Ak;
matrix::SquareMatrix<float, 2> Ck;
matrix::SquareMatrix<float, 2> Pk;
matrix::SquareMatrix<float, 2> Qk;
matrix::SquareMatrix<float, 2> Rk;
matrix::SquareMatrix<float, 2> Kk;
matrix::SquareMatrix<float, 2> SIk;

// 1-2 attitude matrix
matrix::Dcmf Att;
matrix::Dcmf O;

// Velocity of quadrotor body-frame and inertial
matrix::Vector3f V_q;
matrix::Vector3f V_qI,V_qIp;

// Goal position and velocity
matrix::Vector3f P_g;
matrix::Vector3f P_gI;
matrix::Vector3f V_g;

// Object position in the body-frame
matrix::Vector3f P_o,P_oe;

// Object positions in the inertial frame
matrix::Vector3f P_oI;

// Object velocity in the body-frame
matrix::Vector3f V_o;

matrix::SquareMatrix<float, 3> Om; // skew symm. matrix corresponding to omega

matrix::Matrix<float, 3, n_pts > pts;

int i=0; //pts counter
float u_a=0;
float v_a=0;
float w_a=0;

// parameters for adjusting thrust constant
bool do_leveling=false;
int count=0; // counter

// initialize ESC's
bool cal=false; // initialized boolean
bool incr=false; // pulse increment boolean

int n=0;

const int size=6000;

float px[size],py[size],pz[size];
float dist[size];
float min_dist = 1;

float psi_d,phid_d,thetad_d,phi2_d,theta2_d,pd_d,qd_d,rd_d;
float x_e[size],y_e[size],z_e[size],z_o[size];
float u_e[size],v_e[size],w_e[size];
float u_d[size],v_d[size],w_d[size];

float ud[size],vd[size],wd[size];
float udb[size],vdb[size],wdb[size];
float u2,v2,w2;

float phi[size],theta[size],psi[size];
float p[size],q[size],r[size];
float pd[size],qd[size],rd[size];

float e_u[size],e_v[size],e_w[size];
float Ie_u[size],Ie_v[size],Ie_w[size];

float e_phi[size],e_the[size],e_psi[size];
float Ie_ph[size],Ie_th[size],Ie_ps[size];

float e_p[size],e_q[size],e_r[size];
float Ie_p[size],Ie_q[size],Ie_r[size];

float kP[size],bP[size];
float Hx[size],Hy[size],Hz[size],Hp[size];

float phid,thetad,psid,psi2;

float u_c[size],v_c[size],w_c[size];
float Td,ud_c,vd_c,wd_c,psid_c,u2_c,v2_c,w2_c,psi2_c,u3_c,v3_c,w3_c,psi3_c,u4_c,v4_c,w4_c;
float ud_cp,vd_cp,wd_cp,psid_cp,u2_cp,v2_cp,w2_cp,psi2_cp,u3_cp,v3_cp,w3_cp,psi3_cp,u4_cp,v4_cp,w4_cp;
float alu,alv,alw,alud,alvd,alwd,alu2,alv2,alw2,beth,beph,bthd,bphd,bth2,bph2;
float dt,cph,sph,tph,cth,sth,tth,cthd,sthd,cps,sps;

float phi_d[size],theta_d[size],psi_c[size];
float p_d[size],q_d[size],r_d[size];
float t1[size],t2[size],t3[size],T[size];
float om1[size],om2[size],om3[size],om4[size];

int duty1,duty2,duty3,duty4;
float Time[size],sample_freq[size];

float p_gn;
float st_d;

char kcom; // key command
bool traj=false;
bool land=false;
bool avoid=false;
