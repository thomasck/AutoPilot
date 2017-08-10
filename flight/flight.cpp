////////////////////////////////////////////////////////////////////////////
//
//  This file was originally part of RTIMULib and has been modified
//  for the purposes of Thomas Kirven and his masters thesis
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.

/** ---- Information ---- **/
/*  - Most data structures and variables are defined in flight.h */
/*  - The ones defined here are parameters that affect the */
/*    dynamics and control, as well as hardware instances */

#include "flight.h" // flight.h contains all other #inlcudes, data structure inits, and functions

/** ---- Actuator and Control Configuration ---- **/
/*  - define PWM to idle props and fly */
/*  - comment out PWM to disable propellers (sensors and control computations still run)*/
/*  - define FLOW to enable optical flow and velocity control */
/*  - comment out FLOW to track zero attitude and altitude command z_g */
#define PWM
#define FLOW
#define REALSENSE

#ifdef PWM
#define IDLE
#define FLY
#endif

#ifdef FLOW
#define VELOCITY_CTRL
#define DEST_SEEK // enable destination seeking, set x_g,y_g,z_g as destination
//#define FREE_FLY // enable keyboard control
                 // i,k,j,l - commands velocity forward back left and right
                 // z - zeros the velocity command
#endif

#ifdef REALSENSE
#define AVOIDANCE
#endif

/*  -- enable/disable Integral action -- */
/*  - define VIC to use velocity integral control */
/*  - define AIC to use Euler angle integral control */
/*  - define RIC to use rate integral control */
#define VIC
//#define AIC
//#define RIC

/*  -- Adaptive adjustments -- */
/*  Recommend use with zero velocity command and constant height command */
/*  - define ADPT_K to adapt thrust constant k based on e_w  */
/*  - degine ADPT_ATT to adapt reference attitude based on e_u and e_v */
//#define ADPT_K
//#define ADPT_ATT


float g2r=.012383; // raw gyro units to radians (multiplier 1000 deg/s)
//float g2r=.024766; // raw gyro units to radians (multiplier 2000 deg/s)
float a2si=0.001133; // raw accel to m/s^2 (multiplier)

int main() {
  // Bash script which checks if this program (RTIMULibDrive.cpp) is running
  // - if not, it execute 'kpwm' which shuts down the pwm driver
  printf("Starting safe_check...\n");
  system("./safe_check &");

  // set up MPU
  MPU9150 mpu=MPU9150();
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {

    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    printf("DMP ready! Waiting for interrupt...");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    printf("packetSize = %d \n",packetSize);

  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    printf("DMP Initialization failed (code %d ) \n",devStatus);
  }

#ifdef REALSENSE
  rs::log_to_console(rs::log_severity::warn);
  //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

  rs::context ctx;
  if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
  rs::device & dev = *ctx.get_device(0);

  dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
  dev.enable_stream(rs::stream::color, rs::preset::best_quality);
  dev.enable_stream(rs::stream::infrared, rs::preset::best_quality);
  try { dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}
  dev.start();

  apply_depth_control_preset(&dev,5);

  const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
#endif

  // Kalman filter initializations
  // accel
  ph_k.setZero(); Q_k.setZero(); P_k.setZero(); R_k.setZero(); C_k.setZero();
  P_k(0,0)=.01; P_k(1,1)=.01; P_k(2,2)=.01; // initial covariance
  K_k.setZero();
  x_k.setZero();
  x_k(0)=.01; x_k(1)=.01; x_k(2)=.01; // initial state

  // velocity
  x.setZero(); y.setZero();
  A.setZero(); _Q.setZero();
  A = matrix::eye<float,4>(); B.setZero();
  C.setZero(); R.setZero(); _P.setZero();
  S_I.setZero(); K.setZero();
  _P(0,0) = .01; _P(1,1) = .01; _P(2,2) = .01; _P(3,3) = .01;
  x(0)=.01; x(1)=.01; x(2)=.01; x(3)=.01;

  // other velocity
  Ak.setZero(); Ck.setZero(); Rk.setZero(); Pk.setZero();
  Pk(0,0) = .02; Pk(1,1) = .02;

  /* Initial conditions */
  px[0]=10; py[0]=10; pz[0]=0;dist[0]=10;

  x_e[0]=0; y_e[0]=0; z_e[0]=0;
  z_o[0]=0;
  u_e[0]=0; v_e[0]=0; w_e[0]=0; // 1st intermediate frame velocities
  ud[0]=0; vd[0]=0; wd[0]=0; // 1st intermediate frame accelerations
  udb[0]=0; vdb[0]=0; wdb[0]=0; // body accelerations

  phi[0]=0; theta[0]=0; psi[0]=0; // Euler angles
  phid=0; thetad=0; psid=0; // Euler ange rates
  p[0]=0; q[0]=0; r[0]=0; // body rates
  pd[0]=0; qd[0]=0; rd[0]=0; // rate derivatives

  e_u[0]=0; e_v[0]=0; e_w[0]=0; // velocity errors
  e_phi[0]=0; e_the[0]=0; e_psi[0]=0; // angle errors
  e_p[0]=0; e_q[0]=0; e_r[0]=0; // rate errors

  Ie_u[0]=0; Ie_v[0]=0; Ie_w[0]=0; // integral of velocity errors
  Ie_ph[0]=0; Ie_th[0]=0; Ie_ps[0]=0; // integral of angle errors
  Ie_p[0]=0; Ie_q[0]=0; Ie_r[0]=0; // integral of rate errors

  alw=-10.0; alwd=.1;

  psi_d=0; // desired yaw

  Time[0]=0; // initial time
  sample_freq[0]=100;

  // reference model derivatives (previous value holders)
  ud_cp=0; vd_cp=0; wd_cp=0; psid_cp=0;
  u2_cp=0; v2_cp=0; w2_cp=0; psi2_cp=0;
  u3_cp=0; v3_cp=0; w3_cp=0; psi3_cp=0;
  u4_cp=0; v4_cp=0; w4_cp=0;

  // refrence model commands
  u_c[0]=0; v_c[0]=0; w_c[0]=0; psi_c[0]=0;
  ud_c=0; vd_c=0; wd_c=0; psid_c=0;
  u2_c=0; v2_c=0; w2_c=0; psi2_c=0;
  u3_c=0; v3_c=0; w3_c=0; psi3_c=0;


  // EKF init for acceleration
  R_k(0,0)=1; R_k(1,1)=1; R_k(2,2)=1; // accel sensor noise diag(a_x,a_y,a_z)
  C_k(0,0) = 1; C_k(1,1) = 1; C_k(2,2) = 1;

  // process noise variances
  float phdv=.01*.01;
  float thdv=.01*.01;
  float psdv=.01*.01;

  // EKF init for velocity
  C(0,0) = 1; C(1,1) = 1; C(2,3) = 1; // measurement matrix
  _Q(0,0)=0.00001; _Q(1,1)=0.00001; _Q(2,2)=.05; _Q(3,3)=.0005; // process noise matrix

  R(2,2) = .01;
  // skew sym matrix for omega (zeros on diagonal)
  Om.setZero();
  V_q.setZero();
  V_qI.setZero();
  V_qIp.setZero();

  // set up console io
  struct termios ctty;

  tcgetattr(fileno(stdout), &ctty);
  ctty.c_lflag &= ~(ICANON);
  tcsetattr(fileno(stdout), TCSANOW, &ctty);

    // Set up flow module on i2c-1
#ifdef FLOW
  mraa::I2c* flow;
  flow = new mraa::I2c(0);
  flow->address(0x42);
#endif

  // set up pwm driver
  int freq=500; // maximum frequency 500 hz
  float per=1000000/freq; // pwm period in microseconds
  int high=(int)(2000/per*4095+.5); // integer pulse value cooresponding to 2 ms
  int low=(int)(650/per*4095+.5); // integer pulse value cooresponding to .65 ms

  int inc=(int)((high-low)/200+.5); // pulse width increment
  float wmin=173.0; // min prop rate (rad/s)
  float wmax=911.0;  // max prop rate (rad/s)

  int pulse=low;
  int min1=low; int min2=low; int min3=low; int min4=low;
  int curPwm=15;

  // set up PCA9685 driver on i2c-0
#ifdef PWM
  upm::PCA9685 *pwm = new upm::PCA9685(0,0x40);

  pwm->setModeSleep(true);
  pwm->setPrescaleFromHz(freq);
  pwm->setModeSleep(false);
  usleep(50000);

  printf("ESC frequency set to %d Hz \n",freq);

  pwm->ledOnTime(PCA9685_ALL_LED, 0); // pulse on
  pwm->ledOffTime(PCA9685_ALL_LED, low); // pulse off

  printf("c - Calibrate ESC's\n");
  printf("s - use saved EPROM calibration\n\n");

  while(!cal) {
    if ((kcom = getUserChar()) != 0) {

      switch (kcom) {

      case 'c':
	printf("1) Disconnect ESC's from battery\n");
	printf("2) Key h to set pulsewidth to high (1.999 ms)\n");
	printf("3) Reconnect ESC's to battery\n");
	printf("4) Key l to set pulsewidth to low (.65 ms)\n");
	printf("5) Key i to increment pulswidth until motors start to spin\n");
	printf("6) Key x to save effective pulsewidth range\n");
	break;

      case 's':
	cal=true;
	min1=1425; min2=1425; min3=1550; min4=1540;
	printf(" - using EPROM saved throttle range \n");
        printf("min1=%d; min2=%d; min3=%d; min4=%d; \n",min1,min2,min3,min4);
	printf(" - done \n");
        break;

      case 'h' :
	incr=true;
	pulse=high-1; // only works when slightly < 2.0 ms
	printf(" - setting high (%f ms) \n",(float)pulse/4095.0*per/1000);
	break;

      case 'l' :
        incr=true;
        pulse=low;
        printf(" - setting low (%f ms) \n",(float)pulse/4095.0*per/1000);
        break;

      case 'i' :
	incr=true;
	pulse=pulse+inc;
	if (pulse>high) pulse=high;
	printf(" - incremented to %f ms pulse \n",(float)pulse/4095.0*per/1000);
	break;

      case 'd' :
        incr=true;
        pulse=pulse-inc;
	if (pulse<low) pulse=low;
        printf(" - decremented to %f ms pulse \n",(float)pulse/4095.0*per/1000);
        break;

      case 'q' :
        incr=true;
        inc=inc+1;
	printf(" - increment value changed to %d\n",inc);
	break;

      case 'a' :
        incr=true;
       	inc=inc-1;
	if (inc<=0) inc=1;
        printf(" - increment value changed to %d\n",inc);
        break;

      case 'w':
	min1=pulse;
	pwm->ledOffTime(curPwm, low);
	curPwm=1;
	break;

      case 'e':
	min2=pulse;
	pwm->ledOffTime(curPwm, low);
	curPwm=14;
	break;

      case 'r':
	min3=pulse;
	pwm->ledOffTime(curPwm, low);
	curPwm=0;
	break;

      case 'x' :
	min4=pulse;
	pwm->ledOffTime(curPwm, low);
	printf("min1=%d; min2=%d; min3=%d; min4=%d; \n",min1,min2,min3,min4);
	cal=true;
	break;
      }

      if (incr) {

	pwm->ledOffTime(curPwm,pulse);
	// uncomment to calibrate ESC's
/*	pwm->ledOffTime(0,pulse);
	pwm->ledOffTime(1,pulse);
	pwm->ledOffTime(14,pulse);
	pwm->ledOffTime(15,pulse);
*/
	incr=false;
      }
    }
  }
#endif

  float rng=high-max(max(min1,min2),max(min3,min4));  // effective pwm range
  //printf("range = %f \n",rng);

  // physical parameters
  float m = 1.082; // mass of quadrotor 1.01024 w/2200mah lip  (.9217 w/1300 mah lipo)
  m = 1.02;
  //m = .9217;
  float g = 9.80665; // magnitude of acceleration due to gravity
  float ll = .1185; // moment arm
  float ls = .1185;
  float k = 6.0*pow(10,-6); // (7.5-6.0)*E-6 8045 thrust constant (wi^.5)
  //float k = 1.29*pow(10,-3); // .00122 - .0014 thrust constant (wi^1)
  kP[0]=k; //initial k
  float b = 1.4*pow(10,-7); // (7.0-10.0*E-8) 8045  // propeller drag constant (wi^.5)
  //float b = .000122;
  bP[0]=b;

  // principle inertias
  float Ixx=.00585; // .00585
  float Iyy=.00545; // .0054
  float Izz=.0072; // .01 .. .008
  float Ip=.00002; // .00002 propeller lengthwise inertia
/*
  Ixx=.00561; // .00533
  Iyy=.00551; // .0049
  Izz=.0066; // .00655
*/
  // velocity gains
  float ku=2.0;
  float kv=2.0;
  float kw=2.0;
  float kiw=1.0;

  // attitude gains
  float kphi=7.0;
  float ktheta=7.0;
  float kpsi=7.0;

  // rate gains
  float kp=7.0;
  float kq=7.0;
  float kr=7.0;

  // integral gains remain zero unless integral controls are defined
  float kiu=0; float kiv=0;
  float kiph=0; float kith=0; float kips=0;
  float kip=0; float kiq=0; float kir=0;

#ifdef VIC // velocity integral control
  kiu=1.0;
  kiv=1.0;
#endif

#ifdef AIC // angle integral control
  kiph=4.5;
  kith=4.5;
  kips=4.5;
#endif

#ifdef RIC // rate integral control
  kip=6.0;
  kiq=6.0;
  kir=6.0;
#endif

  // reference model parameters
  float a0=625;
  float a1=500;
  float a2=150;
  float a3=20; // p=-5

  // reference velocity command parameters // pole location
  //a0=28561; a1=8788; a2=1014; a3=52; // p=-13
  //a0=2401; a1=1372; a2=294; a3=28; //p=-7

  // reference angle command for yaw
  float b0=27;
  float b1=27;
  float b2=9; // p=-6

  float sd=.5; // set speed of quadcopter
  st_d=1.0; // stopping distance in terms of speed

  // set goal position
  float x_g=0;
  float y_g=0;
  float z_g=-1.0;

  P_g(0) = x_g;
  P_g(1) = y_g;
  P_g(2) = z_g;

  p_gn = P_g.norm();

  // inner and outer ellipsoid semi-principles axes
  float ai=ll*4.3; float bi=ll*3.15; float ci=ll*2.0;
  float ao=1.8*ai; float bo=1.8*bi; float co=1.8*ci;

  printf("outer ellipsoid semi principle axes: %f, %f, %f \n",ao,bo,co);
  printf("inner ellipsoid semi principle axes: %f, %f, %f \n",ai,bi,ci);

  // Ellipsoid radii and potential
  float R_o,R_i,U_e;

  float ep=1.1; // potential constant

  float d_min=2; // closest approach to obstacle (meters)
  bool col=false; // collision boolean
  int cnt = 0;

  pts.setAll(10);
  P_o.setZero();

  u_d[0]=0; v_d[0]=0; w_d[0]=0;

  t1[0]=0; t2[0]=0; t3[0]=0;
  // initial thrust;
  T[0]=m*g;

  om1[0]=T[0]/(4.0*k);
  om2[0]=T[0]/(4.0*k);
  om3[0]=T[0]/(4.0*k);
  om4[0]=T[0]/(4.0*k);

  om1[0]=sqrt(om1[0]); om2[0]=sqrt(om2[0]);
  om3[0]=sqrt(om3[0]); om4[0]=sqrt(om4[0]);

  Hx[0]=Ixx*p[0]; Hy[0]=Iyy*q[0]; Hz[0]=Izz*r[0];
  Hp[0]=(om1[0]+om3[0]-om2[0]-om4[0])*Ip;

  // set up loop timing
  auto prev = std::chrono::high_resolution_clock::now();
  auto now = std::chrono::high_resolution_clock::now();
  long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now-prev).count();

  float phi_r=0; float theta_r=0; float psi_r=0;
  float arx=0; float ary=0; float arz=0;
  // let dmp settle and get reference orientaion (refQ)
  int acnt=0;
  for (int j=0; j<1000; j++) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      printf("fifo overflow!");

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&refQ,fifoBuffer); // obviously only last refQ is saved
      refQ.normalize();
      corrQ = (refQ.getConjugate()).getProduct(refQ);
      corrQ.normalize();

      if (j>699) {
        mpu.dmpGetEuler(ypr_r, &refQ);
	phi_r=phi_r+ypr_r[2];
	theta_r=theta_r+ypr_r[1];
	psi_r=psi_r+ypr_r[0];

	mpu.dmpGetAccel(&ar,fifoBuffer);
	mpu.dmpGetGravity(&gravity, &corrQ);
        mpu.dmpGetLinearAccel(&aaReal, &ar, &gravity);

	arx=arx+aaReal.x;
	ary=ary+aaReal.y;
	arz=arz+aaReal.z;
	acnt=acnt+1;
      }
    }
  }
  arx=arx/acnt;
  ary=ary/acnt;
  arz=arz/acnt;
  phi_r=phi_r/acnt;
  theta_r=theta_r/acnt;
  psi_r=psi_r/acnt;

  ypr_r[0]=psi_r;
  ypr_r[1]=theta_r;
  ypr_r[2]=phi_r;

  printf("IMU orientation on ground : \n roll = %f pitch = %f yaw = %f \n",
	 ypr_r[2],-ypr_r[1],-ypr_r[0]);
  // hard code reference orientation (roll&pitch) -0.060702 pitch = -0.013980
  ypr_r[1]=0.03; // pitch (switch sign) roll = -0.082126 pitch = -0.013426
  ypr_r[2]=-.064;  // roll
  refQ=fromEuler(ypr_r);
  refQ.normalize();

  // accelerometer offset from center of mass
  //float rx=.012;
  //float rz=-.025;

  // Idle propellers at 1/30th full throttle
#ifdef IDLE
  pwm->ledOffTime(0,min1+rng/15);
  pwm->ledOffTime(1,min2+rng/15);
  pwm->ledOffTime(14,min3+rng/15);
  pwm->ledOffTime(15,min4+rng/15);
#endif

  uint8_t rate=mpu.getRate();
  printf("sample rate set to %d hz\n",1000/(1+rate));
  uint8_t fsync=mpu.getExternalFrameSync();
  printf("Fsync config value set to %d \n",fsync);
  uint8_t mode=mpu.getDLPFMode();
  printf("DLPF mode %d \n",mode);

  // get takeoff command
  bool takeoff=false;
  while (!takeoff) {
    std::cout.flush();
    printf("Takeoff? y/n: \n");
    switch (tolower(getchar())) {
    case 'y' :
      takeoff=true;
      break;

    case 'n' :
#ifdef PWM
      setAllPWM(pwm,low);
#endif
#ifdef FLOW
      delete flow;
#endif
      printf(" - Exiting. \n");
      exit(1);
      break;
    }
  }
  prev = std::chrono::high_resolution_clock::now();

  while (1) {
    /*
    if (!dmpReady) {
#ifdef PWM
      setAllPWM(pwm,low);
#endif

#ifdef FLOW
      delete flow;
#endif
      // write data to .txt files
      write_Freq(n,Time,sample_freq);
      write_Position(n,x_e,y_e,z_e);
      write_Velocity(n,Time,u_e,v_e,w_e,u_c,v_c,w_c);
      write_Acceleration(n,Time,ud,vd,wd);
      write_Angles(n,Time,phi,theta,psi,phi_d,theta_d,psi_c);
      write_Angle_Errors(n,Time,e_phi,e_the,e_psi);
      write_Rates(n,Time,p,q,r,p_d,q_d,r_d);
      write_Rate_Errors(n,Time,e_p,e_q,e_r);
      write_Inputs(n,Time,T,t1,t2,t3,om1,om2,om3,om4);

      exit(1);
      break;
    }
    */
    // reset interrupt flag and get INT_STATUS byte (resets after read)
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();	// reset so we can continue cleanly
      printf("fifo overflow!");
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      //printf("FFdmpIntStatus = %X\n",dmpIntStatus);
      //printf("FFmpuIntStatus = %X\n",mpuIntStatus);
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&Q,fifoBuffer);
      mpu.dmpGetGyro(gyro,fifoBuffer);

      now = std::chrono::high_resolution_clock::now();
      microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now-prev).count();
      dt=(float)(microseconds)/1000000.0;
      prev = now;
      Time[n+1] = Time[n]+dt;
      sample_freq[n+1]=1.0/dt;

      // corrected quaternion
      corrQ = (refQ.getConjugate()).getProduct(Q);
      corrQ.normalize();

      /*
	mpu.dmpGetYawPitchRoll(ypr,&corrQ,&gravity);
	phi[n+1] = ypr[2];
	theta[n+1] = ypr[1];
	psi[n+1] = ypr[0];
      */
      // Euler angles
      mpu.dmpGetEuler(ypr, &corrQ);
      phi[n+1] = ypr[2];
      theta[n+1] = -ypr[1];
      psi[n+1] = -ypr[0];

      // define current trig values. save space.
      cph = cos(phi[n+1]); sph = sin(phi[n+1]); tph = tan(phi[n+1]);
      cth = cos(theta[n+1]); sth = sin(theta[n+1]); tth = tan(theta[n+1]);
      cps = cos(psi[n+1]); sps = sin(psi[n+1]);

      // angular velocity
      p[n+1] = gyro[0]*g2r+.034;
      q[n+1] = -gyro[1]*g2r-.0185;
      r[n+1] = -gyro[2]*g2r-.0062;

      // kinematics
      phid = p[n+1]+q[n+1]*sph*tth+r[n+1]*cph*tth;
      thetad = q[n+1]*cph-r[n+1]*sph;
      psid = q[n+1]*sph/cth+r[n+1]*cph/cth;

      // angular momentum
      //Hx[n+1]=Ixx*p[n+1]; Hy[n+1]=Iyy*q[n+1]; Hz[n+1]=Izz*r[n+1];

#ifdef REALSENSE
      if(dev.is_streaming()) dev.wait_for_frames();
      auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
      //auto depth = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth));
      min_dist = 30;
      for(int yy=0; yy<depth_intrin.height; ++yy) {
	for(int xx=0; xx<depth_intrin.width; ++xx) {
	  if (points->z){
	    dist[n+1] = points->x*points->x+points->y*points->y+points->z*points->z;
	    if (dist[n+1]<min_dist) {
	      min_dist = dist[n+1];
	      px[n+1] = points->x;
	      py[n+1] = points->y;
	      pz[n+1] = points->z;
	    }
	  }
	  ++points;
	}
      }

      dist[n+1] = sqrt(min_dist);
#endif

      // if roll or pitch is dangerously large, kill pwm and write data
      if ((abs(phi[n+1])>1.3)||(abs(theta[n+1])>1.3)) {

#ifdef PWM
	setAllPWM(pwm,low);
#endif

#ifdef FLOW
	delete flow;
#endif
	printf("FLIP\n");
	printf("phi[n+1] = %f \t theta[n+1] = %f\n",phi[n+1],theta[n+1]);
	printf("Closest object approach: %f \n",d_min);

	write_Params(n,Time,kP,bP);
	write_Map(i,pts);

	// write data to .txt files for gnuplot
	write_Freq(n,Time,sample_freq);
	write_Position(n,Time,x_e,y_e,z_e,z_o);
	write_Velocity(n,Time,u_e,v_e,w_e,u_c,v_c,w_c);
	//write_Vel_Int(n,Time,Ie_u,Ie_v,Ie_w);
	write_Vel_Int(n,Time,u_d,v_d,w_d);
	write_Acceleration(n,Time,ud,vd,wd);
	write_Angles(n+1,Time,phi,theta,psi,phi_d,theta_d,psi_c);
	write_Angle_Errors(n,Time,e_phi,e_the,e_psi);
	write_Rates(n,Time,p,q,r,p_d,q_d,r_d);
	write_Rate_Errors(n,Time,e_p,e_q,e_r);
	write_Momentum(n,Time,Hx,Hy,Hz,Hp);
	write_Inputs(n,Time,T,t1,t2,t3,om1,om2,om3,om4);
	write_Pos_Obj(n,Time,px,py,pz,dist);

	exit(1);
      }

      mpu.dmpGetAccel(&aa,fifoBuffer);
      mpu.dmpGetGravity(&gravity, &corrQ);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      aaReal.x=aaReal.x-arx;
      aaReal.y=aaReal.y+ary;
      aaReal.z=aaReal.z;
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &corrQ);

      // inertial acceleration
      ud[n+1] = (aaWorld.x)*a2si;
      vd[n+1] = -(aaWorld.y)*a2si;
      wd[n+1] = -(aaWorld.z)*a2si;

      y_k(0)=ud[n+1];
      y_k(1)=vd[n+1];
      y_k(2)=wd[n+1];

      // here, O is roll-pitch-yaw consecutive rotation (body to inertial)
      O = matrix::Eulerf(phi[n+1],theta[n+1],psi[n+1]);

      // extended kalman filter for acceleration estimates
      ph_k(0,0) = 1+alwd*dt/alw;
      ph_k(0,1) = -psid*dt;
      ph_k(0,2) = -kw*x_k(0)*dt/alw;

      ph_k(1,0) = psid*dt;
      ph_k(1,1) = 1+alwd*dt/alw;
      ph_k(1,2) = -kw*x_k(1)*dt/alw;

      ph_k(2,2) = 1-kw*dt;

      Q_k(0,0) = (psid*(psid*(phdv*alw*alw*cps*cps+thdv*alw*alw*sps*sps+psdv*x_k(0)*x_k(0))+(alwd*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv))/alw)+(alwd*(psid*(psdv*x_k(0)*x_k(1) + alw*alw*cps*phdv*sps - alw*alw*cps*sps*thdv)+(alwd*(thdv*alw*alw*cps*cps+phdv*alw*alw*sps*sps+psdv*x_k(1)*x_k(1)))/alw))/alw)*dt*dt*dt/3+(2*psid*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv)+(2*alwd*(thdv*alw*alw*cps*cps+phdv*alw*alw*sps*sps+psdv*x_k(1)*x_k(1)))/alw)*dt*dt/2 + (thdv*alw*alw*cps*cps+phdv*alw*alw*sps*sps+psdv*x_k(1)*x_k(1))*dt;

      Q_k(0,1) = (psid*(psid*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv)+(alwd*(thdv*alw*alw*cps*cps+phdv*alw*alw*sps*sps+psdv*x_k(1)*x_k(1)))/alw)-(alwd*(psid*(phdv*alw*alw*cps*cps + thdv*alw*alw*sps*sps + psdv*x_k(0)*x_k(0))+(alwd*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv))/alw))/alw)*dt*dt*dt/3+(psid*(thdv*alw*alw*cps*cps+phdv*alw*alw*sps*sps+psdv*x_k(1)*x_k(1))-psid*(phdv*alw*alw*cps*cps+thdv*alw*alw*sps*sps+psdv*x_k(0)*x_k(0))-(2*alwd*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv))/alw)*dt*dt/2+(alw*alw*cps*sps*thdv-alw*alw*cps*phdv*sps-psdv*x_k(0)*x_k(1))*dt;

      Q_k(1,0) = (psid*(psid*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv)-(alwd*(phdv*alw*alw*cps*cps+thdv*alw*alw*sps*sps+psdv*x_k(0)*x_k(0)))/alw)+(alwd*(psid*(thdv*alw*alw*cps*cps + phdv*alw*alw*sps*sps + psdv*x_k(1)*x_k(1))-(alwd*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv))/alw))/alw)*dt*dt*dt/3+(psid*(thdv*alw*alw*cps*cps+phdv*alw*alw*sps*sps+psdv*x_k(1)*x_k(1))-psid*(phdv*alw*alw*cps*cps+thdv*alw*alw*sps*sps+psdv*x_k(0)*x_k(0))-(2*alwd*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv))/alw)*dt*dt/2+(alw*alw*cps*sps*thdv-alw*alw*cps*phdv*sps-psdv*x_k(0)*x_k(1))*dt;

      Q_k(1,1) = (psid*(psid*(thdv*alw*alw*cps*cps+phdv*alw*alw*sps*sps+psdv*x_k(1)*x_k(1))-(alwd*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv))/alw)-(alwd*(psid*(psdv*x_k(0)*x_k(1) + alw*alw*cps*phdv*sps - alw*alw*cps*sps*thdv)-(alwd*(phdv*alw*alw*cps*cps+thdv*alw*alw*sps*sps+psdv*x_k(0)*x_k(0)))/alw))/alw)*dt*dt*dt/3+((2*alwd*(phdv*alw*alw*cps*cps+thdv*alw*alw*sps*sps+psdv*x_k(0)*x_k(0)))/alw-2*psid*(psdv*x_k(0)*x_k(1)+alw*alw*cps*phdv*sps-alw*alw*cps*sps*thdv))*dt*dt/2+(phdv*alw*alw*cps*cps+thdv*alw*alw*sps*sps+psdv*x_k(0)*x_k(0))*dt;

      P_k = ph_k*P_k*ph_k.transpose()+Q_k;
      SI_k = matrix::inv<float, 3>(C_k*P_k*C_k.transpose()+R_k);
      K_k = P_k*C_k.transpose()*SI_k;

      // predict .. x-k = x(k-1) + xd(k-1)*dt
      x_k(0) = x_k(0) + (alwd*x_k(0)/alw-psid*x_k(1)+alw*(thetad*cps+phid*sps))*dt;
      x_k(1) = x_k(1) + (alwd*x_k(1)/alw+psid*x_k(0)+alw*(thetad*sps-phid*cps))*dt;
      x_k(2) = x_k(2) + alwd*dt;

      // correct
      x_k += K_k*(y_k-x_k);

      // update covariance
      P_k -= K_k*C_k*P_k;

      // accelerations in inertial frame
      ud[n+1]=x_k(0);
      vd[n+1]=x_k(1);
      wd[n+1]=x_k(2);

      // update model
      A(3,2) = dt;

      // input matrix
      B(0,0)=dt;
      B(1,1)=dt;
      B(2,2)=dt;
      B(3,2)=dt*dt/2;

      // get optical flow data
#ifdef FLOW
      flow->readBytesReg(0x00,frame,22);
      flow->readBytesReg(0x16,int_frame,26);
#endif

      z_e[n+1]=-(int16_t)(frame[20] | frame[21] << 8)/(1.0e3f);
      if (abs(z_e[n+1]-z_e[n])>.3) z_e[n+1]=z_e[n]+w_e[n]*dt;
      //if (abs(z_e[n+1]-z_e[n])>.3) z_e[n+1]=z_e[n+1]-;
      h=-z_e[n+1];

/*      qual = (uint8_t)(int_frame[24]);
      if (qual < 50 && !land) {

        u_e[n+1]=x(0);//+ud[n+1]*dt; // u estimate
        v_e[n+1]=x(1);//+vd[n+1]*dt; // v estimate
        w_e[n+1]=x(2);//+wd[n+1]*dt; // w estimate
        z_e[n+1]=x(3);//+w_e[n+1]*dt; // z estimate (height)

        x_e[n+1] = x_e[n]+V_qI(0)*dt;
        y_e[n+1] = y_e[n]+V_qI(1)*dt;

        if (z_e[n+1]>-.3) z_e[n+1]=-.3;
      } else {
*/
	dt_flow = (uint32_t)(int_frame[12] | int_frame[13] << 8 | int_frame[14] << 16 | int_frame[15] << 24)/1.0e6f;

        if (dt_flow > 0.5f || dt_flow < 1.0e-2f) {
	  dt_flow = dt;
        }

        flow_x_rad = (int16_t)(int_frame[2] | int_frame[3] << 8)*1.3f/1.0e4f;
        flow_y_rad = (int16_t)(int_frame[4] | int_frame[5] << 8)*1.3f/1.0e4f;

        gyro_x_rad = (int16_t)(int_frame[6] | int_frame[7] << 8)*1.3f/1.0e4f;
        gyro_y_rad = (int16_t)(int_frame[8] | int_frame[9] << 8)*1.3f/1.0e4f;

        y(0) = -(flow_y_rad - gyro_y_rad)*z_e[n+1]/dt_flow;
        y(1) = (flow_x_rad - gyro_x_rad)*z_e[n+1]/dt_flow;

	// get rate compensated velocity in bodyframe
//	y(0)=(int16_t)(frame[6] | frame[7] << 8)/6.95e2f;
//        y(1)=(int16_t)(frame[8] | frame[9] << 8)/6.95e2f;

	// estimate b3 direction velocity
        y(2)=(u_e[n]+ud[n]*dt)*O(0,2)+(v_e[n]+vd[n]*dt)*O(1,2)+(w_e[n]+wd[n]*dt)*O(2,2);

        V_q=y; // save quad velocity in body frame

        V_qI = O * V_q; // inertial quad velocity

//	u_e[n+1] = V_qI(0);
//      v_e[n+1] = V_qI(1);

        v=sqrt(y(0)*y(0)+y(1)*y(1)); // speed

        y = V_qI;

        y(2)=z_e[n+1];

        // limits on height and speed for stddev calculation
        if (h > h_max) h = h_max;
        if (h < h_min) h = h_min;
        if (v > v_max) v = v_max;
        if (v < v_min) v = v_min;

        // optical flow error estimate
        flow_vxy_stddev = P[0] * h + P[1] * h * h + P[2] * v + P[3] * v * h + P[4] * v * h * h;

	rot_sq = phi[n+1]*phi[n+1]+theta[n+1]*theta[n+1];
	rotrate_sq = p[n+1]*p[n+1]+q[n+1]*q[n+1]+r[n+1]*r[n+1];

        R(0,0) = flow_vxy_stddev * flow_vxy_stddev;//+20.0f*(rot_sq+rotrate_sq);
        R(1,1) = R(0,0);

        // predict states
        x = A * x + B * x_k;
        // calculate residual (measured-predicted)
        _P = A*_P*A.transpose() + _Q;
        // residual covariance, (inverse)
        S_I = matrix::inv<float, 3>(C * _P * C.transpose() + R);
        K = _P * C.transpose() * S_I; // Kalman gain
        _P -= K * C * _P;
        // correct states
        x += K * (y - C * x);

        // filter output
        u_e[n+1]=x(0); // u estimate
        v_e[n+1]=x(1); // v estimate
        w_e[n+1]=x(2); // w estimate

        x_e[n+1] = x_e[n]+(V_qI(0)+V_qIp(0))*dt/2;
        y_e[n+1] = y_e[n]+(V_qI(1)+V_qIp(1))*dt/2;
	z_e[n+1]=x(3); // z estimate (height)

        if (z_e[n+1]>-.3) z_e[n+1]=-.3;
  //    }
#ifdef DEST_SEEK
      // omega skew sym. matrix
      /*
      Om(0,1) = -r[n+1];
      Om(0,2) = q[n+1];
      Om(1,0) = r[n+1];
      Om(1,2) = -p[n+1];
      Om(2,0) = -q[n+1];
      Om(2,1) = p[n+1];

      // enhance estimate of P_g using measured height
      P_gI = O * P_g; // body to inertial
      P_gI(2) = z_e[n+1] - z_g; // hard code height error since we have height measure
      P_g = O.transpose() * P_gI; // put back into body frame

      // propogate P_g
      V_g = - V_q - Om*P_g;
      P_g = P_g + V_g*dt;

      // rotate new P_g into inertial frame for control
      P_gI = O * P_g;*/
      P_gI(2) = z_e[n+1] - z_g; // might as well hard code this measurement again
      P_gI(0) = x_e[n+1] - x_g;
      P_gI(1) = y_e[n+1] - y_g;

      p_gn=P_gI.norm(); // distance to goal

      if (p_gn<.002||land) { // go straight down
	//land = true;
	sd=.8;
	P_gI(2)=-1;
      }

      // compute destination seeking velocity
      if (p_gn >= st_d) {
	u_d[n+1]=-sd*P_gI(0)/p_gn;
	v_d[n+1]=-sd*P_gI(1)/p_gn;
	w_d[n+1]=-sd*P_gI(2)/p_gn;
      } else {
	u_d[n+1]=-sd*P_gI(0)/st_d;
	v_d[n+1]=-sd*P_gI(1)/st_d;
	w_d[n+1]=-sd*P_gI(2)/st_d;
      }
#ifdef AVOIDANCE
      if (!land && avoid) {

	cnt=0;
	u_a = 0;
	v_a = 0;
	w_a = 0;

	// update relative position of points
	for (int j=0; j<n_pts; j++) {
	  pts(0,j) = pts(0,j)-(V_qI(0)+V_qIp(0))*dt/2;
	  pts(1,j) = pts(1,j)-(V_qI(1)+V_qIp(1))*dt/2;
	  pts(2,j) = pts(2,j)-(V_qI(2)+V_qIp(2))*dt/2;

	  P_oI(0) = pts(0,j);
	  P_oI(1) = pts(1,j);
	  P_oI(2) = pts(2,j);

	  P_o = O.transpose() * P_oI;

	  if (in_Elpsd(P_o,ao,bo,co)) {
	    cnt++;
	    // get outer and inner ellipsoid radii
	    R_o = ElpsdRad(P_o,ao,bo,co);
	    R_i = ElpsdRad(P_o,ai,bi,ci);

	    U_e = ep*(1-(P_o.norm() - R_i)/(R_o - R_i));
	    //printf("U_e = %f\n",U_e);

	    // add avoidance velocities
	    u_a=u_a + U_e * P_oI(0)/P_oI.norm();
	    v_a=v_a + U_e * P_oI(1)/P_oI.norm();
	    w_a=w_a + U_e * P_oI(2)/P_oI.norm();
	  }
	}
	P_o(0) = pz[n+1];
	P_o(1) = px[n+1];
	P_o(2) = py[n+1];

        if (!(P_o(0)==0||P_o(1)==0||P_o(2)==0)) {

	  P_oI = O * P_o;

	  pts(0,i) = P_oI(0);
	  pts(1,i) = P_oI(1);
	  pts(2,i) = P_oI(2);

	  if (++i == n_pts) i=0;

	  if (in_Elpsd(P_o,ao,bo,co)) {
	    cnt++;
	    // get outer and inner ellipsoid radii
	    R_o = ElpsdRad(P_o,ao,bo,co);
	    R_i = ElpsdRad(P_o,ai,bi,ci);

	    U_e = ep*(1-(P_o.norm() - R_i)/(R_o - R_i));
	    //printf("U_e = %f\n",U_e);

	    // add avoidance velocities
	    u_a=u_a + U_e * P_oI(0)/P_oI.norm();
	    v_a=v_a + U_e * P_oI(1)/P_oI.norm();
	    w_a=w_a + U_e * P_oI(2)/P_oI.norm();
	  }
        }

	if (cnt>25) {
	  float n_a = sqrt(u_a/cnt*u_a/cnt+v_a/cnt*v_a/cnt);
	  if (n_a>1.5) {
             u_a=u_a/n_a;
	     v_a=v_a/n_a;
	  }
	  u_d[n+1]=u_d[n+1] - u_a/cnt;
	  v_d[n+1]=v_d[n+1] - v_a/cnt;
//	  w_d[n+1]=w_d[n+1] - w_a/cnt;

	  //printf("u_a = %f \n",-u_a/cnt);
	  //printf("v_a = %f \n",-v_a/cnt);
	  //printf("w_a = %f \n",-w_a/cnt);
	}
      }
      //printf("psi_d = %f\n",atan2(v_d[n+1],u_d[n+1]));
#endif
#endif
V_qIp(0) = V_qI(0);
V_qIp(1) = V_qI(1);
//psi_d = atan2(v_d[n+1],u_d[n+1]);
//printf("psi_d = %f\n",psi_d);
//psi_d=0;
#ifdef FREE_FLY

      P_gI(2) = z_e[n+1]-z_g; // relative k position of goal

      if (land) { // go straight down
	P_gI(2)=-1;
      }

      p_gn=abs(P_gI(2)); // distance to goal

      // compute w_d
      if (p_gn >= st_d) {
	w_d[n+1]=-sd*P_gI(2)/p_gn;
      } else {
	w_d[n+1]=-sd*P_gI(2)/st_d;
      }

      u_d[n+1]=u_d[n];
      v_d[n+1]=v_d[n];

#endif

      // put trajectories here
      if (traj) {
	// circle trajectory
	u_d[n+1]=1.0*cos(Time[n+1]*2.5);
	v_d[n+1]=1.0*sin(Time[n+1]*2.5);
      }

      // get key commands if any
      if ((kcom = getUserChar()) != 0) {
	switch (kcom) {

	case 'a' : // avoid objects
	  avoid = true;
	  x_g = 3.0;
	  //y_g = -1.0;
	  break;

	case 's' : // begin trajectory
	  traj=true;
	  break;

	case 'f' : // yaw to 90 degrees
	  //psi_d=psi_d+.1;
	  psi_d=pi/2;
	  break;

	case 'd': // yaw to -90 degrees
	  //psi_d=psi_d-.1;
	  psi_d=-pi/2;
	  break;

	case ' ' : // land (space bar)
	  kP[0]=k;
	  traj=false;
	  avoid=false;
	  u_d[n+1]=0;
	  v_d[n+1]=0;
	  x_g = x_e[n+1];
	  y_g = y_e[n+1];
	  land=true;
	  sd=.6;
	  z_g = 1;
	  break;

	  // i,j,k,l -- sets u_d,-v_d,-u_d,v_d to .5 m/s respectively
	  // must press z to set u_d,v_d back to zero
	case 'l' :
	  v_d[n+1]=2.0;
	  break;

	case 'j' :
	  v_d[n+1]=-2.0;
	  break;

	case 'i' :
	  u_d[n+1]=2.0;
	  break;

	case 'k' :
	  u_d[n+1]=-2.0;
	  break;

	case 'z' : // hover; zero velocity and attitude, stop trajectory
	  avoid = false;
	  traj=false;
	  u_d[n+1]=0;
	  v_d[n+1]=0;
	  //psi_d=0;
	  break;

	case 'x' : // shutdown pwm, write data, exit program
#ifdef PWM
	  setAllPWM(pwm,low);
#endif

#ifdef FLOW
	  delete flow;
#endif

	  if (col) printf("Collision occured\n");
	  printf("Closest object approach: %f \n",d_min);

	  write_Params(n,Time,kP,bP);

	  write_Map(i,pts);

	  // write data to .txt files for gnuplot
	  write_Freq(n,Time,sample_freq);
	  write_Position(n,Time,x_e,y_e,z_e,z_o);
	  write_Velocity(n,Time,u_e,v_e,w_e,u_c,v_c,w_c);
	  //write_Vel_Int(n,Time,Ie_u,Ie_v,Ie_w);
	  write_Vel_Int(n,Time,u_d,v_d,w_d);
	  write_Acceleration(n,Time,ud,vd,wd);
	  write_Angles(n+1,Time,phi,theta,psi,phi_d,theta_d,psi_c);
	  write_Angle_Errors(n,Time,e_phi,e_the,e_psi);
	  write_Rates(n,Time,p,q,r,p_d,q_d,r_d);
	  write_Rate_Errors(n,Time,e_p,e_q,e_r);
	  write_Momentum(n,Time,Hx,Hy,Hz,Hp);
	  write_Inputs(n,Time,T,t1,t2,t3,om1,om2,om3,om4);
	  write_Pos_Obj(n,Time,px,py,pz,dist);

	  exit(1);
	  break;
	}
      }

      // generate reference commands
      u_c[n+1]=u_c[n]+ud_cp*dt;
      ud_c=ud_cp+u2_cp*dt;
      u2_c=u2_cp+u3_cp*dt;
      u3_c=u3_cp+u4_cp*dt;
      u4_c=-a3*u3_c-a2*u2_c-a1*ud_c-a0*(u_c[n+1]-u_d[n+1]);

      // set previous reference commands (used for solving refrence model with euler method)
      ud_cp=ud_c; u2_cp=u2_c; u3_cp=u3_c; u4_cp=u4_c;

      v_c[n+1]=v_c[n]+vd_cp*dt;
      vd_c=vd_cp+v2_cp*dt;
      v2_c=v2_cp+v3_cp*dt;
      v3_c=v3_cp+v4_cp*dt;
      v4_c=-a3*v3_c-a2*v2_c-a1*vd_c-a0*(v_c[n+1]-v_d[n+1]);

      vd_cp=vd_c; v2_cp=v2_c; v3_cp=v3_c; v4_cp=v4_c;

      w_c[n+1]=w_c[n]+wd_cp*dt;
      wd_c=wd_cp+w2_cp*dt;
      w2_c=w2_cp+w3_cp*dt;
      w3_c=w3_cp+w4_cp*dt;
      //w4_c=-aw3*w3_c-aw2*w2_c-aw1*wd_c-aw0*(w_c[n+1]-w_d[n+1]);
      w4_c=-a3*w3_c-a2*w2_c-a1*wd_c-a0*(w_c[n+1]-w_d[n+1]);

      wd_cp=wd_c; w2_cp=w2_c; w3_cp=w3_c; w4_cp=w4_c;

      psi_c[n+1]=psi_c[n]+psid_cp*dt;
      psid_c=psid_cp+psi2_cp*dt;
      psi2_c=psi2_cp+psi3_cp*dt;
      psi3_c=-b2*psi2_c-b1*psid_c-b0*(psi_c[n+1]-psi_d);

      psid_cp=psid_c; psi2_cp=psi2_c; psi3_cp=psi3_c;

      /** Velocity Control **/

      // velocity errors
      e_u[n+1]=u_e[n+1]-u_c[n+1];
      e_v[n+1]=v_e[n+1]-v_c[n+1];
      e_w[n+1]=w_e[n+1]-w_c[n+1];

      // integral of velocity errors
      Ie_u[n+1]=Ie_u[n]+(e_u[n+1]+e_u[n])*dt/2;
      Ie_v[n+1]=Ie_v[n]+(e_v[n+1]+e_v[n])*dt/2;
      Ie_w[n+1]=Ie_w[n]+(e_w[n+1]+e_w[n])*dt/2;

      alu = ud_c-ku*e_u[n+1]-kiu*Ie_u[n+1];
      alv = vd_c-kv*e_v[n+1]-kiv*Ie_v[n+1];
      alw = wd_c-kw*e_w[n+1]-kiw*Ie_w[n+1]-g;
      //alw = wd_c-kw*e_w[n+1]-(z_e[n+1]-z_g)-g;

      alud = u2_c-ku*(ud[n+1]-ud_c)-kiu*e_u[n+1];
      alvd = v2_c-kv*(vd[n+1]-vd_c)-kiv*e_v[n+1];
      alwd = w2_c-kw*(wd[n+1]-wd_c)-kiw*e_w[n+1];

      // compute thrust
      T[n+1] = -m*alw/cph/cth;
      //if (z_e[n+1]>-.301) T[n+1]=.98*T[n+1];
      if (T[n+1]<0) T[n+1] = 0;
      if (T[n+1]>14) T[n+1] = 14;

      // zero attitude if VELOCITY_CTRL not defined
      theta_d[n+1] = 0;
      phi_d[n+1] = 0;
      thetad_d = 0;
      phid_d = 0;
      theta2_d = 0;
      phi2_d = 0;

#ifdef VELOCITY_CTRL

      Td = m*(-w2_c+kw*(wd[n+1]-wd_c)+kiw*e_w[n+1])/cph/cth
	  +T[n+1]*(phid*tan(phi[n+1])+thetad*tth);
//      Td = m*(-w2_c+kw*(wd[n+1]-wd_c)+w_e[n+1])/cph/cth
  //        +T[n+1]*(phid*tan(phi[n+1])+thetad*tth);

      // Calculate inertial jerks (assholes tbh)
      u2 = -Td/m*O(0,2)+T[n+1]/m*(phid*O(0,1)-thetad*cph*cth*cps+psid*O(1,2));
	/*-lambda*(om2[n]+om4[n]+om1[n]+om3[n])*(ud[n+1]*cps*cth-u_e[n+1]*psid*sps*cth-u_e[n+1]*cps*thetad*sth-wd[n+1]*sth-w_e[n+1]*thetad*cth+vd[n+1]*cth*sps-v_e[n+1]*thetad*sth*sps+v_e[n+1]*cth*psid*cps);*/
      v2 = -Td/m*O(1,2)+T[n+1]/m*(phid*O(1,1)-thetad*cph*cth*sps-psid*O(0,2));
	/*-lambda*(om2[n]+om4[n]+om1[n]+om3[n])*(vd[n+1]*(cph*cps+sph*sps*sth)+v_e[n+1]*(-phid*sph*cps-psid*cph*sps+phid*cph*sps*sth+psid*sph*cps*sth+thetad*sph*sps*cth)-ud[n+1]*(cph*sps-cps*sph*sth)-u_e[n+1]*(-phid*sph*sps+psid*cph*cps+psid*sps*sph*sth-phid*cps*cph*sth-thetad*cps*sph*cth)+wd[n+1]*cth*sph-w_e[n+1]*thetad*sth*sph)+w_e[n+1]*phid*cth*cph;*/
      w2 = w2_c-kw*(wd[n+1]-wd_c)-kiw*e_w[n+1];

      alu2 = u3_c-ku*(u2-u2_c)-kiu*(ud[n+1]-ud_c);
      alv2 = v3_c-kv*(v2-v2_c)-kiv*(vd[n+1]-vd_c);
      alw2 = w3_c-kw*(w2-w2_c)-kiw*(wd[n+1]-wd_c);

      // math equations
      beth = (alu*cps+alv*sps)/alw;
      theta_d[n+1] = atan(beth);

      cthd = cos(theta_d[n+1]); sthd = sin(theta_d[n+1]);

      beph = (alu*sps-alv*cps)*cthd/alw;
      phi_d[n+1] = atan(beph);

      bthd = (cps*(alud+alv*psid)+sps*(alvd-alu*psid)-beth*alwd)/alw;
      thetad_d = bthd/(1+beth*beth);

      bphd = (sps*(cthd*(alud+alv*psid)-thetad_d*sthd*alu)
	      +cps*(cthd*(alu*psid-alvd)+thetad_d*sthd*alv)
	      -beph*alwd)/alw;

      phid_d = bphd/(1+beph*beph);

      bth2 = (cps*(alu2+2*alvd*psid-alu*psid*psid+alv*psi2)
	      +sps*(alv2-2*alud*psid-alv*psid*psid-alu*psi2)
	      -2*bthd*alwd-beth*alw2)/alw;

      theta2_d = bth2/(1+beth*beth)-2*beth*bthd*bthd/pow(1+beth*beth,2);

      bph2 = (sps*((alu2+alv*psi2+2*alvd*psid-alu*(pow(thetad_d,2)+pow(psid,2)))*cthd
		   -(2*thetad_d*(alud+alv*psid)+alu*theta2_d)*sthd)
	      +cps*((alu*psi2-alv2+2*alud*psid+alv*(pow(thetad_d,2)+pow(psid,2)))*cthd
		    +(2*thetad_d*(alvd-alu*psid)+alv*theta2_d)*sthd)-2*bphd*alwd-beph*alw2)/alw;

      phi2_d = bph2/(1+beph*beph)-2*beph*bphd*bphd/pow(1+beph*beph,2);

#endif

      // angle errors
      e_phi[n+1] = phi[n+1]-phi_d[n+1];
      e_the[n+1] = theta[n+1]-theta_d[n+1];
      e_psi[n+1] = psi[n+1]-psi_c[n+1];

      // integral of angle errors
      Ie_ph[n+1] = Ie_ph[n]+(e_phi[n+1]+e_phi[n])*dt/2;
      Ie_th[n+1] = Ie_th[n]+(e_the[n+1]+e_the[n])*dt/2;
      Ie_ps[n+1] = Ie_ps[n]+(e_psi[n+1]+e_psi[n])*dt/2;

      // desired rates
      p_d[n+1] = phid_d-kphi*e_phi[n+1]-kiph*Ie_ph[n+1]
	-(psid_c-kpsi*e_psi[n+1]-kips*Ie_ps[n+1])*sth;
      q_d[n+1] = (thetad_d-ktheta*e_the[n+1]-kith*Ie_th[n+1])*cph
	+(psid_c-kpsi*e_psi[n+1]-kips*Ie_ps[n+1])*sph*cth;
      r_d[n+1] = -(thetad_d-ktheta*e_the[n+1]-kith*Ie_th[n+1])*sph
	+(psid_c-kpsi*e_psi[n+1]-kips*Ie_ps[n+1])*cph*cth;

      // desired rate derivatives
      pd_d = phi2_d-kphi*(phid-phid_d)-kiph*e_phi[n+1]
	-(psi2_c-kpsi*(psid-psid_c)-kips*e_psi[n+1])*sth
	-(psid_c-kpsi*e_psi[n+1]-kips*Ie_ps[n+1])*thetad*cth;
      qd_d = (theta2_d-ktheta*(thetad-thetad_d)-kith*e_the[n+1])*cph
	-(thetad_d-ktheta*e_the[n+1]-kith*Ie_th[n+1])*phid*sph
	+(psi2_c-kpsi*(psid-psid_c)-kips*e_psi[n+1])*sph*cth
	+(psid_c-kpsi*e_psi[n+1]-kips*Ie_ps[n+1])*(phid*cph*cth-thetad*sph*sth);
      rd_d = -(theta2_d-ktheta*(thetad-thetad_d)-kith*e_the[n+1])*sph
	-(thetad_d-ktheta*e_the[n+1]-kith*Ie_th[n+1])*phid*cph
	+(psi2_c-kpsi*(psid-psid_c)-kips*e_psi[n+1])*cph*cth
	-(psid_c-kpsi*e_psi[n+1]-kips*Ie_ps[n+1])*(phid*sph*cth+thetad*cph*sth);

      // rate errors
      e_p[n+1] = p[n+1]-p_d[n+1];
      e_q[n+1] = q[n+1]-q_d[n+1];
      e_r[n+1] = r[n+1]-r_d[n+1];

      // integral of rate errors
      Ie_p[n+1] = Ie_p[n]+(e_p[n+1]+e_p[n])*dt/2;
      Ie_q[n+1] = Ie_q[n]+(e_q[n+1]+e_q[n])*dt/2;
      Ie_r[n+1] = Ie_r[n]+(e_r[n+1]+e_r[n])*dt/2;

      // torques inputs
      t1[n+1] = (pd_d-kp*e_p[n+1]-kip*Ie_p[n+1]-e_phi[n+1])*Ixx;//+q[n+1]*r[n+1]*(Izz-Iyy)+q[n+1]*Hp[n];
      t2[n+1] = (qd_d-kq*e_q[n+1]-kiq*Ie_q[n+1]-e_phi[n+1]*sph*tth
		 -e_the[n+1]*cph-e_psi[n+1]*sph/cth)*Iyy;//-p[n+1]*r[n+1]*(Izz-Ixx)-p[n+1]*Hp[n];
      t3[n+1] = (rd_d-kr*e_r[n+1]-kir*Ie_r[n+1]-e_phi[n+1]*cph*tth
		 +e_the[n+1]*sph-e_psi[n+1]*cph/cth)*Izz;//+p[n+1]*q[n+1]*(Iyy-Ixx);
      /*
+q[n+1]*r[n+1]*(Izz-Iyy)+q[n+1]*Hp[n]
-p[n+1]*r[n+1]*(Izz-Ixx)-p[n+1]*Hp[n]
+p[n+1]*q[n+1]*(Iyy-Ixx)
      t1[n+1] = (pd_d-kp*(p[n+1]-p_d[n+1]))*Ixx+q[n+1]*r[n+1]*(Izz-Iyy)+q[n+1]*Hp[n];
      t2[n+1] = (qd_d-kq*(q[n+1]-q_d[n+1]))*Iyy-p[n+1]*r[n+1]*(Izz-Ixx)-p[n+1]*Hp[n];
      t3[n+1] = (rd_d-kr*(r[n+1]-r_d[n+1]))*Izz;

      t1[n+1] = (pd_d-kp*(p[n+1]-p_d[n+1]))*Ixx+q[n+1]*Hp[n];
      t2[n+1] = (qd_d-kq*(q[n+1]-q_d[n+1]))*Iyy-p[n+1]*Hp[n];
      t3[n+1] = (rd_d-kr*(r[n+1]-r_d[n+1]))*Izz;
      */

      // set sane torque limits (.4 N*m)
      if (abs(t1[n+1])>.4) t1[n+1] = sgn(t1[n+1])*.4;
      if (abs(t2[n+1])>.4) t2[n+1] = sgn(t2[n+1])*.4;
      if (abs(t3[n+1])>.4) t3[n+1] = sgn(t3[n+1])*.4;

#ifdef ADPT_K
      if (z_e[n+1]<-.301) {

	// only do this if integral gain kiw=0;
	k=k-(8.0e-9f)*sigma(e_w[n+1],.03);
	//if (k<1.12*pow(10,-6)) k=1.12*pow(10,-6);
	if (k<.8*kP[0]) k=.8*kP[0];
	if (k>1.2*kP[0]) k=1.2*kP[0];

	kP[n+1]=k;
	//bP[n+1]=b;
      } else {
	kP[n+1]=kP[n];
	//bP[n+1]=bP[n];
      }
#endif

      // propeller mixing to implement thrust and torques
      om1[n+1] = T[n+1]/(4.0*k)+t1[n+1]/(4.0*k*ls)+t2[n+1]/(4.0*k*ll)-t3[n+1]/(4.0*b);
      om2[n+1] = T[n+1]/(4.0*k)-t1[n+1]/(4.0*k*ls)+t2[n+1]/(4.0*k*ll)+t3[n+1]/(4.0*b);
      om3[n+1] = T[n+1]/(4.0*k)-t1[n+1]/(4.0*k*ls)-t2[n+1]/(4.0*k*ll)-t3[n+1]/(4.0*b);
      om4[n+1] = T[n+1]/(4.0*k)+t1[n+1]/(4.0*k*ls)-t2[n+1]/(4.0*k*ll)+t3[n+1]/(4.0*b);

      om1[n+1] = sqrt(om1[n+1]); om2[n+1] = sqrt(om2[n+1]);
      om3[n+1] = sqrt(om3[n+1]); om4[n+1] = sqrt(om4[n+1]);

      // enforce propeller speed limits
      if (om1[n+1]<wmin) om1[n+1]=wmin;
      if (om2[n+1]<wmin) om2[n+1]=wmin;
      if (om3[n+1]<wmin) om3[n+1]=wmin;
      if (om4[n+1]<wmin) om4[n+1]=wmin;

      if (om1[n+1]>wmax) om1[n+1]=wmax;
      if (om2[n+1]>wmax) om2[n+1]=wmax;
      if (om3[n+1]>wmax) om3[n+1]=wmax;
      if (om4[n+1]>wmax) om4[n+1]=wmax;

      // protect math equations from nan
      if (isnan(om1[n+1])) om1[n+1]=om1[n];
      if (isnan(om2[n+1])) om2[n+1]=om2[n];
      if (isnan(om3[n+1])) om3[n+1]=om3[n];
      if (isnan(om4[n+1])) om4[n+1]=om4[n];

      // Angular momentum of props
      Hp[n+1]=(om1[n+1]+om3[n+1]-om2[n+1]-om4[n+1])*Ip;

      // compute pwm duty cycle based on effective pwm range and min/max propeller speeds
      duty1 = (int)(rng*(om1[n+1]-wmin)/(wmax-wmin)+.5f);
      duty2 = (int)(rng*(om2[n+1]-wmin)/(wmax-wmin)+.5f);
      duty3 = (int)(rng*(om3[n+1]-wmin)/(wmax-wmin)+.5f);
      duty4 = (int)(rng*(om4[n+1]-wmin)/(wmax-wmin)+.5f);

      // write pwm control values to PCA9685
#ifdef FLY
      pwm->ledOffTime(15,min1+duty1);
      pwm->ledOffTime(1,min2+duty2);
      pwm->ledOffTime(14,min3+duty3);
      pwm->ledOffTime(0,min4+duty4);
#endif
      n = n+1; // data array increment

      if (n==size-1) { // reset arrays (previous 6000 data points in all arrays are lost)
	x_e[0] = x_e[n]; y_e[0] = y_e[n]; z_e[0] = z_e[n];
	u_e[0] = u_e[n]; v_e[0] = v_e[n]; w_e[0] = w_e[n];
	ud[0] = ud[n]; vd[0] = vd[n]; wd[0] = wd[n];
	u_c[0] = u_c[n]; v_c[0] = v_c[n]; w_c[0] = w_c[n];
	u_d[0] = u_d[n]; v_d[0] = v_d[n]; w_d[0] = w_d[n];

	phi[0] = phi[n]; theta[0] = theta[n]; psi[0] = psi[n];
	p[0] = p[n]; q[0] = q[n]; r[0] = r[n];
	qd[0] = qd[n]; rd[0] = rd[n];

	Hx[0] = Hx[n]; Hy[0] = Hy[n]; Hz[0] = Hz[n];
	Hp[0] = Hp[n];

	e_u[0] = e_u[n]; e_v[0] = e_v[n]; e_w[0] = e_w[n];
	e_phi[0] = e_phi[n]; e_the[0] = e_the[n]; e_psi[0]=e_psi[n];
       	e_p[0] = e_p[n]; e_q[0] = e_q[n]; e_r[0]=e_r[n];

	Ie_u[0]=Ie_u[n];Ie_v[0]=Ie_v[n]; Ie_w[0]=Ie_w[n];
	Ie_ph[0]=Ie_ph[n]; Ie_th[0]=Ie_th[n]; Ie_ps[0]=Ie_ps[n];
	Ie_p[0]=Ie_p[n]; Ie_q[0]=Ie_q[n]; Ie_r[0]=Ie_r[n];

	t1[0]=t1[n]; t2[0]=t2[n]; t3[0]=t3[n]; T[0]=T[n];

	om1[0] = om1[n]; om2[0] = om2[n];
	om3[0] = om3[n]; om4[0] = om4[n];
	n = 0; // reset index

	printf("Indices zero'd \n");
	cout.flush();
      }
    }
  }
}
