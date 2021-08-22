const double defaultSpeed = 6*M_PI;


const double g = 9.81;
double massPole = 49 / 1000;
double lengthPend = 0.244;
double massCart = (80+115+75) / 1000.0;
double cartWidth = 0.07;
double trackLength = 0.62;
const double LEFTMOST = -1.0 * trackLength / 2.0 + cartWidth / 2.0;
const double RIGHTMOST = -1.0 * LEFTMOST;
const double CENTER = 0.0;

double ke = 0.5; // Energy Gain
double kx_P = 5; // Position P gain
double kx_D = 5; // Position D gain

double K[4] = {1, 1, 1, 1}; // LQR controller gains, determined by MATLAB

double stepAngle = 1.8;
double tPulse = 3.4e-6;
double deltat = 0.005;
double currentVelocity = 0;
double gearRadius = (12.9/2.0)/1000.0;

struct State {
  double x;
  double x_dot;
  double theta;
  double theta_dot;
};
