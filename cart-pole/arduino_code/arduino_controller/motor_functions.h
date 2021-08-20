const int defaultSpeed = 1000;

const double g = 9.81;
double masspole = 1;
double lengthPend = 1;
double masscart = 1;

double ke = 0.5; // Energy Gain
double kx_P = 5; // Position P gain
double kx_D = 5; // Position D gain

double K[4] = {1, 1, 1, 1}; // LQR controller gains, determined by MATLAB

double stepAngle = 1.8;
double tPulse = 3.4e-6;
double deltat = 0.01;
double currentVelocity = 0;

struct State {
  double x;
  double x_dot;
  double theta;
  double theta_dot;
};