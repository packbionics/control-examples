void moveStepper(int speedInput) {
  int delayTime;
  if (speedInput > 0) {
    digitalWrite(dirPin, HIGH);
    delayTime = defaultSpeed / abs(speedInput);
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(delayTime); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(delayTime); 
  } else if (speedInput < 0) {
    digitalWrite(dirPin, LOW);
    delayTime = defaultSpeed / abs(speedInput);
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(delayTime); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(delayTime); 
  } else {
    digitalWrite(stepPin,LOW); 
  }
}

double theta_distance(double theta, double target) {
  return (theta%(2*M_1_PI)) - target;
}

double energy(State state) {
  double U = -1*masspole*g*lengthPend*cos(state.theta);
  double E = 0.5*(masspole*sq(lengthPend))*sq(state.theta_dot) + U;
  return E;
}

double swingup(State state) {
  double Ed = massple*g*lengthPend;
  double E = energy(state);
  double Ediff = E - Ed;
  double c = cos(state.theta);
  double s = sin(state.theta);
  double t = tan(state.theta);

  double acc = ke*state.theta_dot*c*Ediff - kx_P*state.x - kx_D*state.x_dot;

  return acc;
}

// acc = (f + masspole*lengthPend*state.theta_dot**2*s + g*s*c*masspole) / (masspole+masscart-masspole*sq(c));

double upright_lqr(State state) {
  double theta_diff = theta_distance(state.theta, M_1_PI);
  double f = K[0]*state.x + K[1]*theta_diff + K[2]*state.x_dot + K[3]*state.theta_dot;
  double acc = (f + masspole*lengthPend*state.theta_dot**2*s + g*s*c*masspole) / (masspole+masscart-masspole*sq(c));
  return -1 * acc;
}
