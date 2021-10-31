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

/**
 * Moves to desired location at a nominal speed
 */
void moveTo( State *state, double x) {

  if (abs(state->x - x) < 0.02) {
    return;
  }
  
  if (state->x < x) {
    while (state->x < x) {
      driveMotor(state, defaultSpeed, tDrive);
    }
  }
  else if (state->x > x) {
    while (state->x > x) {
      driveMotor(state, -1.0 * defaultSpeed, tDrive);
    }
  }
  state->x_dot = 0.0;
}

/**
 * Drives motor to desired speed given angular velocity.
 */
void driveMotor( State *state, double phiDot, double duration) 
{

  //no movement check
  if( abs( phiDot ) < 0.01 ) {
    return;
  }

  if (phiDot > 40) {
    phiDot = 40;
  }
  if (phiDot < -40) {
    phiDot = -40;
  }

  if (abs(phiDot) < minPhiDot) {
    int sign;
    if (phiDot < 0) {
      sign = -1;
    }
    else {
      sign = 1;
    }
    phiDot = sign * phiDot * minPhiDot;
  }
  
  double pulseRate = (abs(phiDot) * 180) / (stepAngle * M_PI);
  double motorDelay = 1 / pulseRate - tPulse;

  double dx;
  double dt = tPulse + motorDelay;

  // Clockwise
  if (phiDot > 0) {
    digitalWrite(dirPin, HIGH);
    dx = (stepAngle / 360.0) * gearRadius * M_PI * 2.0;
  }
  // Counterclockwise
  else {
    digitalWrite(dirPin, LOW);
    dx = - (stepAngle / 360.0) * gearRadius * M_PI * 2.0;
  }

  state->x_dot = dx / dt;

  int nPulses = floor(duration / dt); 
  
  for (int i=0; i<nPulses; i++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(1e6*motorDelay/2); 
    state->x = state->x + dx;
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(1e6*motorDelay/2); 
    if (state->x >= RIGHTMOST || state->x <= LEFTMOST) {
      break;
    }
  }
}

/**
 * Calculates velocity and drives motor to that velocity given desired acceleration.
 */
void acheiveAcc( State *state, double acc )
{
  double duration = tDrive;//min(max(abs(acc)/100,0.01), 0.1);
  //Serial.print("DURATION\n");
  //Serial.print(duration);
  //Serial.print("\n");
  double targetVelocity = state->x_dot + acc*(deltat + duration);
  //Serial.println(targetVelocity);
  double phi_dot = targetVelocity / gearRadius;

  driveMotor(state, phi_dot, duration);
}

double theta_distance(double theta, double target) {
  double diff = theta - target;
  double phi = fmod(abs(diff), 2*M_PI);
  
  double distance;

  if (phi > M_PI) {
    distance = 2*M_PI - phi;
  }
  else {
    distance = phi;
  }

  if (((diff <= 0) && (diff >= -M_PI)) || ((diff >= M_PI) && (diff <= 2*M_PI))) {
    distance = -1*distance;
  }
  else {
    distance = 1*distance;
  }
  
  return distance;
}

/**
 * Returns current total energy of pendulum given sytem's state.
 */
double energy(State state) {
  double U = -1*massPole*g*lengthPend*cos(state.theta);
  double K = 0.5*(massPole*sq(lengthPend))*sq(state.theta_dot);
  return U + K;
}

/**
 * Returns acceleration of cart needed to get pendulum to desired angle given system's state.
 */
double swingup(State state) {
  //desired energy; pendulum at angle pi

  double Ed = massPole*g*lengthPend;
  
  double E = energy(state);
  double Ediff = E - Ed;
  
  double c = cos(state.theta);
  double s = sin(state.theta);
  double t = tan(state.theta);

  double acc = ke*state.theta_dot*c*Ediff - kx_P*state.x - kx_D*state.x_dot;
  
  return acc;
}

// acc = (f + massPole*lengthPend*state.theta_dot**2*s + g*s*c*massPole) / (massPole+massCart-massPole*sq(c));

double upright_lqr(State state) {
  double c = cos(state.theta);
  double s = sin(state.theta);
  double theta_diff = theta_distance(state.theta, M_PI);
  double f = K[0]*state.x + K[1]*theta_diff + K[2]*state.x_dot + K[3]*state.theta_dot;
  double acc = (f + massPole*lengthPend*sq(state.theta_dot)*s + g*s*c*massPole) / (massPole+massCart-massPole*sq(c));
  return -1 * acc;
}
