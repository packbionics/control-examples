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
      driveMotor(state, defaultSpeed);
    }
  }
  else if (state->x > x) {
    while (state->x > x) {
      driveMotor(state, -1.0 * defaultSpeed);
    }
  }
  state->x_dot = 0.0;
}

/**
 * Drives motor to desired speed given angular velocity.
 */
void driveMotor( State *state, double phiDot ) 
{

  //no movement check
  if( abs( phiDot ) < 0.01 ) {
    return;
  }

  if (phiDot > 35) {
    phiDot = 35;
  }
  if (phiDot < -35) {
    phiDot = -35;
  }
  
  double pulseRate = (abs(phiDot) * 180) / (stepAngle * M_PI);
  double motorDelay = 1 / pulseRate - tPulse;

  Serial.print("x:");
  Serial.print(state->x);
  Serial.print("\n");

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

  for (int i=0; i<10; i++) {
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(1e6*motorDelay/2); 
      state->x = state->x + dx;
      digitalWrite(stepPin,LOW); 
      delayMicroseconds(1e6*motorDelay/2); 
  }
}

/**
 * Calculates velocity and drives motor to that velocity given desired acceleration.
 */
void acheiveAcc( State *state, double acc )
{
  double targetVelocity = state->x_dot + acc*deltat;
  double phi_dot = targetVelocity / gearRadius;
  driveMotor(state, phi_dot);
}

double theta_distance(double theta, double target) {
  return (fmod(theta,  (2*M_PI))) - target;
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
