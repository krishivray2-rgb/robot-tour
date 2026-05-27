#include <TektiteRotEv.h>

RotEv rotev;

const float WHEEL_DIAMETER_CM   = 6.0325;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_CM * PI;
const float TRACK_WIDTH_CM      = 13.5;
const float DOWEL_OFFSET_CM     = 4.117;
const float TICKS_PER_REV       = 360.0;
const float CM_PER_TICK         = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
const float GRID_SIZE_CM        = 50.0;

const int BASE_PWM  = 120;
const int MIN_PWM   = 50;
const int MAX_PWM   = 200;
const int TURN_PWM  = 70;

float Kp_straight = 0.75;
float Kp_turn     = 0.45;
const float TURN_TOLERANCE_DEG = 0.4;

float Ki_straight = 0.0;
float Kd_straight = 0.0;
float Ki_turn     = 0.0;
float Kd_turn     = 0.0084;

float gyroBias = 0.0f;

float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;

long continuousL_ticks = 0, continuousR_ticks = 0;
float prevL_angle = 0.0, prevR_angle = 0.0;
long revL = 0, revR = 0;

long prevL_ticks = 0, prevR_ticks = 0;
unsigned long prevOdomTime = 0;

void setMotorL(int pwm) {
  float duty = constrain((float)pwm / 255.0f, -1.0f, 1.0f);
  rotev.motorWrite1(-duty);
}

void setMotorR(int pwm) {
  float duty = constrain((float)pwm / 255.0f, -1.0f, 1.0f);
  rotev.motorWrite2(duty);
}

void stopMotors() {
  rotev.motorWrite1(0.0f);
  rotev.motorWrite2(0.0f);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro...");
  float total = 0.0f;
  const int samples = 2000;
  for (int i = 0; i < samples; i++) {
    total += rotev.readYawRate();
    delayMicroseconds(1000);
  }
  gyroBias = total / (float)samples;
  Serial.print("Gyro bias: ");
  Serial.println(gyroBias, 6);
}

void updateEncoders() {
  float curR_angle = rotev.enc2AngleDegrees();
  float curL_raw   = rotev.enc1AngleDegrees();
  float curL_angle = fmod(360.0f - curL_raw, 360.0f);

  if (curL_angle - prevL_angle < -180.0) revL++;
  else if (curL_angle - prevL_angle > 180.0) revL--;

  if (curR_angle - prevR_angle < -180.0) revR++;
  else if (curR_angle - prevR_angle > 180.0) revR--;

  prevL_angle = curL_angle;
  prevR_angle = curR_angle;

  continuousL_ticks = (revL * 360) + (long)curL_angle;
  continuousR_ticks = (revR * 360) + (long)curR_angle;
}

void updateOdometry() {
  updateEncoders();

  long dL = continuousL_ticks - prevL_ticks;
  long dR = continuousR_ticks - prevR_ticks;
  prevL_ticks = continuousL_ticks;
  prevR_ticks = continuousR_ticks;

  float distL = dL * CM_PER_TICK;
  float distR = dR * CM_PER_TICK;
  float distCenter = (distL + distR) / 2.0;

  unsigned long nowMicros = micros();
  float dt = (nowMicros - prevOdomTime) / 1000000.0f;
  if (dt < 0.0001f) dt = 0.0001f;
  prevOdomTime = nowMicros;

  float yawRate = rotev.readYawRate() - gyroBias;
  float dThetaRad = yawRate * dt;

  float thetaRad = odom_theta * DEG_TO_RAD;
  odom_x     += distCenter * cos(thetaRad + dThetaRad / 2.0);
  odom_y     += distCenter * sin(thetaRad + dThetaRad / 2.0);
  odom_theta += dThetaRad * RAD_TO_DEG;

  while (odom_theta >  180.0) odom_theta -= 360.0;
  while (odom_theta < -180.0) odom_theta += 360.0;
}

void resetOdometry() {
  prevR_angle = rotev.enc2AngleDegrees();
  prevL_angle = fmod(360.0f - rotev.enc1AngleDegrees(), 360.0f);
  revL = 0; revR = 0;
  continuousL_ticks = 0; continuousR_ticks = 0;
  prevL_ticks = 0; prevR_ticks = 0;
  prevOdomTime = micros();
  odom_x = 0.0; odom_y = 0.0; odom_theta = 0.0;
}

void goForward(float distanceCM) {
  updateEncoders();
  long startL = continuousL_ticks;
  long startR = continuousR_ticks;

  float startTheta = odom_theta * DEG_TO_RAD;
  float targetX = odom_x + distanceCM * cos(startTheta);
  float targetY = odom_y + distanceCM * sin(startTheta);
  float totalDist = fabs(distanceCM);

  const float MAX_DUTY    = 0.40f;
  const float CRUISE_DUTY = 0.35f;
  const float ACCEL_TIME  = 0.6f;
  const float baseP       = 0.035f;
  const float D_axial     = 0.012f;
  const float FF_axial    = 0.12f;
  const float P_heading   = 1.5f;
  const float D_heading   = 0.2f;

  unsigned long startTime = micros();
  unsigned long prevLoopTime = micros();

  float linVel = 0.0f;
  long prevL = continuousL_ticks;
  long prevR = continuousR_ticks;

  while (true) {
    updateOdometry();

    unsigned long nowMicros = micros();
    float dt = (nowMicros - prevLoopTime) / 1000000.0f;
    if (dt < 0.0001f) dt = 0.0001f;
    prevLoopTime = nowMicros;

    long curL = continuousL_ticks;
    long curR = continuousR_ticks;
    float distThisLoop = ((float)(curL - prevL) + (float)(curR - prevR)) / 2.0f * CM_PER_TICK;
    prevL = curL;
    prevR = curR;
    float rawVel = distThisLoop / dt;
    linVel = linVel * 0.7f + rawVel * 0.3f;

    long dL = continuousL_ticks - startL;
    long dR = continuousR_ticks - startR;
    float traveled = fabs(((float)dL + (float)dR) / 2.0f * CM_PER_TICK);
    float distRemaining = totalDist - traveled;

    if (fabs(distRemaining) < 0.3f && fabs(linVel) < 0.5f) break;
    if (distRemaining < -5.0f) break;

    float elapsed = (nowMicros - startTime) / 1000000.0f;
    float rampFactor = (elapsed < ACCEL_TIME) ? (elapsed / ACCEL_TIME) : 1.0f;

    float axialPow;
    if (distRemaining > 10.0f) {
      axialPow = CRUISE_DUTY * rampFactor;
    } else {
      float gainMult = fminf(sqrtf(0.015f / fmaxf(fabs(distRemaining), 0.015f)), 4.0f);
      float currentP = baseP * gainMult;
      axialPow = currentP * distRemaining - D_axial * linVel;
      if (distRemaining > 0.0f) axialPow += FF_axial;
      else axialPow -= FF_axial;
    }

    float thetaRad = odom_theta * DEG_TO_RAD;
    float angleToTarget = atan2f(targetY - odom_y, targetX - odom_x);
    float headingError = angleToTarget - thetaRad;
    while (headingError >  M_PI) headingError -= 2.0f * M_PI;
    while (headingError < -M_PI) headingError += 2.0f * M_PI;

    float currentYawRate = rotev.readYawRate() - gyroBias;
    float omega = P_heading * headingError - D_heading * currentYawRate;

    if (fabs(omega) > MAX_DUTY) omega = (omega > 0) ? MAX_DUTY : -MAX_DUTY;
    float maxAxial = MAX_DUTY - fabs(omega);
    axialPow = constrain(axialPow, -maxAxial, maxAxial);

    float dutyL = axialPow - omega;
    float dutyR = axialPow + omega;
    dutyL = constrain(dutyL, -MAX_DUTY, MAX_DUTY);
    dutyR = constrain(dutyR, -MAX_DUTY, MAX_DUTY);

    rotev.motorWrite1(-dutyL);
    rotev.motorWrite2(dutyR);
  }

  stopMotors();
  delay(100);
}

void turnLeft90() {
  updateOdometry();
  float targetHeading = odom_theta + 90.0;

  while (targetHeading >  180.0) targetHeading -= 360.0;
  while (targetHeading < -180.0) targetHeading += 360.0;

  float integral = 0.0;
  float prevError = 0.0;
  unsigned long prevTime = millis();

  unsigned long turnStart = millis();
  unsigned long settledStart = 0;
  bool settled = false;

  while (true) {
    updateOdometry();

    float error = targetHeading - odom_theta;
    while (error >  180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    if (fabs(error) < TURN_TOLERANCE_DEG) {
      if (!settled) { settled = true; settledStart = millis(); }
      if (millis() - settledStart > 150) break;
    } else {
      settled = false;
    }

    if (millis() - turnStart > 5000) break;

    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    prevTime = now;

    integral += error * dt;
    integral = constrain(integral, -100.0, 100.0);

    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = Kp_turn * error + Ki_turn * integral + Kd_turn * derivative;

    int pwm = constrain((int)fabs(output), MIN_PWM, TURN_PWM);

    if (output > 0) {
      setMotorL(-pwm); setMotorR(pwm);
    } else {
      setMotorL(pwm);  setMotorR(-pwm);
    }

    delay(5);
  }

  stopMotors();
  delay(150);
}

void printOdometry() {
  Serial.print("x="); Serial.print(odom_x, 1);
  Serial.print(" y="); Serial.print(odom_y, 1);
  Serial.print(" theta="); Serial.print(odom_theta, 1);
  Serial.print(" EncL="); Serial.print(continuousL_ticks);
  Serial.print(" EncR="); Serial.println(continuousR_ticks);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  rotev.begin();
  rotev.motorEnable(true);
  stopMotors();

  calibrateGyro();

  Serial.println("Ready. Press GO.");
  while (!rotev.goButtonPressed()) { delay(10); }

  resetOdometry();
  for (int i = 0; i < 20; i++) {
    updateEncoders();
    delay(5);
  }
  resetOdometry();

  delay(500);
  Serial.println("GO!");
}

void loop() {
  delay(2000);
  goForward(50.0);

  printOdometry();
  Serial.println("Done.");
  while (true) { delay(1000); }
}
