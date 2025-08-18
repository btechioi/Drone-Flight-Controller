/*
  ESP32 Quadcopter FC (GY-87) — Final cleaned version
  - Madgwick/Mahony AHRS (compile-time selectable)
  - Attitude PID (roll/pitch angle, yaw rate), altitude-hold (baro+accZ EKF)
  - ESP-NOW RC + telemetry (includes AutoTune guided UI)
  - Non-blocking AutoTune (Twiddle/coord-descent hybrid), safety-guarded
  - 4x ESC via LEDC
  - EKF tuning guidance (Q,R), sensor init checks, RC failsafe, battery failsafe
  NOTE: bench test WITHOUT PROPS. Verify motor mapping, ESC endpoints, and sensor orientation.
*/

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>

// =================== Filter selection ===================
#define USE_MADGWICK
// #define USE_MAHONY

#if defined(USE_MADGWICK)
  #include <MadgwickAHRS.h>
  Madgwick filter;           // provides q0,q1,q2,q3
#elif defined(USE_MAHONY)
  #include <MahonyAHRS.h>
  Mahony filter;
#else
  #error "Select USE_MADGWICK or USE_MAHONY"
#endif

// =================== Timing and control ===================
constexpr int   LOOP_HZ        = 250;
constexpr float LOOP_DT        = 1.0f / static_cast<float>(LOOP_HZ);
constexpr int   ESC_FREQ_HZ    = 400;
constexpr int   ESC_RESOLUTION = 16;
constexpr int   ESC_MIN_US     = 1000;
constexpr int   ESC_MAX_US     = 2000;

// Motors (change to match your wiring)
constexpr int MOTOR_FL = 16; constexpr int CH_FL = 0;
constexpr int MOTOR_FR = 17; constexpr int CH_FR = 1;
constexpr int MOTOR_BL = 18; constexpr int CH_BL = 2;
constexpr int MOTOR_BR = 19; constexpr int CH_BR = 3;

// Sensors
MPU6050 mpu;
Adafruit_HMC5883_Unified mag(12345);
Adafruit_BMP085 bmp;

// =================== State ===================
volatile float roll=0.0f, pitch=0.0f, yaw=0.0f;             // deg
volatile float rollRate=0.0f, pitchRate=0.0f, yawRate=0.0f; // deg/s
volatile float altitude=0.0f;    // m
volatile float velZ=0.0f;        // m/s

float gyroBiasX=0.0f, gyroBiasY=0.0f, gyroBiasZ=0.0f;
float accBiasX=0.0f,  accBiasY=0.0f,  accBiasZ=0.0f;

float baroFiltered=0.0f;
constexpr float BARO_ALPHA = 0.02f;

// Failsafe: RC timeout (ms)
constexpr unsigned long RC_FAILSAFE_TIMEOUT = 500UL; // 0.5s

// =================== RC and telemetry ===================
struct RCInput {
  float throttle; // 0..1
  float rollCmd;  // -1..1 (angle cmd)
  float pitchCmd; // -1..1
  float yawCmd;   // -1..1 (rate cmd)
  bool autoTuneReq;
  bool posHold;
  bool altHold;
  bool opConfirm;   // operator confirm/ack (GUI)
  bool opAbort;     // operator abort (GUI)
} rc = {0,0,0,0,false,false,false,false,false};

typedef struct {
  uint8_t cmd_id; // 0 RC
  float throttle, roll, pitch, yaw;
  uint8_t flags;  // bit0:autoTuneReq bit1:altHold bit2:posHold bit3:opConfirm bit4:opAbort
} __attribute__((packed)) rc_packet_t;

// =================== Telemetry struct: add battery/failsafe ===================
typedef struct {
  // Core telemetry
  float roll, pitch, yaw;             // deg
  float rollRate, pitchRate, yawRate; // deg/s
  float altitude;                     // m
  // Autotune status
  uint8_t tune_state;           // enum below
  float tune_err, tune_bestErr;
  float pr_kp, pr_ki, pr_kd;    // roll PID
  float pp_kp, pp_ki, pp_kd;    // pitch PID
  float py_kp, py_ki, py_kd;    // yaw PID
  uint8_t tune_step;            // which param index
  float battery;                // battery voltage (V)
  uint8_t failsafe_flags;       // bit0:rc, bit1:battery, bit2:sensor, bit3:tilt
} __attribute__((packed)) telemetry_t;

uint8_t bcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned long lastTelemetry = 0;
constexpr unsigned long TELEMETRY_INTERVAL = 60UL; // ms
telemetry_t t{};

// =================== PID struct & instances ===================
struct PID {
  float kp=0.0f, ki=0.0f, kd=0.0f;
  float integ=0.0f, lastErr=0.0f;
  float outMin=-400.0f, outMax=400.0f;
  float update(float sp, float meas, float dt){
    float e = sp - meas;
    integ += e*dt;
    float deriv = (e - lastErr)/dt;
    lastErr = e;
    float out = kp*e + ki*integ + kd*deriv;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;
    // anti-windup
    if (ki != 0.0f){
      if (integ*ki > outMax) integ = outMax/ki;
      if (integ*ki < outMin) integ = outMin/ki;
    }
    return out;
  }
  void reset(){ integ=0.0f; lastErr=0.0f; }
};

PID pidRoll, pidPitch, pidYaw, pidAlt;

// ======== Tuned sample gains (start points) ========
inline void setTunedSampleGains(){
  // Roll/Pitch — conservative starting values
  pidRoll.kp=7.0f; pidRoll.ki=1.2f; pidRoll.kd=1.0f;
  pidPitch.kp=7.0f; pidPitch.ki=1.2f; pidPitch.kd=1.0f;

  // Yaw (rate)
  pidYaw.kp=3.5f; pidYaw.ki=0.6f; pidYaw.kd=0.05f;

  // Altitude (baro+accZ EKF)
  pidAlt.kp=2.2f; pidAlt.ki=0.6f; pidAlt.kd=0.5f;

  // Output limits (motor-normalized domain)
  pidRoll.outMin = pidPitch.outMin = pidYaw.outMin = -380.0f;
  pidRoll.outMax = pidPitch.outMax = pidYaw.outMax =  380.0f;
  pidAlt.outMin  = -350.0f; pidAlt.outMax = 350.0f;

  // AHRS default tuning
  #if defined(USE_MADGWICK)
    filter.setBeta(0.08f);    // increase for snappier response, decrease to reduce noise
  #elif defined(USE_MAHONY)
    filter.setKp(0.8f);
    filter.setKi(0.02f);
  #endif
}

// =================== Altitude EKF (2-state: z, vz) ===================
// Guidance notes included here to help future tuning:
//
// - Q (process noise): how much you trust your motion model (accZ integration).
//     * Increase Q[0][0] (altitude process noise) if EKF is slow to follow real altitude changes.
//     * Increase Q[1][1] (velocity process noise) if velocity estimate is noisy or drifting.
//     * Typical starting values: Q[0][0] = 0.01..0.1, Q[1][1] = 0.2..1.0.
// - R (measurement noise): how much you trust barometer.
//     * Increase R if barometer is very noisy (larger R => smoother, slower response).
//     * Decrease R if barometer is very stable and you want EKF to follow it more closely.
//     * Typical starting value: R = 0.5..2.0 (meters^2).
struct AltEKF {
  float x[2] = {0.0f, 0.0f};                  // x[0]: altitude (m), x[1]: vertical velocity (m/s)
  float P[2][2] = {{1.0f,0.0f},{0.0f,1.0f}};  // Covariance
  // Default process noise values chosen conservatively; tune as notes above
  float Q[2][2] = {{0.02f,0.0f},{0.0f,0.5f}}; // Q[0][0]: altitude process noise, Q[1][1]: velocity process noise
  float R = 0.8f;                             // Baro measurement noise (m^2)
  unsigned long lastMicros = 0;
} altEKF;

inline void updateAltitudeEKF(float accZ, float baroAlt) {
  unsigned long now = micros();
  float dt = (altEKF.lastMicros > 0) ? (now - altEKF.lastMicros) * 1e-6f : LOOP_DT;
  altEKF.lastMicros = now;
  if (dt < 1e-5f) dt = LOOP_DT;

  // Predict step (constant-acceleration kinematic)
  altEKF.x[0] += altEKF.x[1] * dt + 0.5f * accZ * dt * dt;
  altEKF.x[1] += accZ * dt;

  // Covariance predict (F*P*F^T + Q) for 2-state linear model
  float P00 = altEKF.P[0][0], P01 = altEKF.P[0][1], P10 = altEKF.P[1][0], P11 = altEKF.P[1][1];
  altEKF.P[0][0] = P00 + dt*(P10 + P01) + dt*dt*P11 + altEKF.Q[0][0];
  altEKF.P[0][1] = P01 + dt*P11 + altEKF.Q[0][1];
  altEKF.P[1][0] = P10 + dt*P11 + altEKF.Q[1][0];
  altEKF.P[1][1] = P11 + altEKF.Q[1][1];

  // Update (barometer measurement)
  float y = baroAlt - altEKF.x[0];
  float S = altEKF.P[0][0] + altEKF.R;
  float K0 = altEKF.P[0][0] / S;
  float K1 = altEKF.P[1][0] / S;

  altEKF.x[0] += K0 * y;
  altEKF.x[1] += K1 * y;

  // Covariance update (Joseph form simplified)
  float P00_new = (1 - K0) * altEKF.P[0][0];
  float P01_new = (1 - K0) * altEKF.P[0][1];
  float P10_new = -K1 * altEKF.P[0][0] + altEKF.P[1][0];
  float P11_new = -K1 * altEKF.P[0][1] + altEKF.P[1][1];

  altEKF.P[0][0] = P00_new;
  altEKF.P[0][1] = P01_new;
  altEKF.P[1][0] = P10_new;
  altEKF.P[1][1] = P11_new;
}

// =================== ESC helpers ===================
inline uint32_t microsToDuty(unsigned int us){
  const uint32_t maxDuty = (1u<<ESC_RESOLUTION)-1u;
  const float period_us = 1000000.0f/static_cast<float>(ESC_FREQ_HZ);
  float duty = (us/period_us)*maxDuty;
  if (duty < 0.0f) duty = 0.0f;
  if (duty > (float)maxDuty) duty = (float)maxDuty;
  return static_cast<uint32_t>(duty);
}
inline void motorUS(int ch, unsigned int us){ ledcWrite(ch, microsToDuty(us)); }
inline void motorsWrite01(float fl, float fr, float bl, float br){
  auto toUS=[&](float v){ v = constrain(v, 0.0f, 1.0f); return static_cast<unsigned int>(map((int)(v*1000.0f),0,1000,ESC_MIN_US,ESC_MAX_US)); };
  motorUS(CH_FL,toUS(fl)); motorUS(CH_FR,toUS(fr)); motorUS(CH_BL,toUS(bl)); motorUS(CH_BR,toUS(br));
}

// =================== ESP-NOW & failsafe handling ===================
unsigned long lastRCRecv = 0;
bool rcFailsafe = false;

void onRecv(const uint8_t* mac, const uint8_t* data, int len){
  if (len == (int)sizeof(rc_packet_t)){
    rc_packet_t p; memcpy(&p, data, len);
    rc.throttle = constrain(p.throttle, 0.0f, 1.0f);
    rc.rollCmd  = constrain(p.roll, -1.0f, 1.0f);
    rc.pitchCmd = constrain(p.pitch, -1.0f, 1.0f);
    rc.yawCmd   = constrain(p.yaw, -1.0f, 1.0f);
    rc.autoTuneReq = p.flags & 0x01;
    rc.altHold     = p.flags & 0x02;
    rc.posHold     = p.flags & 0x04;
    rc.opConfirm   = p.flags & 0x08;
    rc.opAbort     = p.flags & 0x10;
    lastRCRecv = millis();
    rcFailsafe = false;
  }
}

void sendTelemetry(){
  t.roll = roll; t.pitch = pitch; t.yaw = yaw;
  t.rollRate = rollRate; t.pitchRate = pitchRate; t.yawRate = yawRate;
  t.altitude = altitude;
  t.battery = batteryVoltage;
  t.failsafe_flags = (rcFailsafe ? 0x01 : 0) |
                     (batteryFailsafe ? 0x02 : 0) |
                     (sensorFailsafe ? 0x04 : 0) |
                     (tiltFailsafe ? 0x08 : 0);
  esp_now_send(bcast, (uint8_t*)&t, sizeof(t));
}

// =================== AHRS + sensors reading ===================
void readSensorsAndUpdateAHRS(){
  VectorInt16 aa, gg;
  mpu.getAcceleration(&aa);
  mpu.getRotation(&gg);
  sensors_event_t me; mag.getEvent(&me);

  const float aRes = 16384.0f; // ±2g
  const float gRes = 131.0f;   // ±250 dps
  float ax_g = (aa.x / aRes) - accBiasX;
  float ay_g = (aa.y / aRes) - accBiasY;
  float az_g = (aa.z / aRes) - accBiasZ;

  float gx_dps = (gg.x / gRes) - gyroBiasX;
  float gy_dps = (gg.y / gRes) - gyroBiasY;
  float gz_dps = (gg.z / gRes) - gyroBiasZ;

  rollRate  = gx_dps;
  pitchRate = gy_dps;
  yawRate   = gz_dps;

  constexpr float DEG2RAD = 0.01745329252f;
  float gx = gx_dps * DEG2RAD, gy = gy_dps * DEG2RAD, gz = gz_dps * DEG2RAD;

  // AHRS expects gyro (rad/s), accel (g), mag (uT)
  filter.update(gx, gy, gz, ax_g, ay_g, az_g, me.magnetic.x, me.magnetic.y, me.magnetic.z);

  // Quaternion -> Euler (deg), ZYX convention
  float q0=filter.q0, q1=filter.q1, q2=filter.q2, q3=filter.q3;
  float sinr_cosp = 2.0f*(q0*q1 + q2*q3);
  float cosr_cosp = 1.0f - 2.0f*(q1*q1 + q2*q2);
  roll = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

  float sinp = 2.0f*(q0*q2 - q3*q1);
  if (fabs(sinp) >= 1.0f) pitch = copysignf(90.0f, sinp);
  else pitch = asinf(sinp) * 57.2957795f;

  float siny_cosp = 2.0f*(q0*q3 + q1*q2);
  float cosy_cosp = 1.0f - 2.0f*(q2*q2 + q3*q3);
  yaw = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
  if (yaw < 0.0f) yaw += 360.0f;

  // Baro (IIR)
  float newAlt = bmp.readAltitude();
  if (baroFiltered == 0.0f) baroFiltered = newAlt;
  baroFiltered = baroFiltered * (1.0f - BARO_ALPHA) + newAlt * BARO_ALPHA;

  // Vertical acceleration in world Z (approx; small-angle assumption)
  float pitchRad = pitch * DEG2RAD;
  float rollRad  = roll  * DEG2RAD;
  float accZ_world_g = az_g * cosf(pitchRad) * cosf(rollRad)
                     + ax_g * sinf(pitchRad)
                     - ay_g * sinf(rollRad);
  float accZ_ms2 = (accZ_world_g - 1.0f) * 9.80665f; // m/s^2

  // EKF fusion
  updateAltitudeEKF(accZ_ms2, baroFiltered);
  altitude = altEKF.x[0];
  velZ     = altEKF.x[1];
}

// =================== Control / mixing ===================
static inline float baseThrottle(float x){ return constrain(x,0.0f,1.0f); }

bool hoverActive = false;
float hoverRollSet = 0.0f, hoverPitchSet = 0.0f, hoverAltSet = 0.0f, hoverYawSet = 0.0f;
constexpr float HOVER_STICK_THRESH = 0.05f;
constexpr float HOVER_MIN_THR = 0.15f;

void updateHoverLogic() {
  // Activate hover if sticks centered, throttle > min, and not tuning
  static bool lastHover = false;
  bool hoverCmd = !rc.autoTuneReq &&
                  rc.altHold &&
                  fabs(rc.rollCmd) < HOVER_STICK_THRESH &&
                  fabs(rc.pitchCmd) < HOVER_STICK_THRESH &&
                  fabs(rc.yawCmd) < HOVER_STICK_THRESH &&
                  rc.throttle > HOVER_MIN_THR;
  if (hoverCmd && !lastHover) {
    hoverActive = true;
    hoverAltSet = altitude;
    hoverRollSet = 0.0f;
    hoverPitchSet = 0.0f;
    hoverYawSet = yaw;
  } else if (!hoverCmd) {
    hoverActive = false;
  }
  lastHover = hoverCmd;
}

void runController(){
  float rollSet, pitchSet, yawRateSet, altSet;
  if (hoverActive) {
    rollSet  = hoverRollSet;
    pitchSet = hoverPitchSet;
    // Yaw hold: try to keep heading (simple P controller)
    float yawErr = hoverYawSet - yaw;
    if (yawErr > 180.0f) yawErr -= 360.0f;
    if (yawErr < -180.0f) yawErr += 360.0f;
    yawRateSet = constrain(yawErr * 2.0f, -90.0f, 90.0f); // deg/s, simple heading hold
    altSet = hoverAltSet;
  } else {
    rollSet  = rc.rollCmd  * 30.0f;   // deg
    pitchSet = rc.pitchCmd * 30.0f;   // deg
    yawRateSet = rc.yawCmd * 150.0f;  // deg/s
    altSet = altitude; // will be latched if altHold
  }

  float rollOut  = pidRoll.update(rollSet,  roll,  LOOP_DT);
  float pitchOut = pidPitch.update(pitchSet, pitch, LOOP_DT);
  float yawOut   = pidYaw.update(yawRateSet, yawRate, LOOP_DT);

  float altOut = 0.0f;
  static float altHoldSet=0.0f; static bool altLatched=false;
  if (rc.altHold || hoverActive){
    if (!altLatched){ altHoldSet = altSet; altLatched=true; }
    altOut = pidAlt.update(altHoldSet, altitude, LOOP_DT);
  } else { altLatched=false; pidAlt.reset(); }

  float base = baseThrottle(rc.throttle);

  // Quad X mixing (normalized motor commands 0..1)
  float mFL = base + (-rollOut + pitchOut)/1000.0f +  yawOut/1000.0f + altOut/1000.0f;
  float mFR = base + ( rollOut + pitchOut)/1000.0f + -yawOut/1000.0f + altOut/1000.0f;
  float mBL = base + (-rollOut - pitchOut)/1000.0f + -yawOut/1000.0f + altOut/1000.0f;
  float mBR = base + ( rollOut - pitchOut)/1000.0f +  yawOut/1000.0f + altOut/1000.0f;

  motorsWrite01(mFL, mFR, mBL, mBR);
}

/*
Control Scheme Overview:

- **RC Inputs**:
  - `rc.throttle` (0..1): Throttle stick (altitude control or direct throttle)
  - `rc.rollCmd`  (-1..1): Roll stick (left/right tilt)
  - `rc.pitchCmd` (-1..1): Pitch stick (forward/back tilt)
  - `rc.yawCmd`   (-1..1): Yaw stick (heading rate)
  - `rc.altHold`: Altitude hold mode (baro+accZ EKF)
  - `rc.autoTuneReq`: Start/stop autotune
  - `rc.opConfirm`/`rc.opAbort`: Operator confirmation/abort for autotune

- **Arming/Disarming**:
  - Throttle low + yaw left: Arm (hold for 0.8s)
  - Throttle low + yaw right: Disarm (hold for 0.8s)
  - Auto-disarm on failsafe or battery failsafe

- **Failsafes**:
  - RC signal loss: Motors stop, controller disarmed
  - Battery low: Motors stop, controller disarmed
  - Sensor error or excessive tilt: Motors stop, controller disarmed

- **Hover/Position Hold**:
  - When sticks are centered, throttle > 0.15, and `altHold` is active, the FC holds current altitude, heading, and level attitude.

- **Control Loops**:
  - **Attitude**: PID control of roll and pitch angle (setpoint from RC or hover logic)
  - **Yaw**: PID control of yaw rate (setpoint from RC or heading hold in hover)
  - **Altitude**: PID control of altitude (setpoint from RC or hover logic, using EKF estimate)
  - **Mixing**: Quad X mixer combines PID outputs and throttle to generate normalized motor commands (0..1)

- **AutoTune**:
  - Non-blocking, operator-guided autotune for roll/pitch PID gains using excitation and error evaluation.

- **Telemetry**:
  - Sends attitude, rates, altitude, PID gains, battery voltage, and failsafe flags to ground station via ESP-NOW.

- **Motor Outputs**:
  - Four PWM outputs via ESP32 LEDC, mapped to ESCs.

See the `runController()` function for the main control law implementation.
*/

// =================== Full AutoTune (non-blocking) ===================
// ... (keeps your implementation largely intact; uses fullTuner struct from previous step)
enum class AutoTuneState : uint8_t { IDLE=0, WAIT_OP, PREP, EXCITE, EVAL, ADJUST, NEXT_AXIS, DONE, ABORT };

struct FullTuner {
  AutoTuneState state = AutoTuneState::IDLE;
  unsigned long stateStart = 0;
  float bestErr[2][3] = {{1e9,1e9,1e9},{1e9,1e9,1e9}}; // [axis][param]
  float curErr = 0.0f;
  unsigned long evalStart = 0;
  float exciteAmpDeg = 5.0f;
  float exciteFreq = 1.0f;
  float settleMs = 800.0f, exciteMs = 2000.0f, evalMs = 2000.0f;
  float maxTiltDeg = 15.0f;
  float maxThrottle = 0.20f;
  bool newBest = false;

  PID* pids[2] = {&pidRoll, &pidPitch};
  float* params[2][3] = { {&pidRoll.kp, &pidRoll.ki, &pidRoll.kd},
                          {&pidPitch.kp, &pidPitch.ki, &pidPitch.kd} };
  float dp[2][3] = { {0.5f, 0.2f, 0.1f}, {0.5f, 0.2f, 0.1f} };
  int axis = 0; // 0=roll, 1=pitch
  int param = 0; // 0=kp,1=ki,2=kd
  int phase = 0; // 0:+dp,1:-2dp,2:reduce

  void begin() {
    state = AutoTuneState::WAIT_OP; stateStart = millis();
    for (int a=0;a<2;a++) for (int p=0;p<3;p++) bestErr[a][p] = 1e9f;
    axis = 0; param = 0; phase = 0; newBest = false;
  }
  void abort() { state = AutoTuneState::ABORT; stateStart = millis(); }
  bool safetyOK() {
    if (rc.throttle > maxThrottle) return false;
    if (fabs(roll) > maxTiltDeg || fabs(pitch) > maxTiltDeg) return false;
    if (rc.opAbort) return false;
    return true;
  }
  void nextState(AutoTuneState s){ state = s; stateStart = millis(); }
  float currentSetpointOffsetDeg() {
    float t = (millis()/1000.0f);
    return exciteAmpDeg * sinf(2.0f*3.1415926f*exciteFreq*t);
  }
  void twiddleStep() {
    float* paramPtr = params[axis][param];
    float& d = dp[axis][param];
    if (phase==0) { *paramPtr += d; phase=1; }
    else if (phase==1) {
      if (curErr < bestErr[axis][param]) { bestErr[axis][param]=curErr; d*=1.1f; param=(param+1)%3; phase=0; newBest=true; }
      else { *paramPtr -= 2.0f*d; phase=2; }
    } else {
      if (curErr < bestErr[axis][param]) { bestErr[axis][param]=curErr; d*=1.1f; param=(param+1)%3; phase=0; newBest=true; }
      else { *paramPtr += d; d*=0.9f; param=(param+1)%3; phase=0; }
    }
  }
  void update() {
    switch(state) {
      case AutoTuneState::IDLE: return;
      case AutoTuneState::WAIT_OP:
        if (rc.opAbort) { abort(); break; }
        if (rc.opConfirm) nextState(AutoTuneState::PREP);
        break;
      case AutoTuneState::PREP:
        if (!safetyOK()) { abort(); break; }
        pids[axis]->reset();
        evalStart = 0; curErr = 0.0f;
        if (millis()-stateStart > (unsigned long)settleMs) nextState(AutoTuneState::EXCITE);
        break;
      case AutoTuneState::EXCITE:
        if (!safetyOK()) { abort(); break; }
        if (millis()-stateStart > (unsigned long)exciteMs) { evalStart = millis(); curErr = 0.0f; nextState(AutoTuneState::EVAL); }
        break;
      case AutoTuneState::EVAL:
        if (!safetyOK()) { abort(); break; }
        {
          float e = (axis==0) ? fabs(pidRoll.lastErr) : fabs(pidPitch.lastErr);
          float rp = (axis==0) ? fabs(rollRate) : fabs(pitchRate);
          curErr += (e + 0.02f*rp) * (LOOP_DT);
        }
        if (millis()-evalStart > (unsigned long)evalMs) nextState(AutoTuneState::ADJUST);
        break;
      case AutoTuneState::ADJUST:
        if (!safetyOK()) { abort(); break; }
        twiddleStep();
        {
          float sumdp = dp[axis][0] + dp[axis][1] + dp[axis][2];
          if (sumdp < 0.10f) {
            if (axis==0) { axis=1; param=0; phase=0; nextState(AutoTuneState::PREP); }
            else nextState(AutoTuneState::DONE);
          } else nextState(AutoTuneState::EXCITE);
        }
        break;
      case AutoTuneState::DONE:
        if (rc.opAbort) { abort(); break; }
        if (rc.opConfirm) nextState(AutoTuneState::IDLE);
        break;
      case AutoTuneState::ABORT:
        if (rc.opConfirm) nextState(AutoTuneState::IDLE);
        break;
      default: break;
    }
  }
} fullTuner;

// small globals for excitation offsets
float tuneRollOffsetDeg = 0.0f, tunePitchOffsetDeg = 0.0f;

// =================== Battery Monitoring ===================
constexpr int BATTERY_PIN = 34; // ADC1_CH6 (GPIO34) — adjust to your wiring
constexpr float BATTERY_VOLTAGE_DIVIDER = 2.0f; // resistor divider factor
constexpr float BATTERY_ADC_REF = 3.3f;
constexpr int   BATTERY_ADC_MAX = 4095;
constexpr float BATTERY_LOW_VOLTAGE = 10.5f; // 3S LiPo low threshold
constexpr float BATTERY_CRIT_VOLTAGE = 9.6f; // critical threshold

float batteryVoltage = 0.0f;
bool batteryFailsafe = false;

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float v = (raw / static_cast<float>(BATTERY_ADC_MAX)) * BATTERY_ADC_REF * BATTERY_VOLTAGE_DIVIDER;
  return v;
}

// =================== Arming/Disarming ===================
enum class ArmState : uint8_t { DISARMED=0, ARMING, ARMED, DISARMING };
ArmState armState = ArmState::DISARMED;
unsigned long lastArmCmd = 0;
constexpr unsigned long ARM_HOLD_MS = 800UL; // Hold stick for 0.8s to arm/disarm

void updateArmingLogic() {
  static bool lastArmCombo = false, lastDisarmCombo = false;
  bool armCombo = (rc.throttle < 0.05f) && (rc.yawCmd < -0.95f);
  bool disarmCombo = (rc.throttle < 0.05f) && (rc.yawCmd > 0.95f);

  if (armState == ArmState::DISARMED) {
    if (armCombo) {
      if (!lastArmCombo) lastArmCmd = millis();
      if (millis() - lastArmCmd > ARM_HOLD_MS) armState = ArmState::ARMED;
    } else {
      lastArmCmd = millis();
    }
    lastArmCombo = armCombo;
  } else if (armState == ArmState::ARMED) {
    if (disarmCombo) {
      if (!lastDisarmCombo) lastArmCmd = millis();
      if (millis() - lastArmCmd > ARM_HOLD_MS) armState = ArmState::DISARMED;
    } else {
      lastArmCmd = millis();
    }
    lastDisarmCombo = disarmCombo;
  }
  // Auto-disarm on failsafe or battery critical
  if (rcFailsafe || batteryFailsafe) armState = ArmState::DISARMED;
}

// =================== Advanced Failsafes ===================
bool sensorFailsafe = false;
bool tiltFailsafe = false;
constexpr float MAX_SAFE_TILT_DEG = 60.0f;

void updateFailsafes() {
  // Sensor sanity check
  if (isnan(roll) || isnan(pitch) || isnan(yaw) || isnan(altitude)) sensorFailsafe = true;
  else sensorFailsafe = false;

  // Excessive tilt
  tiltFailsafe = (fabs(roll) > MAX_SAFE_TILT_DEG) || (fabs(pitch) > MAX_SAFE_TILT_DEG);

  // Battery failsafe
  batteryFailsafe = (batteryVoltage < BATTERY_LOW_VOLTAGE);
}

// =================== Sensor setup + misc setup ===================
void setupSensors(){
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not detected! Check wiring.");
    while (1) delay(1000);
  }
  if (!mag.begin()) {
    Serial.println("HMC5883L not detected! Check wiring.");
    while (1) delay(1000);
  }
  if (!bmp.begin()) {
    Serial.println("BMP085/BMP180 not detected! Check wiring.");
    while (1) delay(1000);
  }
  filter.begin(LOOP_HZ);
}

void setupMotors(){
  ledcSetup(CH_FL,ESC_FREQ_HZ,ESC_RESOLUTION); ledcAttachPin(MOTOR_FL,CH_FL);
  ledcSetup(CH_FR,ESC_FREQ_HZ,ESC_RESOLUTION); ledcAttachPin(MOTOR_FR,CH_FR);
  ledcSetup(CH_BL,ESC_FREQ_HZ,ESC_RESOLUTION); ledcAttachPin(MOTOR_BL,CH_BL);
  ledcSetup(CH_BR,ESC_FREQ_HZ,ESC_RESOLUTION); ledcAttachPin(MOTOR_BR,CH_BR);
  motorUS(CH_FL,ESC_MIN_US); motorUS(CH_FR,ESC_MIN_US); motorUS(CH_BL,ESC_MIN_US); motorUS(CH_BR,ESC_MIN_US);
}

void setupESPNow(){
  WiFi.mode(WIFI_STA); WiFi.disconnect();
  if (esp_now_init() != ESP_OK){ Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {}; memcpy(peer.peer_addr, bcast, 6); peer.channel = 0; peer.encrypt = false; esp_now_add_peer(&peer);
}

void calibrateIMU(){
  Serial.println("Calibrating IMU... keep still");
  constexpr int N = 300;
  long gx=0,gy=0,gz=0, ax=0,ay=0,az=0;
  for (int i=0;i<N;i++){
    VectorInt16 aa, gg;
    mpu.getAcceleration(&aa);
    mpu.getRotation(&gg);
    gx += gg.x; gy += gg.y; gz += gg.z;
    ax += aa.x; ay += aa.y; az += aa.z;
    delay(5);
  }
  gyroBiasX = (gx/(float)N) / 131.0f;
  gyroBiasY = (gy/(float)N) / 131.0f;
  gyroBiasZ = (gz/(float)N) / 131.0f;
  accBiasX = (ax/(float)N) / 16384.0f;
  accBiasY = (ay/(float)N) / 16384.0f;
  accBiasZ = (az/(float)N) / 16384.0f;
}

// =================== Optional hover stub (user can extend) ===================
bool hoverActive = false;
float hoverRollSet = 0.0f, hoverPitchSet = 0.0f, hoverAltSet = 0.0f;
void updateHoverLogic() {
  // Placeholder: implement auto-hover mode if desired
}

// =================== Setup & Loop ===================
void setup(){
  Serial.begin(115200);
  delay(200);
  setupSensors();
  setupMotors();
  setupESPNow();
  calibrateIMU();
  setTunedSampleGains();
  analogReadResolution(12); // ESP32 ADC resolution
  pinMode(BATTERY_PIN, INPUT);
  Serial.println("FC ready (bench test, props OFF).");
}

void loop(){
  unsigned long startMicros = micros();

  // RC failsafe check
  if ((millis() - lastRCRecv) > RC_FAILSAFE_TIMEOUT) {
    rcFailsafe = true;
    // zero RC state (safe defaults)
    rc.throttle = 0.0f;
    rc.rollCmd = 0.0f;
    rc.pitchCmd = 0.0f;
    rc.yawCmd = 0.0f;
    rc.autoTuneReq = false;
    rc.altHold = false;
    rc.posHold = false;
    rc.opConfirm = false;
    rc.opAbort = false;
  }

  // Battery monitoring
  batteryVoltage = readBatteryVoltage();

  // Arming/disarming
  updateArmingLogic();

  // Update failsafes
  updateFailsafes();

  // Hover logic (user-provided)
  updateHoverLogic();

  // Sensors + AHRS + Alt EKF
  readSensorsAndUpdateAHRS();

  // Full auto-tune handling (non-blocking)
  static bool tuningActive = false;
  if (rc.autoTuneReq && !tuningActive){ fullTuner.begin(); tuningActive=true; }
  if (!rc.autoTuneReq && tuningActive){ tuningActive=false; }
  tuneRollOffsetDeg = 0.0f; tunePitchOffsetDeg = 0.0f;
  if (tuningActive && (fullTuner.state == AutoTuneState::EXCITE || fullTuner.state == AutoTuneState::EVAL)) {
    float off = fullTuner.currentSetpointOffsetDeg();
    if (fullTuner.axis == 0) tuneRollOffsetDeg = off; else tunePitchOffsetDeg = off;
  }
  fullTuner.update();

  // Controller execution (only if armed and no failsafe conditions)
  // Save RC commands, apply excitation offsets for tuning
  float savedRollCmd = rc.rollCmd, savedPitchCmd = rc.pitchCmd;
  rc.rollCmd  = constrain(rc.rollCmd + (tuneRollOffsetDeg/30.0f), -1.0f, 1.0f);
  rc.pitchCmd = constrain(rc.pitchCmd + (tunePitchOffsetDeg/30.0f), -1.0f, 1.0f);

  // Throttle clamp while tuning
  float savedThr = rc.throttle;
  if (tuningActive) rc.throttle = min(rc.throttle, fullTuner.maxThrottle);

  if (armState == ArmState::ARMED && !rcFailsafe && !sensorFailsafe && !tiltFailsafe && !batteryFailsafe) {
    runController();
  } else {
    // SAFE: stop motors when disarmed or failsafe active
    motorsWrite01(0.0f, 0.0f, 0.0f, 0.0f);
  }

  // restore RC
  rc.rollCmd = savedRollCmd; rc.pitchCmd = savedPitchCmd;
  if (tuningActive) rc.throttle = savedThr;

  // Telemetry send
  if ((millis() - lastTelemetry) > TELEMETRY_INTERVAL) {
    t.tune_state = static_cast<uint8_t>(fullTuner.state);
    // bestErr for current axis/param — safe fetch
    t.tune_err = fullTuner.curErr;
    t.tune_bestErr = fullTuner.bestErr[fullTuner.axis][fullTuner.param];
    t.pr_kp = pidRoll.kp; t.pr_ki = pidRoll.ki; t.pr_kd = pidRoll.kd;
    t.pp_kp = pidPitch.kp; t.pp_ki = pidPitch.ki; t.pp_kd = pidPitch.kd;
    t.py_kp = pidYaw.kp; t.py_ki = pidYaw.ki; t.py_kd = pidYaw.kd;
    t.tune_step = static_cast<uint8_t>(fullTuner.param + (fullTuner.axis==1 ? 3 : 0));
    sendTelemetry();
    lastTelemetry = millis();
  }

  // Loop timing
  int target_us = static_cast<int>(LOOP_DT * 1e6f);
  unsigned long used = micros() - startMicros;
  if ((int)used < target_us) delayMicroseconds(target_us - used);
}

// =================== Rangefinder integration (optional) ===================
// For low-altitude flight, consider integrating an accurate rangefinder (sonar/lidar).
// Example placeholder (implement hardware-specific readRangefinder()):
// float rangefinderAlt = readRangefinder();
// if (rangefinderAlt > 0.0f && rangefinderAlt < 3.0f) { // trust only at low altitudes
//   // replace baro/EKF output at low altitude or fuse it: recommended -> use as a measurement in EKF
//   altitude = rangefinderAlt;
// }

// =================== END OF SKETCH ===================
