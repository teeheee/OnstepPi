//--------------------------------------------------------------------------------------------------
// telescope rotator control, axis instance

#include "Rotator.h"

#ifdef ROTATOR_PRESENT

#ifdef AXIS3_SERVO_PRESENT
  ServoControl servoControlAxis3;

  #if AXIS3_ENCODER == AB
    Quadrature encAxis3(AXIS3_ENCODER_A_PIN, AXIS3_ENCODER_B_PIN, 3);
  #elif AXIS3_ENCODER == AB_ESP32
    QuadratureEsp32 encAxis3(AXIS3_ENCODER_A_PIN, AXIS3_ENCODER_B_PIN, 3);
  #elif AXIS3_ENCODER == CW_CCW
    CwCcw encAxis3(AXIS3_ENCODER_A_PIN, AXIS3_ENCODER_B_PIN, 3);
  #elif AXIS3_ENCODER == PULSE_DIR
    PulseDir encAxis3(AXIS3_ENCODER_A_PIN, AXIS3_ENCODER_B_PIN, 3);
  #elif AXIS3_ENCODER == PULSE_ONLY
    PulseOnly encAxis3(AXIS3_ENCODER_A_PIN, &servoControlAxis3.directionHint, 3);
  #elif AXIS3_ENCODER == AS37_H39B_B
    As37h39bb encAxis3(AXIS3_ENCODER_A_PIN, AXIS3_ENCODER_B_PIN, 3);
  #elif AXIS3_ENCODER == SERIAL_BRIDGE
    SerialBridge encAxis3(3);
  #endif

  #if AXIS3_SERVO_FEEDBACK == FB_PID
    Pid pidAxis3(AXIS3_PID_P, AXIS3_PID_I, AXIS3_PID_D, AXIS3_PID_P_GOTO, AXIS3_PID_I_GOTO, AXIS3_PID_D_GOTO, AXIS3_PID_SENSITIVITY);
  #endif

  #if defined(AXIS3_SERVO_DC)
    const ServoDcPins ServoPinsAxis3 = {AXIS3_SERVO_PH1_PIN, AXIS3_SERVO_PH1_STATE, AXIS3_SERVO_PH2_PIN, AXIS3_SERVO_PH2_STATE, AXIS3_ENABLE_PIN, AXIS3_ENABLE_STATE, AXIS3_FAULT_PIN};
    const ServoDcSettings ServoSettingsAxis3 = {AXIS3_DRIVER_MODEL, AXIS3_DRIVER_STATUS, AXIS3_SERVO_MAX_VELOCITY, AXIS3_SERVO_ACCELERATION};
    ServoDc driver3(3, &ServoPinsAxis3, &ServoSettingsAxis3);
  #elif defined(AXIS3_SERVO_TMC2209)
    const ServoTmcPins ServoPinsAxis3 = {AXIS3_STEP_PIN, AXIS3_DIR_PIN, AXIS3_ENABLE_PIN, AXIS3_ENABLE_STATE, AXIS3_M0_PIN, AXIS3_M1_PIN, AXIS3_FAULT_PIN};
    const ServoTmcSettings ServoSettingsAxis3 = {AXIS3_DRIVER_MODEL, AXIS3_DRIVER_STATUS, AXIS3_SERVO_MAX_VELOCITY, AXIS3_SERVO_ACCELERATION, AXIS3_DRIVER_MICROSTEPS, AXIS3_DRIVER_IRUN, AXIS3_DRIVER_VELOCITY};
    ServoTmc2209 driver3(3, &ServoPinsAxis3, &ServoSettingsAxis3);
  #endif

  ServoMotor motor3(3, ((ServoDriver*)&driver3), &encAxis3, AXIS3_ENCODER_ORIGIN, AXIS3_ENCODER_REVERSE == ON, &pidAxis3, &servoControlAxis3, AXIS3_SYNC_THRESHOLD);
#endif

#ifdef AXIS3_STEP_DIR_PRESENT
  const StepDirDriverPins DriverPinsAxis3 = {AXIS3_M0_PIN, AXIS3_M1_PIN, AXIS3_M2_PIN, AXIS3_M2_ON_STATE, AXIS3_M3_PIN, AXIS3_DECAY_PIN, AXIS3_FAULT_PIN};
  const StepDirDriverSettings DriverSettingsAxis3 = {AXIS3_DRIVER_MODEL, AXIS3_DRIVER_MICROSTEPS, AXIS3_DRIVER_MICROSTEPS_GOTO, AXIS3_DRIVER_IHOLD, AXIS3_DRIVER_IRUN, AXIS3_DRIVER_IGOTO, AXIS3_DRIVER_INTPOL, AXIS3_DRIVER_DECAY, AXIS3_DRIVER_DECAY_GOTO, AXIS3_DRIVER_STATUS};
  #if defined(AXIS3_STEP_DIR_LEGACY)
    StepDirGeneric driver3(3, &DriverPinsAxis3, &DriverSettingsAxis3);
  #elif defined(AXIS3_STEP_DIR_TMC_SPI)
    StepDirTmcSPI driver3(3, &DriverPinsAxis3, &DriverSettingsAxis3);
  #elif defined(AXIS3_STEP_DIR_TMC_UART)
    StepDirTmcUART driver3(3, &DriverPinsAxis3, &DriverSettingsAxis3);
  #endif

  const StepDirPins StepDirPinsAxis3 = {AXIS3_STEP_PIN, AXIS3_STEP_STATE, AXIS3_DIR_PIN, AXIS3_ENABLE_PIN, AXIS3_ENABLE_STATE};
  StepDirMotor motor3(3, &StepDirPinsAxis3, ((StepDirDriver*)&driver3));
#endif

const AxisPins PinsAxis3 = {AXIS3_SENSE_LIMIT_MIN_PIN, AXIS3_SENSE_HOME_PIN, AXIS3_SENSE_LIMIT_MAX_PIN, {AXIS3_SENSE_HOME, AXIS3_SENSE_HOME_INIT, AXIS3_SENSE_HOME_DIST_LIMIT, AXIS3_SENSE_LIMIT_MIN, AXIS3_SENSE_LIMIT_MAX, AXIS3_SENSE_LIMIT_INIT}};
const AxisSettings SettingsAxis3 = {AXIS3_STEPS_PER_DEGREE, AXIS3_REVERSE, {AXIS3_LIMIT_MIN, AXIS3_LIMIT_MAX}, AXIS3_BACKLASH_RATE};
Axis axis3(3, &PinsAxis3, &SettingsAxis3, AXIS_MEASURE_DEGREES);

#endif
