// -------------------------------------------------------------------------------------------------
// Pin map for OnStepPi (Teensy3.5/3.6)
#pragma once

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

// Serial ports (see Pins.defaults.h for SERIAL_A)
// Serial1: RX1 Pin 0, TX1 Pin 1
// Serial4: RX4 Pin 31, TX4 Pin 32

#if SERIAL_A_BAUD_DEFAULT != OFF
  #define SERIAL_A              Serial5
#endif
#if SERIAL_B_BAUD_DEFAULT != OFF
  #define SERIAL_B              Serial1
#endif
#if SERIAL_C_BAUD_DEFAULT != OFF
  #define SERIAL_C              Serial4
#endif
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  #define SERIAL_D              SerialUSB1
  #define SERIAL_D_BAUD_DEFAULT 9600
#endif
#if defined(USB_TRIPLE_SERIAL)
  #define SERIAL_E              SerialUSB2
  #define SERIAL_E_BAUD_DEFAULT 9600
#endif

// The multi-purpose pins (Aux3..Aux8 can be analog pwm/dac if supported)
#define AUX0_PIN               2                // Status LED
#define AUX1_PIN               OFF              // ESP8266 GPIO0, SPI MISO/Fault
#define AUX2_PIN               OFF               // ESP8266 RST, SPI MISO/Fault
#define AUX3_PIN               OFF              // Home SW
#define AUX4_PIN               OFF              // OneWire, Home SW
#define AUX5_PIN               OFF                // AXIS3_EN_PIN; true analog output
#define AUX6_PIN               OFF               // AXIS4_EN_PIN; true analog output
#define AUX7_PIN               OFF                   // Limit SW
#define AUX8_PIN               OFF                // Status2 LED, Reticle LED

// Misc. pins
#ifndef DS3234_CS_PIN
  #define DS3234_CS_PIN        OFF                // Default CS Pin for DS3234 on SPI
#endif
#ifndef ONE_WIRE_PIN
  #define ONE_WIRE_PIN         AUX4_PIN          // Default Pin for OneWire bus (note: this pin has a 0.1uF capacitor that must be removed for OneWire to function)
#endif
#define ADDON_GPIO0_PIN        AUX1_PIN          // ESP8266 GPIO0 or SPI MISO/Fault
#define ADDON_RESET_PIN        AUX2_PIN          // ESP8266 RST or SPI MISO/Fault

// The PEC index sense is a logic level input, resets the PEC index on rising edge then waits for 60 seconds before allowing another reset
#ifndef PEC_SENSE_PIN
  #define PEC_SENSE_PIN        OFF                // PEC Sense, analog or digital
#endif

// The status LED is a two wire jumper with a 10k resistor in series to limit the current to the LED
#define STATUS_LED_PIN         2          // LED Anode (+)
#define STATUS_LED_ON_STATE    HIGH             // LED Anode (+)

#define MOUNT_STATUS_LED_PIN   AUX0_PIN          // Default LED Cathode (-)
#ifndef RETICLE_LED_PIN
  #define RETICLE_LED_PIN      AUX8_PIN          // Default LED Cathode (-)
#endif

// For a piezo buzzer
#define STATUS_BUZZER_PIN      OFF                // Tone

// The PPS pin is a 3.3V logic input, OnStep measures time between rising edges and adjusts the internal sidereal clock frequency
#ifndef PPS_SENSE_PIN
  #define PPS_SENSE_PIN        OFF                // PPS time source, GPS for example
#endif

#ifndef LIMIT_SENSE_PIN
  #define LIMIT_SENSE_PIN      AUX7_PIN          // The limit switch sense is a logic level input normally pull high (2k resistor,) shorted to ground it stops gotos/tracking
#endif

#define TMC_MOSI    0
#define TMC_MISO    1
#define TMC_SCK     32
#define TMC_ENABLE  14

#define SHARED_ENABLE_PIN TMC_ENABLE

// Axis1 RA/Azm step/dir driver
#define AXIS1_ENABLE_PIN       SHARED
#define AXIS1_M0_PIN           TMC_MOSI          // SPI MOSI
#define AXIS1_M1_PIN           TMC_SCK           // SPI SCK
#define AXIS1_M2_PIN           11                // SPI CS (UART TX)
#define AXIS1_M3_PIN           TMC_MISO          // SPI MISO (UART RX)
#define AXIS1_STEP_PIN         9
#define AXIS1_DIR_PIN          10
#define AXIS1_DECAY_PIN        OFF
#define AXIS1_FAULT_PIN        OFF
#ifndef AXIS1_SENSE_HOME_PIN
  #define AXIS1_SENSE_HOME_PIN OFF
#endif
#define AXIS1_ENCODER_A_PIN    OFF
#define AXIS1_ENCODER_B_PIN    OFF
#define AXIS1_SERVO_PH1_PIN    OFF
#define AXIS1_SERVO_PH2_PIN    OFF

// Axis2 Dec/Alt step/dir driver
#define AXIS2_ENABLE_PIN       SHARED
#define AXIS2_M0_PIN           TMC_MOSI                 // SPI MOSI
#define AXIS2_M1_PIN           TMC_SCK                 // SPI SCK
#define AXIS2_M2_PIN           26                 // SPI CS (UART TX)
#define AXIS2_M3_PIN           TMC_MISO          // SPI MISO (UART RX)
#define AXIS2_STEP_PIN         24
#define AXIS2_DIR_PIN          25
#define AXIS2_DECAY_PIN        OFF
#define AXIS2_FAULT_PIN        OFF
#ifndef AXIS2_SENSE_HOME_PIN
  #define AXIS2_SENSE_HOME_PIN OFF
#endif
#define AXIS2_ENCODER_A_PIN    OFF
#define AXIS2_ENCODER_B_PIN    OFF
#define AXIS2_SERVO_PH1_PIN    OFF
#define AXIS2_SERVO_PH2_PIN    OFF

// For rotator stepper driver
#define AXIS3_ENABLE_PIN       OFF
#define AXIS3_M0_PIN           OFF               // SPI MOSI
#define AXIS3_M1_PIN           OFF               // SPI SCK
#define AXIS3_M2_PIN           OFF               // SPI CS
#define AXIS3_M3_PIN           OFF               // SPI MISO
#define AXIS3_STEP_PIN         OFF
#define AXIS3_DIR_PIN          OFF

// For focuser1 stepper driver
#define AXIS4_ENABLE_PIN       SHARED
#define AXIS4_M0_PIN           OFF               // SPI MOSI
#define AXIS4_M1_PIN           OFF               // SPI SCK
#define AXIS4_M2_PIN           OFF               // SPI CS
#define AXIS4_M3_PIN           OFF               // SPI MISO
#define AXIS4_STEP_PIN         28 
#define AXIS4_DIR_PIN          29

// For focuser2 stepper driver
#define AXIS5_ENABLE_PIN       SHARED
#define AXIS5_M0_PIN           OFF               // SPI MOSI
#define AXIS5_M1_PIN           OFF               // SPI SCK
#define AXIS5_M2_PIN           OFF               // SPI CS
#define AXIS5_M3_PIN           OFF               // SPI MISO
#define AXIS5_STEP_PIN         39
#define AXIS5_DIR_PIN          35
#define AXIS5_SENSE_HOME_PIN   31

// ST4 interface
#define ST4_RA_W_PIN           OFF                // ST4 RA- West
#define ST4_DEC_S_PIN          OFF                // ST4 DE- South
#define ST4_DEC_N_PIN          OFF                // ST4 DE+ North
#define ST4_RA_E_PIN           OFF                // ST4 RA+ East

#else
#error "Wrong processor for this configuration!"

#endif
