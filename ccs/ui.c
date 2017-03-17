//*****************************************************************************
//
// ui.c - User interface module.
//
// Copyright (c) 2007-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 6734 of the RDK-BLDC Firmware Package.
//
//*****************************************************************************
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/cpu_usage.h"
#include "utils/flash_pb.h"
#include "adc_ctrl.h"
#include "commands.h"
#include "faults.h"
#include "hall_ctrl.h"
#include "main.h"
#include "pins.h"
#include "pwm_ctrl.h"
#include "ui.h"
#include "ui_common.h"
#include "ui_ethernet.h"
#include "ui_onboard.h"
#include "ui_uart.h"
#include "irrigation.h"
#include "version.h"
#include "stdio.h"

//*****************************************************************************
//
//! \page ui_intro Introduction
//!
//! There are two user interfaces for the the Brushless DC motor application.
//! One uses an push button for basic control of
//! the motor and two LEDs for basic status feedback, and the other uses the
//! Ethernet port to provide complete control of all aspects of the motor drive
//! as well as monitoring of real-time performance data.
//!
//! The on-board user interface consists of a push button and
//! two LEDs.  The push button cycles between run
//! forward, stop, run backward, stop.
//!
//! The ``Run'' LED flashes the entire time the application is running.  The
//! LED is off most of the time if the motor drive is stopped and on most of
//! the time if it is running.  The ``Fault'' LED is normally off but flashes
//! at a fast rate when a fault occurs.
//!
//! A periodic interrupt is used to poll the state of the push button and
//! perform debouncing.
//!
//! The Ethernet user interface is entirely handled by the Ethernet user
//! interface
//! module.  The only thing provided here is the list of parameters and
//! real-time data items, plus a set of helper functions that are required in
//! order to properly set the values of some of the parameters.
//!
//! This user interface (and the accompanying Ethernet and on-board user
//! interface modules) is more complicated and consumes more program space than
//! would typically exist in a real motor drive application.  The added
//! complexity allows a great deal of flexibility to configure and evaluate the
//! motor drive, its capabilities, and adjust it for the target motor.
//!
//! The code for the user interface is contained in <tt>ui.c</tt>, with
//! <tt>ui.h</tt> containing the definitions for the structures, defines,
//! variables, and functions exported to the remainder of the application.
//
//*****************************************************************************

//*****************************************************************************
//
//! \defgroup ui_api Definitions
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! The rate at which the user interface interrupt occurs.
//
//*****************************************************************************
#define UI_INT_RATE             200
#define UI_TICK_MS              (1000/UI_INT_RATE)
#define UI_TICK_US              (1000000/UI_INT_RATE)
#define UI_TICK_NS              (1000000000/UI_INT_RATE)

//*****************************************************************************
//
//! The rate at which the timer interrupt occurs.
//
//*****************************************************************************
#define TIMER1A_INT_RATE        100
#define TIMER1A_TICK_MS         (1000/TIMER1A_INT_RATE)
#define TIMER1A_TICK_US         (1000000/TIMER1A_INT_RATE)
#define TIMER1A_TICK_NS         (1000000000/TIMER1A_INT_RATE)


#define UI_NUM_BYTES_HP 27

#define UI_NUM_SPEED 128
#define UI_BASE_SPEED 0
#define UI_MAX_SPEED 12000
#define UI_NUM_HALLS 4
#define UI_GAIN_SWITCH_SPEED 3600

//limit for handpiece hall sensors
#define LIMIT_HALL_INDEX_MISSING  10
#define LIMIT_HALL_SPEED_HIGH 296
#define LIMIT_HALL_SPEED_LOW  73
#define LIMIT_HALL_SPEED_NOISE  20
#define LIMIT_HALL_SPEED_RANGE  116
#define LIMIT_HP_VOLTAGE_NOISE  35
#define LIMIT_HP_VOLTAGE1_COUNT 256
#define LIMIT_HP_VOLTAGE2_COUNT 426

// the count limit for consecutive phase short check
#define LIMIT_PHASE_SHORT_CNT 30

// handpiece hall sensor polarity threshold
#define HALL_POLARITY_THRD 300

// delay for hand piece reset
#define HP_RESET_CNT 100

// irrigation current high limit, about 2.5 Amps, the motor has a resistance of 19.9 Ohms
// if 48 volts fully applied, the current is about 2.4 Amps, this check is
// only check for short, which usally has much higher current reading.
#define IRRIGATION_CURRENT_LIMIT 12222

// irrigation current high count limit
#define IRRIGATION_CURRENT_LIMIT_COUNT 10

//*****************************************************************************
//
// Forward declarations for functions declared within this source file for use
// in the parameter and real-time data structures.
//
//*****************************************************************************
static void UIConnectionTimeout(void);
static void UIControlType(void);
static void UIDirectionSet(void);
static void UIPWMFrequencySet(void);
static void UIUpdateRate(void);
static void UISetIrrigationLevel(void);
static void UIFAdjI(void);
static void UIDynamicBrake(void);
void UIButtonPress(void);
static void UIButtonHold(void);
static void UIDecayMode(void);
static void UISetEEOrigin(void);
static void UISetEEAxis(void);
static void UISetEENormal(void);
static void UISetEESerialNumber(void);
static void UIResetHandPiece(void);

//*****************************************************************************
//
//! The blink rate of the two LEDs on the board; this is the number of user
//! interface interrupts for an entire blink cycle.  The run LED is the first
//! entry of the array and the fault LED is the second entry of the array.
//
//*****************************************************************************
static unsigned short g_pusBlinkRate[2] =
{
    0, 0
};

//*****************************************************************************
//
//! The blink period of the two LEDs on the board; this is the number of user
//! interface interrupts for which the LED will be turned on.  The run LED is
//! the first entry of the array and the fault LED is the second entry of the
//! array.
//
//*****************************************************************************
static unsigned short g_pusBlinkPeriod[2];

//*****************************************************************************
//
//! The count of user interface interrupts that have occurred.  This is used
//! to determine when to toggle the LEDs that are blinking.
//
//*****************************************************************************
static unsigned long g_ulBlinkCount = 0;

//*****************************************************************************
//
//! This array contains the base address of the GPIO blocks for the two LEDs
//! on the board.
//
//*****************************************************************************
static const unsigned long g_pulLEDBase[2] =
{
    PIN_LEDRUN_PORT,
    PIN_LEDFAULT_PORT
};

//*****************************************************************************
//
//! This array contains the pin numbers of the two LEDs on the board.
//
//*****************************************************************************
static const unsigned char g_pucLEDPin[2] =
{
    PIN_LEDRUN_PIN,
    PIN_LEDFAULT_PIN
};

//*****************************************************************************
//
//! The specification of the control variable on the motor.  This variable is
//! used by the serial interface as a staging area before the value gets
//! placed into the flags in the parameter block by UIControlType().
//
//*****************************************************************************
static unsigned char g_ucControlType = 0;

//*****************************************************************************
//
//! The specification of the type of sensor presence on the motor.  This
//! variable is used by the serial interface as a staging area before the
//! value gets placed into the flags in the parameter block by
//! UISensorType().
//
//*****************************************************************************
static unsigned char g_ucSensorType = 0;


//*****************************************************************************
//
//! The specification of the modulation waveform type for the motor drive.
//! This variable is used by the serial interface as a staging area before the
//! value gets placed into the flags in the parameter block by
//! UIModulationType().
//
//*****************************************************************************
static unsigned char g_ucModulationType = 0;

//*****************************************************************************
//
//! The specification of the motor drive direction.  This variable is used by
//! the serial interface as a staging area before the value gets placed into
//! the flags in the parameter block by UIDirectionSet().
//
//*****************************************************************************
static unsigned char g_ucDirection = 0;

//*****************************************************************************
//
//! The specification of the PWM frequency for the motor drive.  This variable
//! is used by the serial interface as a staging area before the value gets
//! placed into the flags in the parameter block by UIPWMFrequencySet().
//
//*****************************************************************************
static unsigned char g_ucFrequency = 0;

//*****************************************************************************
//
//! The specification of the update rate for the motor drive.  This variable is
//! used by the serial interface as a staging area before the value gets
//! updated in a synchronous manner by UIUpdateRate().
//
//*****************************************************************************
static unsigned char g_ucUpdateRate = 0;

//*****************************************************************************
//
//! The I coefficient of the frequency PI controller.  This variable is used by
//! the serial interface as a staging area before the value gets placed into
//! the parameter block by UIFAdjI().
//
//*****************************************************************************
long g_lFAdjI = 0;

//*****************************************************************************
//
//! The I coefficient of the frequency PI controller for a previous cycle.
//
//*****************************************************************************
long g_lFAdjIPrev = 0;

//*****************************************************************************
//
//! The I coefficient of the power PI controller.  This variable is used by
//! the serial interface as a staging area before the value gets placed into
//! the parameter block by UIPAdjI().
//
//*****************************************************************************
static long g_lPAdjI = 0;

//*****************************************************************************
//
//! A boolean that is true when the on-board user interface should be active
//! and false when it should not be.
//
//*****************************************************************************
static unsigned long g_ulUIUseOnboard = 1;

//*****************************************************************************
//
//! A boolean that is true when dynamic braking should be utilized.  This
//! variable is used by the serial interface as a staging area before the value
//! gets placed into the flags in the parameter block by UIDynamicBrake().
//
//*****************************************************************************
static unsigned char g_ucDynamicBrake = 0;

//*****************************************************************************
//
//! The processor usage for the most recent measurement period.  This is a
//! value between 0 and 100, inclusive.
//
//*****************************************************************************
unsigned char g_ucCPUUsage = 0;

//*****************************************************************************
//
//! A boolean that is true when slow decay mode should be utilized.  This
//! variable is used by the serial interface as a staging area before the value
//! gets placed into the flags in the parameter block by UIDecayMode().
//
//*****************************************************************************
static unsigned char g_ucDecayMode = 1;

//*****************************************************************************
//
//! A 32-bit unsigned value that represents the value of various GPIO signals
//! on the board.  Bit 0 corresponds to CFG0; Bit 1 corresponds to CFG1; Bit
//! 2 correpsonds to CFG2; Bit 8 corresponds to the Encoder A input; Bit 9
//! corresponds to the Encode B input; Bit 10 corresponds to the Encoder
//! Index input.
//
//*****************************************************************************
unsigned long g_ulGPIOData = 0;

//*****************************************************************************
//
//! The Analog Input voltage, specified in millivolts.
//
//*****************************************************************************
short g_usIrrigationVoltage;

//*****************************************************************************
//
//! The Analog Input voltage offset, specified in millivolts.
//
//*****************************************************************************

short g_usIrrigationVoltageOffset;

//*****************************************************************************
//
//! The trigger hall status, bit masked in 4 LSB, set when error.
//
//*****************************************************************************

unsigned char g_ucTriggerHallStatus = 0x00;

//*****************************************************************************
//
//! The flag for integral gain change
//
//*****************************************************************************

volatile unsigned char g_ucIntegralGainChanged = 0x00;

//*****************************************************************************
//
//! This structure instance contains the configuration values for the
//! Brushless DC motor drive.
//
//*****************************************************************************
tDriveParameters g_sParameters =
{
    //
    // The sequence number (ucSequenceNum); this value is not important for
    // the copy in SRAM.
    //
  //  0,

    //
    // The CRC (ucCRC); this value is not important for the copy in SRAM.
    //
    //0,

    //
    // The parameter block version number (ucVersion).
    //
    5,

    //
    // The minimum pulse width (ucMinPulseWidth).
    //
    25,

    //
    // The PWM dead time (ucDeadTime).
    //
    4,

    //
    // The PWM update rate (ucUpdateRate).
    //
    0,

    //
    // The number of poles (ucNumPoles).
    //
    4,

    //
    // The modulation type (ucModulationType).
    //
    MOD_TYPE_SENSORLESS,

    //
    // The acceleration rate (usAccel).
    //
    50000,

    //
    // The deceleration rate (usDecel).
    //
    50000,

    //
    // The minimum motor drive current (sMinCurrent).
    //
    0,

    //
    // The maximum motor drive current (sMaxCurrent).
    //
    15000,

    //
    // The precharge time (ucPrechargeTime).
    //
    3,

    //
    // The maximum ambient microcontroller temperature (ucMaxTemperature).
    //
    52,

    //
    // The flags (usFlags).
    //
    (FLAG_PWM_FREQUENCY_25K |
     (FLAG_DIR_FORWARD << FLAG_DIR_BIT) |
     (FLAG_ENCODER_ABSENT << FLAG_ENCODER_BIT) |
     (FLAG_BRAKE_ON << FLAG_BRAKE_BIT) |
     (FLAG_SENSOR_TYPE_GPIO << FLAG_SENSOR_TYPE_BIT) |
     (FLAG_SENSOR_POLARITY_HIGH << FLAG_SENSOR_POLARITY_BIT) |
     (FLAG_SENSOR_SPACE_120 << FLAG_SENSOR_SPACE_BIT)),


    //
    // The irrigation level (usIrrigationLevel).
    //
    255,

	//The cutting type
	1,
    //
    // The power acceleration (usAccelPower).
    //
    1000,

    //
    // The minimum motor speed (ulMinSpeed).
    //
    3200,

    //
    // The maximum motor speed (ulMaxSpeed).
    //
    12000,

    //
    // The minimum DC bus voltage (ulMinVBus).
    //
    43200,

    //
    // The maximum DC bus voltage (ulMaxVBus).
    //
    52800,

    //
    // The brake engage voltage (ulBrakeOnV).
    //
    50000,

    //
    // The brake disengage voltage (ulBrakeOffV).
    //
    10000,

    //
    // The DC bus voltage at which the deceleration rate is reduced (ulDecelV).
    //
    50000,

    //
    // The frequency adjust P coefficient (lFAdjP).
    //
    (unsigned long)(40000),

    //
    // The frequency adjust I coefficient (lFAdjI).
    //
    (unsigned long)(600),

    //
    // The power adjust P coefficient (lPAdjP).
    //
    (unsigned long)(20000),

    //
    // The brake maximum time (ulBrakeMax).
    //
    60 * 1000,

    //
    // The brake cooling time (ulBrakeCool).
    //
    55 * 1000,

    //
    // The motor current at which the acceleration rate is reduced, specified
    // in milli-amperes (sAccelCurrent).
    //
    15000,

    //
    // The power deceleration (usAccelPower).
    //
    1000,

    //
    // The ethernet connection timeout, specified in seconds
    // (ulConnectionTimeout).
    //
    500,

    //
    // The number of PWM periods to skip in a commuation before looking for
    // the Back EMF zero crossing event (ucBEMFSkipCount).
    //
    3,

    //
    // The closed-loop control target type (ucControlType).
    //
    CONTROL_TYPE_NORMAL,

    //
    // The Back EMF Threshold Voltage for sensorless startup
    // (usSensorlessBEMFThresh).
    //
    1500,

    //
    // The sensorless startup hold time (usStartupCount);
    //
    10,

    //
    // The open-loop sensorless ramp time (usSensorlessRampTime).
    //
    100,

    //
    // The motor current limit for motor operation (sTargetCurrent).
    //
    0,

    //
    // Padding (2 Bytes)
    //
    {0, 0},

    //
    // The starting voltage for sensorless startup (ulSensorlessStartVoltage).
    //
    4500,

    //
    // The ending voltage for sensorless startup (ulSensorlessEndVoltage).
    //
    14500,

    //
    // The starting speed for sensorless startup (ulSensorlessStartSpeed).
    //
    300,

    //
    // The ending speed for sensorless startup (ulSensorlessEndSpeed).
    //
    3000,

    //
    // The minimum motor power (ulMinPower).
    //
    0,

    //
    // The maximum motor power (ulMaxPower).
    //
    10000,

    //
    // The target motor power (ulTargetPower).
    //
    0,

    //
    // The target motor speed (ulTargetSpeed).
    //
    0,

    //
    // The power adjust I coefficient (lPAdjI).
    //
    (unsigned long)(2500),
};

//*****************************************************************************
//
//! The target type for this drive.  This is used by the user interface
//! module.
//
//*****************************************************************************
const unsigned long g_ulUITargetType = RESP_ID_TARGET_BLDC;

//*****************************************************************************
//
//! The ee serial number
//
//*****************************************************************************
char g_usEESerialNumber[UI_EE_DEFAULT_SIZE];

//*****************************************************************************
//
//! The ee origin for handpiece
//
//*****************************************************************************
char g_usEEOrigin[UI_EE_CONST_SIZE];

//*****************************************************************************
//
//! The ee axis for handpiece
//
//*****************************************************************************
char g_usEEAxis[UI_EE_CONST_SIZE];

//*****************************************************************************
//
//! The ee normal for handpiece
//
//*****************************************************************************
char g_usEENormal[UI_EE_CONST_SIZE];


//*****************************************************************************
//
//! The operating time of the handpiece, the unit is in seconds
//
//*****************************************************************************
unsigned long g_ulHPOpTime;

//*****************************************************************************
//
//! The operating time of the handpiece, the unit is in milliseconds ticks
//
//*****************************************************************************
unsigned long g_ulHPOpTicks;

//*****************************************************************************
//
//! The operating time of the handpiece, 4 bytes os data plus 1 byte check sum
//
//*****************************************************************************
char g_usHPOpTimeStr[UI_EE_DEFAULT_SIZE];

//*****************************************************************************
//
//! The error code of handpiece
//
//*****************************************************************************
char g_usHPError[UI_EE_DEFAULT_SIZE];

//*****************************************************************************
//
//! The version of the firmware for handpiece
//
//*****************************************************************************
char g_usFirmwareVersionH[FIRMWARE_VER_LENGTH] = "NA";

volatile char g_ucHPInitStart =0x00;

//*****************************************************************************
//
//! The current speed of the motor's rotor.
//
//*****************************************************************************
unsigned long g_ulRotorSpeed = 0;

//*****************************************************************************
//
//! The reset flag of handpiece.
//
//*****************************************************************************
unsigned char g_ucHPReset = 1;

//*****************************************************************************
//
//! An array of structures describing the Brushless DC motor drive parameters
//! to the Ethernet user interface module.
//
//*****************************************************************************
const tUIParameter g_sUIParameters[] =
{
    //
    // The firmware version.
    //
    {
        PARAM_FIRMWARE_VERSION,
        FIRMWARE_VER_LENGTH,
        0,
        0,
        0,
        (unsigned char *)&g_usFirmwareVersion,
        0
    },

    //
    // The firmware version of handpiece
    //
    {
        PARAM_FIRMWARE_VERSION_H,
        FIRMWARE_VER_LENGTH,
        0,
        0,
        0,
        (unsigned char *)&g_usFirmwareVersionH,
        0
    },

    //
    // The minimum motor speed.  This is specified in RPM, ranging from 0 to
    // 20000 RPM.
    //
    {
        PARAM_MIN_SPEED,
        4,
        0,
        6000,
        1,
        (unsigned char *)&(g_sParameters.ulMinSpeed),
        0
    },

    //
    // The maximum motor speed.  This is specified in RPM, ranging from 0 to
    // 20000 RPM.
    //
    {
        PARAM_MAX_SPEED,
        4,
        0,
        12000,
        1,
        (unsigned char *)&(g_sParameters.ulMaxSpeed),
        0
    },

    //
    // The target motor speed.  This is specified in RPM, ranging from 0 to
    // 20000 RPM.
    //
    {
        PARAM_TARGET_SPEED,
        4,
        0,
        15000,
        1,
        (unsigned char *)&(g_sParameters.ulTargetSpeed),
        0
    },

    //
    // The current motor speed.  This is specified in RPM, ranging from 0 to
    // 20000 RPM.  This is a read-only parameter.
    //
    {
        PARAM_CURRENT_SPEED,
        4,
        0,
        15000,
        0,
        (unsigned char *)&g_ulMeasuredSpeed,
        0
    },

    //
    // The acceleration rate for the motor drive.  This is specified in RPM per
    // second, ranging from 1 RPM/sec to 50000 RPM/sec.
    //
    {
        PARAM_ACCEL,
        2,
        1,
        50000,
        1,
        (unsigned char *)&(g_sParameters.usAccel),
        0
    },

    //
    // The deceleration rate for the motor drive.  This is specified in RPM per
    // second, ranging from 1 RPM/sec to 6000 RPM/sec.
    //
    {
        PARAM_DECEL,
        2,
        1,
        50000,
        1,
        (unsigned char *)&(g_sParameters.usDecel),
        0
    },

    //
    // The type of modulation to be used to drive the motor.  The following
    // values are defined.
    // MOD_TYPE_TRAPEZOID   - 6-point/trapezoid modulation, using Hall sensors
    //                        for position/commutation.
    // MOD_TYPE_SENSORLESS  - 6-point/trapezoid modulation, sensorless, using
    //                        Back EMF for position/commutation.
    // MOD_TYPE_SINE        - Sinusoid modulation, using Hall sensors for
    //                        position.
    //
    {
        PARAM_MODULATION,
        1,
        0,
        2,
        1,
        &g_ucModulationType,
        0
    },

    //
    // The direction of motor rotation.  When the value is zero, the motor is
    // driven in the forward direction.  When the value is one, the motor is
    // driven in the backward direction.
    //
    {
        PARAM_DIRECTION,
        1,
        0,
        1,
        1,
        &g_ucDirection,
        0
    },

    //
    // The PWM frequency to be used.  When this value is zero, the PWM
    // frequency is 8 KHz.  When this value is one, the PWM frequency is
    // 12.5 KHz.  When this value is two, the PWM frequency is 16 KHz.  When
    // this value is three, the PWM frequency is 20 KHz.
    //
    {
        PARAM_PWM_FREQUENCY,
        1,
        0,
        7,
        1,
        &g_ucFrequency,
        UIPWMFrequencySet
    },

    //
    // The dead-time between switching off one side of a motor phase (high or
    // low) and turning on the other.  This is specified in 20 nanoseconds
    // units, ranging from 500 ns to 5100 ns.
    //
    {
        PARAM_PWM_DEAD_TIME,
        1,
        2,
        255,
        1,
        &(g_sParameters.ucDeadTime),
        PWMSetDeadBand
    },

    //
    // The rate at which the PWM duty cycles are updated.  This is specified in
    // PWM periods, ranging from 1 to 256.
    //
    {
        PARAM_PWM_UPDATE,
        1,
        0,
        255,
        1,
        &g_ucUpdateRate,
        UIUpdateRate
    },

    //
    // The minimum PWM pulse width.  This is specified in 1/10ths of a
    // microsecond, ranging from 0 us to 25 us.
    //
    {
        PARAM_PWM_MIN_PULSE,
        1,
        0,
        250,
        1,
        &(g_sParameters.ucMinPulseWidth),
        PWMSetMinPulseWidth
    },

    //
    // The fault status.
    //
    {
        PARAM_CLEAR_FAULT,
        4,
        0,
        0xFFFFFFFF,
        1,
        (unsigned char *)&g_ulFaultFlags,
        MainClearFaults
    },

    //
    // The irrigation level (0~255).
    //
    {
        PARAM_IRR_LEVEL,
        2,
        0,
        256,
        1,
        (unsigned char *)&(g_sParameters.usIrrigationLevel),
        UISetIrrigationLevel
    },
	//The cutting type (0 for handpiece, 1 for footpedal)

	  {
	        PARAM_CUT_TYPE,
	        1,
	         0,
	         1,
	         1,
	         (unsigned char *)&(g_sParameters.usCutType),
	        0
	 	    },
    //
    // The minimum allowable drive current during operation.  This is specified
    // in milli-amperes, ranging from 0 to 10A.
    //
    {
        PARAM_MIN_CURRENT,
        2,
        0,
        15000,
        100,
        (unsigned char *)&(g_sParameters.sMinCurrent),
        0
    },

    //
    // The maximum allowable drive current during operation.  This is specified
    // in milli-amperes, ranging from 0 to 10A.
    //
    {
        PARAM_MAX_CURRENT,
        2,
        0,
        15000,
        100,
        (unsigned char *)&(g_sParameters.sMaxCurrent),
        0
    },

    //
    // The target drive current during operation.  This is specified
    // in milli-amperes, ranging from 0 to 10A.
    //
    {
        PARAM_TARGET_CURRENT,
        2,
        0,
        15000,
        100,
        (unsigned char *)&(g_sParameters.sTargetCurrent),
        0
    },

    //
    // The minimum allowable bus voltage during operation.  This is specified
    // in millivolts, ranging from 1 V to 40 V.
    //
    {
        PARAM_MIN_BUS_VOLTAGE,
        4,
        0,
        50000,
        100,
        (unsigned char *)&(g_sParameters.ulMinVBus),
        0
    },

    //
    // The maximum allowable bus voltage during operation.  This is specified
    // in millivolts, ranging from 1 V to 40 V.
    //
    {
        PARAM_MAX_BUS_VOLTAGE,
        4,
        0,
        50000,
        100,
        (unsigned char *)&(g_sParameters.ulMaxVBus),
        0
    },

    //
    // The P coefficient for the frequency adjust PI controller.
    //
    {
        PARAM_SPEED_P,
        4,
        0x80000000,
        0x7fffffff,
        1,
        (unsigned char *)&(g_sParameters.lFAdjP),
        0
    },

    //
    // The I coefficient for the frequency adjust PI controller.
    //
    {
        PARAM_SPEED_I,
        4,
        0x80000000,
        0x7fffffff,
        1,
        (unsigned char *)&g_lFAdjI,
        UIFAdjI
    },

    //
    // The voltage at which the brake circuit is applied.  This is specified in
    // millivolts, ranging from 1 V to 40 V.
    //
    {
        PARAM_BRAKE_ON_VOLTAGE,
        4,
        1000,
        60000,
        100,
        (unsigned char *)&(g_sParameters.ulBrakeOnV),
        0
    },

    //
    // The voltage at which the brake circuit is disengaged.  This is specified
    // in millivolts, ranging from 1 V to 40 V.
    //
    {
        PARAM_BRAKE_OFF_VOLTAGE,
        4,
        1000,
        60000,
        100,
        (unsigned char *)&(g_sParameters.ulBrakeOffV),
        0
    },

    //
    // This indicates if the on-board user interface should be utilized.  When
    // one, the on-board user interface is active, and when zero it is not.
    //
    {
        PARAM_USE_ONBOARD_UI,
        1,
        0,
        1,
        1,
        (unsigned char *)&g_ulUIUseOnboard,
        0
    },

    //
    // The amount of time to precharge the bootstrap capacitor on the high side
    // gate driver before starting the motor drive, specified in milliseconds.
    //
    {
        PARAM_PRECHARGE_TIME,
        1,
        0,
        255,
        1,
        &(g_sParameters.ucPrechargeTime),
        0
    },

    //
    // This indicates if dynamic braking should be utilized.  When one, dynamic
    // braking is active, and when zero it is not.
    //
    {
        PARAM_USE_DYNAM_BRAKE,
        1,
        0,
        1,
        1,
        &g_ucDynamicBrake,
        UIDynamicBrake
    },

    //
    // The maximum amount of time to apply dynamic braking, specified in
    // milliseconds.
    //
    {
        PARAM_MAX_BRAKE_TIME,
        4,
        0,
        60 * 1000,
        1,
        (unsigned char *)&(g_sParameters.ulBrakeMax),
        0
    },

    //
    // The time at which dynamic braking can be reapplied after entering its
    // cooling mode, specified in milliseconds.  Note that the cooling time is
    // the maximum braking time minus this parameter.
    //
    {
        PARAM_BRAKE_COOL_TIME,
        4,
        0,
        60 * 1000,
        1,
        (unsigned char *)&(g_sParameters.ulBrakeCool),
        0
    },

    //
    // The fault status flags.
    //
    {
        PARAM_FAULT_STATUS,
        4,
        0,
        0xffffffff,
        1,
        (unsigned char *)&g_ulFaultFlags,
        0
    },

    //
    // The motor status.
    //
    {
        PARAM_MOTOR_STATUS,
        1,
        0,
        0,
        0,
        &g_ucMotorStatus,
        0
    },

    //
    // The voltage at which the deceleration rate is reduced.  This is
    // specified in volts, raning from 1 V to 40 V.
    //
    {
        PARAM_DECEL_VOLTAGE,
        4,
        0,
        50000,
        100,
        (unsigned char *)&(g_sParameters.ulDecelV),
        0
    },

    //
    // The maximum allowable ambient temperature.  This is specified in degrees
    // Celsius, ranging from 0 to 85 C.
    //
    {
        PARAM_MAX_TEMPERATURE,
        1,
        0,
        85,
        1,
        &(g_sParameters.ucMaxTemperature),
        0
    },

    //
    // The motor current at which the acceleration rate is reduced.  This is
    // specified in milli-amperes, ranging from 0 A to 10 A.
    //
    {
        PARAM_ACCEL_CURRENT,
        2,
        0,
        15000,
        100,
        (unsigned char *)&(g_sParameters.sAccelCurrent),
        0
    },

    //
    // The current decay mode.
    //
    {
        PARAM_DECAY_MODE,
        1,
        0,
        1,
        1,
        &g_ucDecayMode,
        UIDecayMode,
    },

    //
    // The current value of the GPIO data input(s).
    //
    {
        PARAM_GPIO_DATA,
        4,
        0,
        0xffffffff,
        1,
        (unsigned char *)&g_ulGPIOData,
        0,
    },

    //
    // The current number of packets received on the Ethernet interface.
    //
    {
        PARAM_ETH_RX_COUNT,
        4,
        0,
        0xffffffff,
        1,
        (unsigned char *)&g_ulEthernetRXCount,
        0,
    },

    //
    // The current number of packets transmitted on the Ethernet interface.
    //
    {
        PARAM_ETH_TX_COUNT,
        4,
        0,
        0xffffffff,
        1,
        (unsigned char *)&g_ulEthernetTXCount,
        0,
    },

    //
    // The Ethernet TCP Connection Timeout.
    //
    {
        PARAM_ETH_TCP_TIMEOUT,
        4,
        0,
        0xffffffff,
        1,
        (unsigned char *)&g_sParameters.ulConnectionTimeout,
        UIConnectionTimeout,
    },

    //
    // The skip count for Back EMF zero crossing detection hold-off.
    //
    {
        PARAM_BEMF_SKIP_COUNT,
        1,
        1,
        100,
        1,
        &(g_sParameters.ucBEMFSkipCount),
        0,
    },

    //
    // The startup count for sensorless mode.
    //
    {
        PARAM_STARTUP_COUNT,
        2,
        0,
        0xffff,
        1,
        (unsigned char *)&(g_sParameters.usStartupCount),
        0,
    },

    //
    // The starting voltage for sensorless startup.
    //
    {
        PARAM_STARTUP_STARTV,
        4,
        0,
        50000,
        1,
        (unsigned char *)&(g_sParameters.ulSensorlessStartVoltage),
        0,
    },

    //
    // The ending voltage for sensorless startup.
    //
    {
        PARAM_STARTUP_ENDV,
        4,
        0,
        50000,
        1,
        (unsigned char *)&(g_sParameters.ulSensorlessEndVoltage),
        0,
    },

    //
    // The starting speed for sensorless startup.
    //
    {
        PARAM_STARTUP_STARTSP,
        4,
        0,
        60000,
        1,
        (unsigned char *)&(g_sParameters.ulSensorlessStartSpeed),
        0,
    },

    //
    // The ending speed for sensorless startup.
    //
    {
        PARAM_STARTUP_ENDSP,
        4,
        0,
        60000,
        1,
        (unsigned char *)&(g_sParameters.ulSensorlessEndSpeed),
        0
    },

    //
    // The target motor power.  This is specified in milliWatts, ranging from
    // 0 to 360 W.
    //
    {
        PARAM_TARGET_POWER,
        4,
        0,
        360000,
        1,
        (unsigned char *)&(g_sParameters.ulTargetPower),
        0
    },
    
    //
    // The minimum motor power.  This is specified in milliwats, ranging from
    // 0 to 360 W.
    //
    {
        PARAM_MIN_POWER,
        4,
        0,
        360000,
        1,
        (unsigned char *)&(g_sParameters.ulMinPower),
        0
    },

    //
    // The maximum motor power.  This is specified in milliwats ranging from
    // 0 to 360 W.
    //
    {
        PARAM_MAX_POWER,
        4,
        0,
        360000,
        1,
        (unsigned char *)&(g_sParameters.ulMaxPower),
        0
    },

    //
    // The P coefficient for the power adjust PI controller.
    //
    {
        PARAM_POWER_P,
        4,
        0x80000000,
        0x7fffffff,
        1,
        (unsigned char *)&(g_sParameters.lPAdjP),
        0
    },

    //
    // The I coefficient for the power adjust PI controller.
    //
    {
        PARAM_POWER_I,
        4,
        0x80000000,
        0x7fffffff,
        1,
        (unsigned char *)&g_lPAdjI,
        0
    },

    //
    // The power acceleration rate for the motor drive.  This is specified in
    // milliwatts per second, ranging from 1 mW/sec to 50000 mW/sec.
    //
    {
        PARAM_ACCEL_POWER,
        2,
        1,
        50000,
        1,
        (unsigned char *)&(g_sParameters.usAccelPower),
        0
    },

    //
    // The power deceleration rate for the motor drive.  This is specified in
    // milliwatts per second, ranging from 1 mW/sec to 50000 mW/sec.
    //
    {
        PARAM_DECEL_POWER,
        2,
        1,
        50000,
        1,
        (unsigned char *)&(g_sParameters.usDecelPower),
        0
    },

    //
    // The control mode for the motor (speed/power).
    //
    {
        PARAM_CONTROL_MODE,
        1,
        0,
        1,
        1,
        &g_ucControlType,
        UIControlType
    },

    //
    // The ending speed for sensorless startup.
    //
    {
        PARAM_STARTUP_RAMP,
        2,
        0,
        0xffff,
        1,
        (unsigned char *)&(g_sParameters.usSensorlessRampTime),
        0,
    },

    //
    // The Back EMF Threshold Voltage for sensorless startup, specified in
    // millivolts.
    //
    {
        PARAM_STARTUP_THRESH,
        2,
        0,
        0xffff,
        1,
        (unsigned char *)&(g_sParameters.usSensorlessBEMFThresh),
        0,
    },

    //
    // The EE origin of handpiece, single precision binary data, specified in mm.
    // the last byte is the checksum of the constants
    //
    {
        PARAM_HP_EE_ORIGIN,
        UI_EE_CONST_SIZE,
        0,
        0,
        1,
        (unsigned char *)&g_usEEOrigin,
        UISetEEOrigin,
    },

    //
    // The EE origin of handpiece, single precision binary data, specified in mm.
    // the last byte is the checksum of the constants
    //
    {
        PARAM_HP_EE_AXIS,
        UI_EE_CONST_SIZE,
        0,
        0,
        1,
        (unsigned char *)&g_usEEAxis,
        UISetEEAxis,
    },

    //
    // The EE origin of handpiece, single precision binary data, specified in mm.
    // the last byte is the checksum of the constants
    //
    {
        PARAM_HP_EE_NORMAL,
        UI_EE_CONST_SIZE,
        0,
        0,
        1,
        (unsigned char *)&g_usEENormal,
        UISetEENormal,
    },


    //
    // The EE serial number of handpiece, 4 byte binary string plus 1 byte check sum
    //
    {
        PARAM_HP_EE_SERIAL,
        UI_EE_DEFAULT_SIZE,
        0,
        0,
        1,
        (unsigned char *)&g_usEESerialNumber,
        UISetEESerialNumber,
    },

    //
    // The reset handpiece flag, 1 for reset
    //
    {
        PARAM_HP_RESET,
        1,
        0,
        0,
        1,
        (unsigned char *)&g_ucHPReset,
        UIResetHandPiece,
    },
};

//*****************************************************************************
//
//! The number of motor drive parameters.  This is used by the user
//! interface module.
//
//*****************************************************************************
const unsigned long g_ulUINumParameters = (sizeof(g_sUIParameters) /
                                           sizeof(g_sUIParameters[0]));

//*****************************************************************************
//
//! An array of structures describing the Brushless DC motor drive real-time
//! data items to the serial user interface module.
//
//*****************************************************************************
const tUIRealTimeData g_sUIRealTimeData[] =
{
    //
    // The current through phase A of the motor.  This is a signed 16-bit
    // value providing the current in milli-amperes.
    //
    {
        DATA_PHASE_A_CURRENT,
        2,
        (unsigned char *)&(g_psPhaseCurrent[0])
    },

    //
    // The current through phase B of the motor.  This is a signed 16-bit
    // value providing the current in milli-amperes.
    //
    {
        DATA_PHASE_B_CURRENT,
        2,
        (unsigned char *)&(g_psPhaseCurrent[1])
    },

    //
    // The current through phase C of the motor.  This is a signed 16-bit
    // value providing the current in milli-amperes.
    //
    {
        DATA_PHASE_C_CURRENT,
        2,
        (unsigned char *)&(g_psPhaseCurrent[2])
    },

    //
    // The current through the entire motor.  This is a signed 16-bit
    // value providing the current in milli-amperes.
    //
    {
        DATA_MOTOR_CURRENT,
        2,
        (unsigned char *)&g_sMotorCurrent
    },

    //
    // The voltage of the DC bus.  This is a 32-bit value providing the voltage
    // in milli-volts.
    //
    {
        DATA_BUS_VOLTAGE,
        4,
        (unsigned char *)&g_ulBusVoltage
    },

    //
    // The frequency of the rotor.  This is a 16-bit value providing the
    // motor speed in RPM.
    //
    {
        DATA_ROTOR_SPEED,
        4,
        (unsigned char *)&g_ulMeasuredSpeed
    },

    //
    // The processor usage.  This is an 8-bit value providing the percentage
    // between 0 and 100.
    //
    {
        DATA_PROCESSOR_USAGE,
        1,
        &g_ucCPUUsage
    },

    //
    // The state of the motor drive.
    //
    {
        DATA_MOTOR_STATUS,
        1,
        &g_ucMotorStatus
    },

    //
    // The direction motor is running.
    //
    {
    	DATA_DIRECTION,
        1,
        &g_ucDirection
    },

    //
    // The fault status flags.
    //
    {
        DATA_FAULT_STATUS,
        4,
        (unsigned char *)&g_ulFaultFlags
    },

    //
    // The ambient temperature of the microcontroller.  This is an 8-bit value
    // providing the temperature in Celsius.
    //
    {
        DATA_TEMPERATURE,
        2,
        (unsigned char *)&g_sAmbientTemp
    },

    //
    // The analog input voltage.  This is a 16-bit value providing the analog
    // input voltage in milli-volts.
    //
    {
        DATA_ANALOG_INPUT,
        2,
        (unsigned char *)&g_usIrrigationVoltage
    },

    //
    // The pwm duty cycle of the drive.
    //
    {
        DATA_MOTOR_PWM,
        4,
        (unsigned char *)&g_ulDutyCycle
    },

    //
    // The trigger information, there are four hall sensors, each is a 16 bit integer
    //
    {
        DATA_TRIGGER_INFO,
        8,
        (unsigned char *)&g_ulRxDataInt[1]
    },

    //
    // The direction hall information, there are 2 hall sensors, each is a 16 bit integer
    //
    {
        DATA_DIR_HALL_INFO,
        4,
        (unsigned char *)&g_ulRxDataInt[5]
    },

    //
    // The commanded motor speed in RPM.
    //
    {
        DATA_ROTOR_SPEED_CMD,
        4,
        (unsigned char *)&(g_sParameters.ulTargetSpeed)
    },
};

//*****************************************************************************
//
//! The number of motor drive real-time data items.  This is used by the serial
//! user interface module.
//
//*****************************************************************************
const unsigned long g_ulUINumRealTimeData = (sizeof(g_sUIRealTimeData) /
                                             sizeof(g_sUIRealTimeData[0]));

//*****************************************************************************
//
//! An array of structures describing the on-board switches.
//
//*****************************************************************************
const tUIOnboardSwitch g_sUISwitches[] =
{
    //
    // The run/stop/mode button.  Pressing the button will cycle between
    // stopped and running, and holding the switch for five seconds will toggle
    // between sine wave and space vector modulation.
    //
    {
        PIN_SWITCH_PIN_BIT,
        UI_INT_RATE * 5,
        UIButtonPress,
        0,
        UIButtonHold
    }
};

//*****************************************************************************
//
//! The number of switches in the g_sUISwitches array.  This value is
//! automatically computed based on the number of entries in the array.
//
//*****************************************************************************
#define NUM_SWITCHES            (sizeof(g_sUISwitches) /   \
                                 sizeof(g_sUISwitches[0]))

//*****************************************************************************
//
//! The number of switches on this target.  This value is used by the on-board
//! user interface module.
//
//*****************************************************************************
const unsigned long g_ulUINumButtons = NUM_SWITCHES;

//*****************************************************************************
//
//! This is the count of the number of samples during which the switches have
//! been pressed; it is used to distinguish a switch press from a switch
//! hold.  This array is used by the on-board user interface module.
//
//*****************************************************************************
unsigned long g_pulUIHoldCount[NUM_SWITCHES];

//*****************************************************************************
//
//! This is the board id, read once from the configuration switches at startup.
//
//*****************************************************************************
unsigned char g_ucBoardID = 0;

//*****************************************************************************
//
//! The running count of system clock ticks.
//
//*****************************************************************************
static unsigned long g_ulUITickCount = 0;

char rxData[64];
unsigned short g_ulRxDataInt[7];
static int g_ucSpeedThrottle = 0;
static int g_triggerInfo=0;
static int g_ucTIndexPrev = 0;
unsigned long g_ucInitHallReading[6];
unsigned int g_ucHallMin[UI_NUM_HALLS];
unsigned int g_ucHallMax[UI_NUM_HALLS];
unsigned long hallReadingSum;
unsigned int handHallSpdPole;
char tStr[32];
volatile char g_ucHPInitDone = 0x00;
volatile char g_ucUpdateOpTime = 0x00;
volatile char g_ucState = 0x00;
volatile unsigned char g_ucDataComplete = 1;
volatile int initReadingDone =0;
volatile short cutterEnableStatus = 0; //disabled
volatile short cutterOverrideStatus = 0;
volatile long expandioAStatus = 0xffff;


//*****************************************************************************
//
//! Updates the Ethernet TCP connection timeout.
//!
//! This function is called when the variable controlling the presence of an
//! encoder is updated.  The value is then reflected into the usFlags member
//! of #g_sParameters.
//!
//! \return None.
//
//*****************************************************************************
static void
UIConnectionTimeout(void)
{
    //
    // Update the encoder presence flag in the flags variable.
    //
    g_ulConnectionTimeoutParameter = g_sParameters.ulConnectionTimeout;
}

//*****************************************************************************
//
//! Updates the irrigation level.
//!
//! This function is called when the variable controlling the irrigation level
//! is updated.
//!
//! \return None.
//
//*****************************************************************************
static void
UISetIrrigationLevel(void)
{
	if(g_sParameters.usIrrigationLevel > 0)
	    IrrSetLevel(g_sParameters.usIrrigationLevel);
}

//*****************************************************************************
//
//! Reset Handpiece.
//!
//! This function is called when the variable reset handpiece is updated.
//!
//! \return None.
//
//*****************************************************************************
static void
UIResetHandPiece(void)
{
   // a place holder
}



//*****************************************************************************
//
//! Updates the control mode bit for the motor dive.
//!
//! This function is called when the variable controlling the motor control
//! variable (speed/power) is updated.  The value is then reflected into the
//! usFlags member of #g_sParameters.
//!
//! \return None.
//
//*****************************************************************************
static void
UIControlType(void)
{
    //
    // See if the motor drive is running.
    //
    if(MainIsRunning())
    {
        //
        // Not allowed to change control type while motor is running.
        //
        g_ucControlType = g_sParameters.ucControlType;

        //
        // There is nothing further to do.
        //
        return;
    }

    //
    // Update the encoder presence flag in the flags variable.
    //
    g_sParameters.ucControlType = g_ucControlType;
}

//*****************************************************************************
//
//! Updates the motor drive direction bit.
//!
//! This function is called when the variable controlling the motor drive
//! direction is updated.  The value is then reflected into the usFlags
//! member of #g_sParameters.
//!
//! \return None.
//
//*****************************************************************************
static void
UIDirectionSet(void)
{
    //
    // Update the direction flag in the flags variable.
    //
    HWREGBITH(&(g_sParameters.usFlags), FLAG_DIR_BIT) = g_ucDirection;

    //
    // Change the direction of the motor drive.
    //
    MainSetDirection(g_ucDirection ? false : true);
}

//*****************************************************************************
//
//! Updates the PWM frequency of the motor drive.
//!
//! This function is called when the variable controlling the PWM frequency of
//! the motor drive is updated.  The value is then reflected into the usFlags
//! member of #g_sParameters.
//!
//! \return None.
//
//*****************************************************************************
static void
UIPWMFrequencySet(void)
{
    //
    // See if the motor drive is running.
    //
    if(MainIsRunning())
    {
        //
        // The modulation type can not changed when the motor drive is running
        // (that could be catastrophic!), so revert the modulation type
        // variable back to the value in the flags.
        //
        g_ucFrequency = g_sParameters.usFlags & FLAG_PWM_FREQUENCY_MASK;
        if(g_ucFrequency > 3)
        {
            g_ucFrequency = (g_ucFrequency & 0x3) + 4;
        }

        //
        // There is nothing further to do.
        //
        return;
    }

    //
    // Map the UI parameter value to actual frequency value.
    //
    switch(g_ucFrequency)
    {
        case 0:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_8K);
            break;

        case 1:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_12K);
            break;

        case 2:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_16K);
            break;

        case 4:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_25K);
            break;

        case 5:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_40K);
            break;

        case 6:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_50K);
            break;

        case 7:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_80K);
            break;

        case 3:
        default:
            g_sParameters.usFlags =
                ((g_sParameters.usFlags & ~FLAG_PWM_FREQUENCY_MASK) |
                 FLAG_PWM_FREQUENCY_20K);
            break;
    }

    //
    // Change the PWM frequency.
    //
    MainSetPWMFrequency();
}

//*****************************************************************************
//
//! Sets the update rate of the motor drive.
//!
//! This function is called when the variable specifying the update rate of the
//! motor drive is updated.  This allows the motor drive to perform a
//! synchronous change of the update rate to avoid discontinuities in the
//! output waveform.
//!
//! \return None.
//
//*****************************************************************************
static void
UIUpdateRate(void)
{
    //
    // Set the update rate of the motor drive.
    //
    PWMSetUpdateRate(g_ucUpdateRate);
}

//*****************************************************************************
//
//! Updates the I coefficient of the frequency PI controller.
//!
//! This function is called when the variable containing the I coefficient of
//! the frequency PI controller is updated.  The value is then reflected into
//! the parameter block.
//!
//! \return None.
//
//*****************************************************************************
static void
UIFAdjI(void)
{
    //
    // Update the frequency PI controller.
    //
    MainUpdateFAdjI(g_lFAdjI);
}

//*****************************************************************************
//
//! Updates the dynamic brake bit of the motor drive.
//!
//! This function is called when the variable controlling the dynamic braking
//! is updated.  The value is then reflected into the usFlags member of
//! #g_sParameters.
//!
//! \return None.
//
//*****************************************************************************
static void
UIDynamicBrake(void)
{
    //
    // Update the dynamic braking flag in the flags variable.
    //
    HWREGBITH(&(g_sParameters.usFlags), FLAG_BRAKE_BIT) = g_ucDynamicBrake;
}

//*****************************************************************************
//
//! Updates the decay mode bit of the motor drive.
//!
//! This function is called when the variable controlling the decay mode
//! is updated.  The value is then reflected into the usFlags member of
//! #g_sParameters.
//!
//! \return None.
//
//*****************************************************************************
static void
UIDecayMode(void)
{
    //
    // Update the decay mode flag in the flags variable.
    //
    HWREGBITH(&(g_sParameters.usFlags), FLAG_DECAY_BIT) = g_ucDecayMode;
}

//*****************************************************************************
//
//! Updates the ee serial number to the handpiece eeprom.
//!
//! This function is called when ee serial number is updated.
//!
//! \return None.
//
//*****************************************************************************
static void UISetEESerialNumber(void)
{
	int i;
	unsigned char crc=0;

	//while motor is running, do not change
	if(MainIsRunning())
	{
		return;
	}
	//prepare the header
	tStr[0] = 0xff;
	tStr[1] = 0x0a;
	tStr[2] = 0x83;
	tStr[3] = 0x00;

	//check checksum
	for(i=0; i< UI_EE_DEFAULT_SIZE-1; i++)
    crc = crc8_add(g_usEESerialNumber[i], crc);

	if(crc != g_usEESerialNumber[UI_EE_DEFAULT_SIZE-1])
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	//copy the data over
	memcpy(&(tStr[4]),g_usEESerialNumber, UI_EE_DEFAULT_SIZE);

	//save the constants to eeprom in handpiece
	if(ui_uart_ucmd(tStr, UI_EE_DEFAULT_SIZE + 4) ==-1)
    {
		MainSetFault(FAULT_HP_COMM);
		return;
    }

	//reset initialization done flag
	g_ucHPInitDone = 0x00;

}

//*****************************************************************************
//
//! Updates the ee origin to the handpiece eeprom.
//!
//! This function is called when ee origin is updated.
//!
//! \return None.
//
//*****************************************************************************
static void UISetEEOrigin(void)
{
	int i;
	unsigned char crc=0;

	//while motor is running, do not change
	if(MainIsRunning())
	{
		return;
	}
	//prepare the header
	tStr[0] = 0xff;
	tStr[1] = 0x12;
	tStr[2] = 0x83;
	tStr[3] = 0x01;

	//check checksum
	for(i=0; i< UI_EE_CONST_SIZE-1; i++)
    crc = crc8_add(g_usEEOrigin[i], crc);

	if(crc != g_usEEOrigin[UI_EE_CONST_SIZE-1])
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	//copy the data over
	memcpy(&(tStr[4]),g_usEEOrigin, UI_EE_CONST_SIZE);

	//save the constants to eeprom in handpiece
	if(ui_uart_ucmd(tStr, UI_EE_CONST_SIZE + 4) ==-1)
    {
		MainSetFault(FAULT_HP_COMM);
		return;
    }

	//reset initialization done flag
	g_ucHPInitDone = 0x00;

}

//*****************************************************************************
//
//! Updates the ee axis to the handpiece eeprom.
//!
//! This function is called when ee origin is updated.
//!
//! \return None.
//
//*****************************************************************************
static void UISetEEAxis(void)
{
	int i;
	unsigned char crc=0;

	//while motor is running, do not change
	if(MainIsRunning())
	{
		return;
	}

	//prepare the header
	tStr[0] = 0xff;
	tStr[1] = 0x12;
	tStr[2] = 0x83;
	tStr[3] = 0x02;

	//check checksum
	for(i=0; i< UI_EE_CONST_SIZE-1; i++)
    crc = crc8_add(g_usEEAxis[i], crc);

	if(crc != g_usEEAxis[UI_EE_CONST_SIZE-1])
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	//copy the data over
	memcpy(&(tStr[4]), g_usEEAxis, UI_EE_CONST_SIZE);

	//save the constants to eeprom in handpiece
	if(ui_uart_ucmd(tStr, UI_EE_CONST_SIZE + 4) ==-1)
    {
		MainSetFault(FAULT_HP_COMM);
		return;
    }

	//reset initialization done flag
	g_ucHPInitDone = 0x00;
}

//*****************************************************************************
//
//! Updates the ee normal to the handpiece eeprom.
//!
//! This function is called when ee origin is updated.
//!
//! \return None.
//
//*****************************************************************************
static void UISetEENormal(void)
{
	int i;
	unsigned char crc=0;

	//while motor is running, do not change
	if(MainIsRunning())
	{
		return;
	}

	//prepare the header
	tStr[0] = 0xff;
	tStr[1] = 0x12;
	tStr[2] = 0x83;
	tStr[3] = 0x15;

	//check checksum
	for(i=0; i< UI_EE_CONST_SIZE-1; i++)
    crc = crc8_add(g_usEENormal[i], crc);

	if(crc != g_usEENormal[UI_EE_CONST_SIZE-1])
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	//copy the data over
	memcpy(&(tStr[4]), g_usEENormal, UI_EE_CONST_SIZE);

	//save the constants to eeprom in handpiece
	if(ui_uart_ucmd(tStr, UI_EE_CONST_SIZE + 4) ==-1)
    {
		MainSetFault(FAULT_HP_COMM);
		return;
    }

	//reset initialization done flag
	g_ucHPInitDone = 0x00;
}

//*****************************************************************************
//
//! Starts the motor drive.
//!
//! This function is called by the serial user interface when the run command
//! is received.  The motor drive will be started as a result; this is a no
//! operation if the motor drive is already running.
//!
//! \return None.
//
//*****************************************************************************
void
UIRun(void)
{
	GPIOPinWrite(PIN_LEDRUN_PORT, PIN_LEDRUN_PIN, PIN_LEDRUN_PIN);
}

//*****************************************************************************
//
//! Stops the motor drive.
//!
//! This function is called by the serial user interface when the stop command
//! is received.  The motor drive will be stopped as a result; this is a no
//! operation if the motor drive is already stopped.
//!
//! \return None.
//
//*****************************************************************************
void
UIStop(void)
{
    //
    // Stop the motor drive.
    //
    MainStop();
}

//*****************************************************************************
//
//! Emergency stops the motor drive.
//!
//! This function is called by the serial user interface when the emergency
//! stop command is received.
//!
//! \return None.
//
//*****************************************************************************
void
UIEmergencyStop(void)
{
    //
    // Emergency stop the motor drive.
    //
    MainEmergencyStop();

    //
    // Indicate that the emergency stop fault has occurred.
    //
    MainSetFault(FAULT_EMERGENCY_STOP);
}

//*****************************************************************************
//
//! Loads the motor drive parameter block from flash.
//!
//! This function is called by the serial user interface when the load
//! parameter block function is called.  If the motor drive is running, the
//! parameter block is not loaded (since that may result in detrimental
//! changes, such as changing the motor drive from sine to trapezoid).
//! If the motor drive is not running and a valid parameter block exists in
//! flash, the contents of the parameter block are loaded from flash.
//!
//! \return None.
//
//*****************************************************************************
void
UIParamLoad(void)
{
    unsigned char *pucBuffer;
    unsigned long ulIdx;

    //
    // Return without doing anything if the motor drive is running.
    //
    if(MainIsRunning())
    {
        return;
    }

    //
    // Get a pointer to the latest parameter block in flash.
    //
    pucBuffer = FlashPBGet();

    //
    // See if a parameter block was found in flash.
    //
    if(pucBuffer)
    {
        //
        // Loop through the words of the parameter block to copy its contents
        // from flash to SRAM.
        //
        for(ulIdx = 0; ulIdx < (sizeof(tDriveParameters) / 4); ulIdx++)
        {
            ((unsigned long *)&g_sParameters)[ulIdx] =
                ((unsigned long *)pucBuffer)[ulIdx];
        }
    }

    //
    // Set the local variables (used by the serial interface) based on the
    // values in the parameter block values.
    //
    g_ucControlType = g_sParameters.ucControlType;
    g_ucModulationType = g_sParameters.ucModulationType;
    g_ucDirection = HWREGBITH(&(g_sParameters.usFlags), FLAG_DIR_BIT);
    g_ucFrequency = g_sParameters.usFlags & FLAG_PWM_FREQUENCY_MASK;
    if(g_ucFrequency > 3)
    {
        g_ucFrequency = (g_ucFrequency & 0x3) + 4;
    }
    g_ucUpdateRate = g_sParameters.ucUpdateRate;
    g_lFAdjI = g_sParameters.lFAdjI;
	g_lFAdjIPrev = g_lFAdjI;
    g_lPAdjI = g_sParameters.lPAdjI;
    g_ucDynamicBrake = HWREGBITH(&(g_sParameters.usFlags), FLAG_BRAKE_BIT);
    g_ucSensorType = HWREGBITH(&(g_sParameters.usFlags), FLAG_SENSOR_TYPE_BIT);
    g_ucSensorType |= (HWREGBITH(&(g_sParameters.usFlags),
                                 FLAG_SENSOR_SPACE_BIT) << 1);



    g_ucDecayMode =  HWREGBITH(&(g_sParameters.usFlags), FLAG_DECAY_BIT);

    //
    // Loop through all of the parameters.
    //
    for(ulIdx = 0; ulIdx < g_ulUINumParameters; ulIdx++)
    {
    	// id number greater than 0x4E do not need call back on initialization
    	// the initialization is done in a different time.
        //
        // If there is an update function for this parameter, then call it now
        // since the parameter value may have changed as a result of the load.
        //
        if(g_sUIParameters[ulIdx].pfnUpdate && g_sUIParameters[ulIdx].ucID < 0x4F)
        {
            g_sUIParameters[ulIdx].pfnUpdate();
        }
    }
}

//*****************************************************************************
//
//! Saves the motor drive parameter block to flash.
//!
//! This function is called by the serial user interface when the save
//! parameter block function is called.  The parameter block is written to
//! flash for use the next time a load occurs (be it from an explicit request
//! or a power cycle of the drive).
//!
//! \return None.
//
//*****************************************************************************
void
UIParamSave(void)
{
    //
    // Return without doing anything if the motor drive is running.
    //
    if(MainIsRunning())
    {
        return;
    }

    //
    // Save the parameter block to flash.
    //
    FlashPBSave((unsigned char *)&g_sParameters);
}


//*****************************************************************************
//
//! Handles button presses.
//!
//! This function is called when a press of the on-board push button has been
//! detected.  If the motor drive is running, it will be stopped.  If it is
//! stopped, the direction will be reversed and the motor drive will be
//! started.
//!
//! \return None.
//
//*****************************************************************************
void
UIButtonPress(void)
{
    //
    // See if the motor drive is running.
    //
    if(MainIsRunning())
    {
        //
        // Stop the motor drive.
        //
        MainStop();
    }
    else
    {
        //
        // Reverse the motor drive direction.
        //
        g_ucDirection ^= 1;
        UIDirectionSet();

        //
        // Start the motor drive.
        //
        MainRun();
    }
}

//*****************************************************************************
//
//! Handles button holds.
//!
//! This function is called when a hold of the on-board push button has been
//! detected.  The modulation type of the motor will be toggled between sine
//! wave and space vector modulation, but only if a three phase motor is in
//! use.
//!
//! \return None.
//
//*****************************************************************************
static void
UIButtonHold(void)
{
    //
    // Toggle the modulation type.  UIModulationType() will take care of
    // forcing sine wave modulation for single phase motors.
    //
    //g_ucModulation ^= 1;
    //UIModulationType();
}

//*****************************************************************************
//
//! Sets the blink rate for an LED.
//!
//! \param ulIdx is the number of the LED to configure.
//! \param usRate is the rate to blink the LED.
//! \param usPeriod is the amount of time to turn on the LED.
//!
//! This function sets the rate at which an LED should be blinked.  A blink
//! period of zero means that the LED should be turned off, and a blink period
//! equal to the blink rate means that the LED should be turned on.  Otherwise,
//! the blink rate determines the number of user interface interrupts during
//! the blink cycle of the LED, and the blink period is the number of those
//! user interface interrupts during which the LED is turned on.
//!
//! \return None.
//
//*****************************************************************************
static void
UILEDBlink(unsigned long ulIdx, unsigned short usRate, unsigned short usPeriod)
{
    //
    // Clear the blink rate for this LED.
    //
    g_pusBlinkRate[ulIdx] = 0;

    //
    // A blink period of zero means that the LED should be turned off.
    //
    if(usPeriod == 0)
    {
        //
        // Turn off the LED.
        //
        GPIOPinWrite(g_pulLEDBase[ulIdx], g_pucLEDPin[ulIdx],
                     (ulIdx == 0) ? g_pucLEDPin[0] : 0);
    }

    //
    // A blink rate equal to the blink period means that the LED should be
    // turned on.
    //
    else if(usRate == usPeriod)
    {
        //
        // Turn on the LED.
        //
        GPIOPinWrite(g_pulLEDBase[ulIdx], g_pucLEDPin[ulIdx],
                     (ulIdx == 0) ? 0 : g_pucLEDPin[ulIdx]);
    }

    //
    // Otherwise, the LED should be blinked at the given rate.
    //
    else
    {
        //
        // Save the blink rate and period for this LED.
        //
        g_pusBlinkRate[ulIdx] = usRate;
        g_pusBlinkPeriod[ulIdx] = usPeriod;
    }
}

//*****************************************************************************
//
//! Sets the blink rate for the run LED.
//!
//! \param usRate is the rate to blink the run LED.
//! \param usPeriod is the amount of time to turn on the run LED.
//!
//! This function sets the rate at which the run LED should be blinked.  A
//! blink period of zero means that the LED should be turned off, and a blink
//! period equal to the blink rate means that the LED should be turned on.
//! Otherwise, the blink rate determines the number of user interface
//! interrupts during the blink cycle of the run LED, and the blink period
//! is the number of those user interface interrupts during which the LED is
//! turned on.
//!
//! \return None.
//
//*****************************************************************************
void
UIRunLEDBlink(unsigned short usRate, unsigned short usPeriod)
{
    //
    // The run LED is the first LED.
    //
    UILEDBlink(0, usRate, usPeriod);
}

//*****************************************************************************
//
//! Sets the blink rate for the fault LED.
//!
//! \param usRate is the rate to blink the fault LED.
//! \param usPeriod is the amount of time to turn on the fault LED.
//!
//! This function sets the rate at which the fault LED should be blinked.  A
//! blink period of zero means that the LED should be turned off, and a blink
//! period equal to the blink rate means that the LED should be turned on.
//! Otherwise, the blink rate determines the number of user interface
//! interrupts during the blink cycle of the fault LED, and the blink period
//! is the number of those user interface interrupts during which the LED is
//! turned on.
//!
//! \return None.
//
//*****************************************************************************
void
UIFaultLEDBlink(unsigned short usRate, unsigned short usPeriod)
{
    //
    // The fault LED is the second LED.
    //
    UILEDBlink(1, usRate, usPeriod);
}

//*****************************************************************************
//
//! This function returns the current number of system ticks.
//!
//! \return The number of system timer ticks.
//
//*****************************************************************************
unsigned long
UIGetTicks(void)
{
    unsigned long ulTime1;
    unsigned long ulTime2;
    unsigned long ulTicks;

    //
    // We read the SysTick value twice, sandwiching taking the snapshot of
    // the tick count value. If the second SysTick read gives us a higher
    // number than the first read, we know that it wrapped somewhere between
    // the two reads so our tick count value is suspect.  If this occurs,
    // we go round again. Note that it is not sufficient merely to read the
    // values with interrupts disabled since the SysTick counter keeps
    // counting regardless of whether or not the wrap interrupt has been
    // serviced.
    //
    do
    {
        ulTime1 = TimerValueGet(TIMER1_BASE, TIMER_A);
        ulTicks = g_ulUITickCount;
        ulTime2 = TimerValueGet(TIMER1_BASE, TIMER_A);
    }
    while(ulTime2 > ulTime1);

    //
    // Calculate the number of ticks
    //
    ulTime1 = ulTicks + (SYSTEM_CLOCK / TIMER1A_INT_RATE) - ulTime2;

    //
    // Return the value.
    //
    return(ulTime1);
}

//*****************************************************************************
//
//! Handles the Timer1A interrupt.
//!
//! This function is called when Timer1A asserts its interrupt.  It is
//! responsible for keeping track of system time.  This should be the highest
//! priority interrupt.
//!
//! \return None.
//
//*****************************************************************************
void
Timer1AIntHandler(void)
{
    //
    // Clear the Timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Increment the running count of timer ticks, based on the Timer1A Tick
    // interrupt rate.
    //
    g_ulUITickCount += (SYSTEM_CLOCK / TIMER1A_INT_RATE);
    
    

}


//*****************************************************************************
//
//! Handles the SysTick interrupt.
//!
//! This function is called when SysTick asserts its interrupt.  It is
//! responsible for handling the on-board user interface elements (push button
//! and potentiometer) if enabled, and the processor usage computation.
//!
//! \return None.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    unsigned long ulIdx, ulCount;
    //static int cnt;
    static int watchDogState = 0;
    static short adcCount;

    //
    // Run the Hall module tick handler.
    //
    HallTickHandler();

    //
    // Run the ADC module tick handler.
    //
    ADCTickHandler();

    //
    // Run the UI Ethernet tick handler.
    //
    UIEthernetTick(UI_TICK_US);

    //
    // Convert the ADC Analog Input reading to milli-volts.  Each volt at the
    // ADC input corresponds to ~20 volts at the Analog Input.
    //
    if((ulCount = ADCReadAnalog()) != 0xffffffff)
    {

    	g_usIrrigationVoltage = (short)(ulCount* 15 * 125 / 32);


    	//  calculate offset
    	if((adcCount < 300) )
    	{
    		if(adcCount >250)
    		{
    			if(adcCount == 251)
    				g_usIrrigationVoltageOffset =g_usIrrigationVoltage;
    			else
    			{
    				g_usIrrigationVoltageOffset += g_usIrrigationVoltage;
    				g_usIrrigationVoltageOffset /=2;
    			}
    		}
    		adcCount++;
    	}
    	else
    	{
    		g_usIrrigationVoltage -= g_usIrrigationVoltageOffset;
    	}
    }

    //
    // Read the config switch settings into the GPIO data variable.
    //
    g_ulGPIOData = ((GPIOPinRead(PIN_CFG0_PORT,
                    (PIN_CFG0_PIN | PIN_CFG1_PIN | PIN_CFG2_PIN)) >> 2) &
                    0x07);

    //
    // Increment the blink counter.
    //
    g_ulBlinkCount++;
    
    //
    // Loop through the two LEDs.
    //
    for(ulIdx = 0; ulIdx < 2; ulIdx++)
    {
        //
        // See if this LED is enabled for blinking.
        //
        if(g_pusBlinkRate[ulIdx] != 0)
        {
            //
            // Get the count in terms of the clock for this LED.
            //
            ulCount = g_ulBlinkCount % g_pusBlinkRate[ulIdx];

            //
            // The LED should be turned on when the count is zero.
            //
            if(ulCount == 0)
            {
                GPIOPinWrite(g_pulLEDBase[ulIdx], g_pucLEDPin[ulIdx],
                             (ulIdx == 0) ? 0 : g_pucLEDPin[ulIdx]);
            }

            //
            // The LED should be turned off when the count equals the period.
            //
            if(ulCount == g_pusBlinkPeriod[ulIdx])
            {
                GPIOPinWrite(g_pulLEDBase[ulIdx], g_pucLEDPin[ulIdx],
                             (ulIdx == 0) ? g_pucLEDPin[0] : 0);
            }
        }
    }

    
    
    // toggle the watch dog to keep main relay alive
    watchDogState ^= 1;
    if(watchDogState)
    {
    	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    }
    else
    {
    	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    }

    //
    // Send real-time data, if appropriate.
    //
    if(g_ucDataComplete)
    {
        UIEthernetSendRealTimeData();
    }

    // incremment the operation if motor is running
    if(g_ulMeasuredSpeed != 0)
    	g_ulHPOpTicks++;
}

//
//
//
//

int UIGetHandPieceData()
{
	int i,rLen;
	unsigned long tStart = UIGetTicks();

	//wait for reply
	do
	{
		//time out in ~ 0.1 seconds
		if(UIGetTicks() - tStart > 5000000)
		{
			g_ucHPInitDone = 0x00;
			MainSetFault(FAULT_HP_COMM);
			g_ucDataComplete = 1;
			return(-1);
		}
		rLen = ui_uart_receive(rxData,NULL);

		// give a little delay ~1 ms
		SysCtlDelay(20000);

	}while(rLen ==-1);

    for (i =0; i< 5; i++)
    {
    	g_ulRxDataInt[i] = *((unsigned short*)rxData+i);

    	//check for open or short reading, the maximum value of hall reading is 0x200
    	if((g_ulRxDataInt[i] == 0))
    	{
    		g_ucDataComplete = 1;
    		MainSetFault(FAULT_HP_A2D);
    		return -1;
    	}
    }
    
    //swap 1.5 and 2.5 volts for backward compatibility
    g_ulRxDataInt[6] = *((unsigned short*)rxData+5);
    g_ulRxDataInt[5] = *((unsigned short*)rxData+6);

	// added check polarity for the initial reading
	if(!initReadingDone)
	{
		//initialize the polarity to default
		handHallSpdPole = 0;

		//reverse the polarity
		for(i=0; i<UI_NUM_HALLS; i++)
	    {
		    if(g_ulRxDataInt[i+1] > HALL_POLARITY_THRD)
		    {
		    	handHallSpdPole = 1;
		    	break;
		    }
	    }
	}

	// convert data
	g_ucDataComplete = 0;
	//check polarity
	if(	handHallSpdPole == 1)
	{
		for( i = 0; i < UI_NUM_HALLS; i++)
		{

			if(g_ulRxDataInt[i+1] > 512 ) g_ulRxDataInt[i+1] = 512;
			g_ulRxDataInt[i+1] = 512 - g_ulRxDataInt[i+1];

		}
	}

    g_ucDataComplete = 1;

    return 1;
}


void initHandPiece(void)
{
	int i;
	int cnt = 0;
	char EECmdBuf[UI_EE_CONST_SIZE];

	//first, check if this function is already called
	if(g_ucHPInitStart) return;


	//check if user reset command is received, if so start rest sequence

	if(g_ucHPReset != 0)
	{
		if(g_ucHPReset++ == 1)
		{
			ExpandedIOUpdate(EXPANDEDIO_PORTB,EXPANDEDIO_HOLD_HANDPIECE);
			return;
		}

		if(g_ucHPReset == HP_RESET_CNT)
		{
			ExpandedIOUpdate(EXPANDEDIO_PORTB,EXPANDEDIO_RELEASE_HANDPIECE);
			ExpandedIOUpdate(EXPANDEDIO_PORTA,EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_DISABLE | EXPANDEDIO_CUTTER_DISABLED);
			g_ucHPReset = 0;
			initReadingDone = 0x00;
			g_ucHPInitDone = 0x00;
		}
		else
		{
			return;
		}
	}

	//fisrt set start flag
	g_ucHPInitStart = 0x01;

	//
	// now start reading handpiece information,
	// handpiece start in host command mode,
	//

	//read serial number, this is a fixed number once it is set during production
	EECmdBuf[0]=0xFF;
	EECmdBuf[1]=0x05;
	EECmdBuf[2]=0x81;
	EECmdBuf[3]=0x00;
	// loop here until there is a connection
	while (ui_uart_ucmd(EECmdBuf, 4)==-1)
	{
	    if(cnt ++ > 5)
	        MainSetFault(FAULT_HP_COMM);
	}

	memcpy(g_usEESerialNumber, EECmdBuf, UI_EE_DEFAULT_SIZE);

	//read ee origin
	EECmdBuf[0]=0xFF;
	EECmdBuf[1]=0x05;
	EECmdBuf[2]=0x81;
	EECmdBuf[3]=0x01;
	if(ui_uart_ucmd(EECmdBuf, 4)==-1)
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	memcpy(g_usEEOrigin, EECmdBuf, UI_EE_CONST_SIZE);

	//read ee axis
	EECmdBuf[0]=0xFF;
	EECmdBuf[1]=0x05;
	EECmdBuf[2]=0x81;
	EECmdBuf[3]=0x02;
	if(ui_uart_ucmd(EECmdBuf, 4)==-1)
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	memcpy(g_usEEAxis, EECmdBuf, UI_EE_CONST_SIZE);

	//read ee normal
	EECmdBuf[0]=0xFF;
	EECmdBuf[1]=0x05;
	EECmdBuf[2]=0x81;
	EECmdBuf[3]=0x15;
	if(ui_uart_ucmd(EECmdBuf, 4)==-1)
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	memcpy(g_usEENormal, EECmdBuf, UI_EE_CONST_SIZE);

	//read operating time, this is used as an initial value to calculate
	// the operating time, it is written to handpiece every time the burr is stopping
	g_usHPOpTimeStr[0]=0xFF;
	g_usHPOpTimeStr[1]=0x05;
	g_usHPOpTimeStr[2]=0x81;
	g_usHPOpTimeStr[3]=0x03;
	if(ui_uart_ucmd(g_usHPOpTimeStr, 4)==-1)
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}
	g_ulHPOpTime = *(unsigned long *)g_usHPOpTimeStr;

	//read error code, it is a four byte code,
	//each byte represent a indivisual error,
	//so there are four error code history can be
	//saved
	g_usHPError[0]=0xFF;
	g_usHPError[1]=0x05;
	g_usHPError[2]=0x81;
	g_usHPError[3]=0x04;
	if(ui_uart_ucmd(g_usHPError, 4)==-1)
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	//read handpiece firmware version
	g_usFirmwareVersionH[0]=0xFF;
	g_usFirmwareVersionH[1]=0x05;
	g_usFirmwareVersionH[2]=0x81;
	g_usFirmwareVersionH[3]=0x16;
	if(ui_uart_ucmd(g_usFirmwareVersionH, 4) ==-1)
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	//Now we finished all intial reading, set handpiece in streaming mode
	tStr[0]=0xFF;
	tStr[1]=0x05;
	tStr[2]=0x00;
	tStr[3]=0x00;
	if(ui_uart_ucmd(tStr, 4)==-1)
	{
		MainSetFault(FAULT_HP_COMM);
		return;
	}

	//intiailize some intital min and max value
	for (i =0; i< UI_NUM_HALLS; i++)
	{
		g_ucHallMin[i] = 999;
		g_ucHallMax[i] = 0;
	}

	//clear the communication error
	if(g_ulFaultFlags == FAULT_HP_COMM)
	{
       MainClearFaults();
	}
	//set initialization done flag
	g_ucHPInitDone = 0x01;
	g_ucHPInitStart = 0x00;

	//reset initial hall reading done flag
	initReadingDone = 0;
}

//*****************************************************************************
//
//! Initializes the user interface.
//!
//! This function initializes the user interface modules (on-board and serial),
//! preparing them to operate and control the motor drive.
//!
//! \return None.
//
//*****************************************************************************
void
UIInit(void)
{
    //set up burr enabled, relay watchdog pin as output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_5);

    //setup enable cutter, motor over current as input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6);

    //
    //
    // Make the push button pin be a GPIO input.
    //
    GPIOPinTypeGPIOInput(PIN_SWITCH_PORT, PIN_SWITCH_PIN);
    GPIOPadConfigSet(PIN_SWITCH_PORT, PIN_SWITCH_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOInput(PIN_SWITCH_PORT, PIN_CUTTER_FAULT);
    GPIOPinWrite(PIN_LEDRUN_PORT, PIN_LEDRUN_PIN, 0);

    //
    // Make the LEDs be GPIO outputs and turn them off.
    //
    GPIOPinTypeGPIOOutput(PIN_LEDRUN_PORT, PIN_LEDRUN_PIN);
    GPIOPinTypeGPIOOutput(PIN_LEDFAULT_PORT, PIN_LEDFAULT_PIN);
    GPIOPinWrite(PIN_LEDRUN_PORT, PIN_LEDRUN_PIN, 0);
    GPIOPinWrite(PIN_LEDFAULT_PORT, PIN_LEDFAULT_PIN, 0);

    //set trigger status bit to high
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);

    g_ucBoardID = 0x05;


    //
    // Initialize the Ethernet user interface.
    //
    UIEthernetInit(GPIOPinRead(PIN_SWITCH_PORT, PIN_SWITCH_PIN) ?
                   true : false);

    //
    // Initialize the on-board user interface.
    //
    UIOnboardInit(GPIOPinRead(PIN_SWITCH_PORT, PIN_SWITCH_PIN), 0);

    //
    // Initialize the processor usage routine.
    //
    CPUUsageInit(SYSTEM_CLOCK, UI_INT_RATE, 2);


    //irrigation init
    IrrInit();


    //
    // Configure SysTick to provide a periodic user interface interrupt.
    //
    SysTickPeriodSet(SYSTEM_CLOCK / UI_INT_RATE);
    SysTickIntEnable();
    SysTickEnable();

    //
    // Configure and enable a timer to provide a periodic interrupt.
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SYSTEM_CLOCK / TIMER1A_INT_RATE));
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER1A);
    TimerEnable(TIMER1_BASE, TIMER_A);


    //
    // Load the parameter block from flash if there is a valid one.
    //
    UIParamLoad();
    
    //
    // Initialization for uart
    //
    ui_uart_init();

    //
    // Initialization for Handpiece
    //
    initHandPiece();
}

//*****************************************************************************
//
//! Get speed throttle position from the hand piece, it is in a discrete step format.
//!
//! This function calculate speed from speed hall sensors, it is used when there is no fault
//! on the hall sensor.
//!
//! \return None.
//
//*****************************************************************************

int getThrottleSpeed(unsigned long *initHallReading)
{
	//unsigned long hallReadingSum;
	int tSpeedThrottle =0;
	int tHallMin =9999;
	int tIndex = 0;
	unsigned int tHall;
	int hallSpacing = 0;
	static int hallMissCnt = 0;
	int i;
	int lInt,lZeroIndex,lFullSpeedIndex;
	
	hallReadingSum = 0;

	//check halllsensors for error
	for( i =0; i<UI_NUM_HALLS; i++)
	{
		tHall = 0;

		// if the hall reading is too large or too small return error
		if(g_ulRxDataInt[i+1] > LIMIT_HALL_SPEED_HIGH)
		{
			g_ucTriggerHallStatus |= (0x01 << i);
			MainSetFault(WARN_HALL_SPEED_HIGH(i));
		}
		if(g_ulRxDataInt[i+1] < LIMIT_HALL_SPEED_LOW)
		{
			g_ucTriggerHallStatus |= (0x01 << i);
			MainSetFault(WARN_HALL_SPEED_LOW(i));
		}
	}

	//check number of available hall sensors
	if(g_ucTriggerHallStatus & 0x0F)
	{
		lInt = 0;
		lZeroIndex = lFullSpeedIndex =-1;
		//check if there are at least two hall sensors are good
		for( i =0; i<UI_NUM_HALLS; i++)
		{
			if(g_ucTriggerHallStatus & (0x01 << i))
			{
				lInt +=1;
			}
			else
			{
                if(lZeroIndex ==-1)
                {
                    lZeroIndex = i;
                }
                else if(lZeroIndex > i)
                {
                    lZeroIndex = i;
                }

                if(lFullSpeedIndex ==-1)
                {
                    lFullSpeedIndex = i;
                }
                else  if(lFullSpeedIndex < i)
                {
                    lFullSpeedIndex = i;
                }
			}
		}

		if(lInt > 2)
		{
			// there are at least three hall sensors are bad, stop drive the motor
			//Emergency stop the motor drive.
			MainEmergencyStop();
			//this is a non recoverable error
			MainSetFault(FAULT_HP_HALL);

		}
		else
		{
			//this will call the backup speed calculator
			//find the first two good hall sensors start from the first hall
			//in case the first two hall sensors are good, speed needs to handle
			//differently

			//detect full speed
			if((lZeroIndex ==0) && (lFullSpeedIndex == 1))
			{
                //check zero speed
                if((g_ulRxDataInt[lFullSpeedIndex+1] - g_ulRxDataInt[lZeroIndex+1]) > 30)
                {
                    tSpeedThrottle = 0;
                }

                //check full speed
				if(g_ulRxDataInt[lZeroIndex+1] > 200 )
				{
					tSpeedThrottle = UI_NUM_SPEED;
				}
			}
			else
			{
			    //check zero speed
	            if(g_ulRxDataInt[lFullSpeedIndex+1] - g_ulRxDataInt[lZeroIndex+1] > 30)
	            {
	                tSpeedThrottle = 0;
	            }

	            //check full speed
	            if(g_ulRxDataInt[lZeroIndex+1] - g_ulRxDataInt[lFullSpeedIndex+1] > 30)
				{
					tSpeedThrottle = UI_NUM_SPEED;
				}
			}
		}

		// reset hall fault bits since the warning is already generated
		g_ucTriggerHallStatus = 0x00;
	}
	else
	{
		for( i =0; i<UI_NUM_HALLS; i++)
		{
			tHall = 0;

			//find the hall number which has the smallest reading
			if(g_ulRxDataInt[i+1] < tHallMin )
			{
				tHallMin = g_ulRxDataInt[i+1];
				tIndex = i;
			}

			//update hall min/max if neccessary
			if(g_ulRxDataInt[i+1] < g_ucHallMin[i])
			{
				g_ucHallMin[i] = g_ulRxDataInt[i+1];
			}
			if(g_ulRxDataInt[i+1] > g_ucHallMax[i])
			{
				g_ucHallMax[i] = g_ulRxDataInt[i+1];
			}

			// calculate the sum of halls
			if(g_ulRxDataInt[i+1] > g_ucHallMin[i])
			{
				tHall = g_ulRxDataInt[i+1] - g_ucHallMin[i];
			}
			hallReadingSum |=  tHall << ((3-i)*7);

		}

		//update minimum hall reading index
		g_ucInitHallReading[5] = tIndex;

		//calculate speed
		hallReadingSum = hallReadingSum - initHallReading[2];
		tSpeedThrottle = hallReadingSum / initHallReading[4];


		//check zero speed
		if( MainIsRunning())
		{
			if( ((g_ulRxDataInt[2] -  g_ulRxDataInt[1]) > 20 ) && (tIndex == 0) )
			{
				tSpeedThrottle = 0;
			}
		}
		else
		{
			if (tIndex ==0)
			{
				tSpeedThrottle = 0;
			}
		}

		//maintain maximum speed wheen the magnet is no top of last hall
		if(tIndex ==3)
		{
			tSpeedThrottle = UI_NUM_SPEED;
		}

		// make sure the speed is not exceeding the maximum
		if(tSpeedThrottle> UI_NUM_SPEED) tSpeedThrottle = UI_NUM_SPEED;


		g_triggerInfo=tSpeedThrottle;

		 		// for footpedal cutting type
		 		if(g_sParameters.usCutType==1)
		 			{

		 		tSpeedThrottle = UI_NUM_SPEED;

		 		}
		//check if hall signal is skipped
		hallSpacing = g_ucTIndexPrev - tIndex;

		if(hallSpacing >1 || hallSpacing < -1)
		{
			hallMissCnt++;
		}

		if(hallMissCnt >LIMIT_HALL_INDEX_MISSING )
		{
			MainSetFault(WARN_HALL_SPEED_SEQUENCE);
		}
		else
		{
			hallMissCnt = 0;
		}

		//update precious throttle index
		if(g_ucTIndexPrev != tIndex)
		{
			g_ucTIndexPrev = tIndex;
		}
	}
	
	return tSpeedThrottle;
}
//*****************************************************************************
//
//! Get intial speed hall sensor reading from the hand piece.
//!
//! This function calculate the speed mapping based on the intial reading of the speed
//! hall sensors.
//!
//! \return 1 on success, otherwise, return -1.
//
//*****************************************************************************

int getInitHallReading()
{
	int i;
	int tempMin= 9999, tempMax= 0;
	int tempIndex= 0;
	
	g_ucInitHallReading[2] = 0;
	
	// do not proceed all hall reading are zeros, just wait for next cycle
	if(g_ulRxDataInt[0] ==0) {return -1;}
	

    //check all the hall reading range
    for(i=0; i<UI_NUM_HALLS; i++)
    {
        if(g_ulRxDataInt[i+1] > LIMIT_HALL_SPEED_HIGH)
        {
            MainSetFault(WARN_HALL_SPEED_HIGH(i));
            g_ucTriggerHallStatus |= (0x01 << i);
        }
        if(g_ulRxDataInt[i+1] < LIMIT_HALL_SPEED_LOW)
        {
            MainSetFault(WARN_HALL_SPEED_LOW(i));
            g_ucTriggerHallStatus |= (0x01 << i);
        }
    }

	//get min/max
	for(i=0; i<UI_NUM_HALLS; i++)
	{
	    if(!(g_ucTriggerHallStatus && (0x01 << i)))
	    {
	        if(g_ulRxDataInt[i+1] < tempMin)
	        {
	            tempMin = g_ulRxDataInt[i+1];
	            tempIndex = i;
	        }

	        if(g_ulRxDataInt[i+1] > tempMax)
	        {
	            tempMax = g_ulRxDataInt[i+1];
	        }
	    }
	}
	
	if (g_sParameters.usCutType==0 )
	{
	//add a check to insure the trigger is fully released to start
    if(tempIndex != 0)
    {
    	MainSetFault(FAULT_HALL_INIT);
    	return -1;
    }
	}
	
	//check range		
	g_ucInitHallReading[0] = tempMin;
	g_ucInitHallReading[1] = tempMax;
	if((g_ucInitHallReading[1] - g_ucInitHallReading[0]) < LIMIT_HALL_SPEED_RANGE)
	{
		MainSetFault(WARN_HALL_SPEED_RANGE);
	}	

	//check for open hall sensor
	for(i=0; i< UI_NUM_HALLS; i++)
	{
		tempMin = abs(g_ulRxDataInt[i+1]- 255);
		tempMax = abs(tempIndex - i);
		if((tempMin < LIMIT_HALL_SPEED_NOISE) && (tempMax <2))
		{
			MainSetFault(WARN_HALL_SPEED_LOW(i));
			g_ucTriggerHallStatus |= (0x01 << i);
		}
	}
	
	//get total hall reading range by forming two 28 bit intergers, every hall takes 7 bits,
	//It is assume that the low/high readings of all the hall sensors are close to each other
	g_ucInitHallReading[2] = 0;
	g_ucInitHallReading[3] = 0;
	
	for(i=0; i<UI_NUM_HALLS; i++)
	{
		g_ucInitHallReading[2] |= (g_ulRxDataInt[i+1] - g_ucInitHallReading[0]) << ((3-i)*7); 	
		g_ucInitHallReading[3] |= (g_ulRxDataInt[i+1] - g_ucInitHallReading[0]) << (i*7);
	}
	
	
	// get hall spacing
	g_ucInitHallReading[4] = (g_ucInitHallReading[3] - g_ucInitHallReading[2])/ UI_NUM_SPEED;
	
	g_ucInitHallReading[5] = tempIndex;
	
	return(1);
}

//*****************************************************************************
//
//! Set commanded speed from hall sensor reading from the hand piece.
//!
//! This function calculate the actual speed command based reading of the speed
//! hall sensors.
//!
//! \return None.
//
//*****************************************************************************

void UICheckAndSetSpeed(void)
{
	static int phaseShortCnt = 0;
	static int g_ucMotorStarted = 0;
	unsigned long ulInt;
	long l_cutterEnableOverride= 0;
	int l_irrCurrent;
	static unsigned int irrOverCurrentCnt = 0;

    //read setting from handpiece
    if(UIGetHandPieceData()==-1)return;
    
    //get the intial reading
    if(!initReadingDone )
    {
        if(getInitHallReading() == -1) return;
    	initReadingDone =1;
    	return;
    }
    
    // calculate embient temperature
    // T =R*(125/256) - 50,
    ulInt = (unsigned long)g_ulRxDataInt[0] * 125;
    g_sAmbientTemp =(short) (( ulInt - 12800) >> 8);
    
    // get speed
    g_ucSpeedThrottle = getThrottleSpeed(g_ucInitHallReading);
    //read enable and override inputs for later use
    l_cutterEnableOverride = GPIOPinRead(GPIO_PORTB_BASE,CUTTER_ENABLE_BIT | CUTTER_OVERRIDE_BIT);

    //check and reset override status bit
	if ((l_cutterEnableOverride & CUTTER_OVERRIDE_BIT) && cutterOverrideStatus)
	{
		cutterOverrideStatus = 0; //clear status bit once override is cleared
		g_ucSpeedThrottle = getThrottleSpeed(g_ucInitHallReading);
		// set speed
			if(g_ucSpeedThrottle ==0)
			{
				 g_sParameters.ulTargetSpeed = 0;
			}
			else
			{
				 g_sParameters.ulTargetSpeed = UI_BASE_SPEED + (g_ucSpeedThrottle-1) *
											   (UI_MAX_SPEED- UI_BASE_SPEED)/(UI_NUM_SPEED-1);
				 //set to minimum speed if the commanded speed is too low
				 if(g_sParameters.ulTargetSpeed < g_sParameters.ulMinSpeed)
				 {
					 g_sParameters.ulTargetSpeed = g_sParameters.ulMinSpeed;
				 }
				 //also clip on the maximum speed as well
				 if(g_sParameters.ulTargetSpeed > g_sParameters.ulMaxSpeed)
				 {
					 g_sParameters.ulTargetSpeed = g_sParameters.ulMaxSpeed;
				 }
			}

			//change the integral when the command speed is above the switch speed
			if((g_sParameters.ulTargetSpeed  > UI_GAIN_SWITCH_SPEED) &&
					(g_ucIntegralGainChanged == 0x00))
			{
				g_lFAdjI = g_sParameters.lPAdjI;
				g_ucIntegralGainChanged = 0x01;
			}

			//check handpiece trigger board for voltage errors
			if(g_ulRxDataInt[5] > LIMIT_HP_VOLTAGE1_COUNT + LIMIT_HP_VOLTAGE_NOISE ||
					g_ulRxDataInt[5] < LIMIT_HP_VOLTAGE1_COUNT - LIMIT_HP_VOLTAGE_NOISE)
			{
				MainSetFault(WARN_HP_VOLTAGE_RANGE);
			}

			if(g_ulRxDataInt[6] > LIMIT_HP_VOLTAGE2_COUNT + LIMIT_HP_VOLTAGE_NOISE ||
					g_ulRxDataInt[6] < LIMIT_HP_VOLTAGE2_COUNT - LIMIT_HP_VOLTAGE_NOISE)
			{
				MainSetFault(WARN_HP_VOLTAGE_RANGE);
			}





	}

    //check burr enable or override, these signals are active low
    if((!(l_cutterEnableOverride & CUTTER_ENABLE_BIT)) || (!(l_cutterEnableOverride & CUTTER_OVERRIDE_BIT)))
    {
    	//check for overide, if true, force a re-initialization of handpiece, this is to prevent handpiece from
    	//automatically running when trigger is stuck.
    	if(!(l_cutterEnableOverride & CUTTER_OVERRIDE_BIT) && !cutterOverrideStatus)
    	{

			if(getInitHallReading() == -1)
			{
				ExpandedIOUpdate(EXPANDEDIO_PORTA, EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_DISABLE | EXPANDEDIO_CUTTER_DISABLED);
				return; //simply return and wait for user to correct the fault
			}
			else
			{
	    		cutterOverrideStatus = 1; //set status bit when override is active
			}
    	}

    	if (cutterOverrideStatus==1){
    		g_ucSpeedThrottle=g_triggerInfo;
    		// set speed
    		    if(g_ucSpeedThrottle ==0)
    		    {
    		         g_sParameters.ulTargetSpeed = 0;
    		    }
    		    else
    		    {
    		    	 g_sParameters.ulTargetSpeed = UI_BASE_SPEED + (g_ucSpeedThrottle-1) *
    		    	                               (UI_MAX_SPEED- UI_BASE_SPEED)/(UI_NUM_SPEED-1);
    		    	 //set to minimum speed if the commanded speed is too low
    		    	 if(g_sParameters.ulTargetSpeed < g_sParameters.ulMinSpeed)
    		    	 {
    		    		 g_sParameters.ulTargetSpeed = g_sParameters.ulMinSpeed;
    		    	 }
    		    	 //also clip on the maximum speed as well
    		    	 if(g_sParameters.ulTargetSpeed > g_sParameters.ulMaxSpeed)
    		    	 {
    		    		 g_sParameters.ulTargetSpeed = g_sParameters.ulMaxSpeed;
    		    	 }
    		    }

    		    //change the integral when the command speed is above the switch speed
    		    if((g_sParameters.ulTargetSpeed  > UI_GAIN_SWITCH_SPEED) &&
    		    		(g_ucIntegralGainChanged == 0x00))
    		    {
    		    	g_lFAdjI = g_sParameters.lPAdjI;
    		    	g_ucIntegralGainChanged = 0x01;
    		    }

    		    //check handpiece trigger board for voltage errors
    			if(g_ulRxDataInt[5] > LIMIT_HP_VOLTAGE1_COUNT + LIMIT_HP_VOLTAGE_NOISE ||
    					g_ulRxDataInt[5] < LIMIT_HP_VOLTAGE1_COUNT - LIMIT_HP_VOLTAGE_NOISE)
    		    {
    		    	MainSetFault(WARN_HP_VOLTAGE_RANGE);
    		    }

    			if(g_ulRxDataInt[6] > LIMIT_HP_VOLTAGE2_COUNT + LIMIT_HP_VOLTAGE_NOISE ||
    					g_ulRxDataInt[6] < LIMIT_HP_VOLTAGE2_COUNT - LIMIT_HP_VOLTAGE_NOISE)
    		    {
    		    	MainSetFault(WARN_HP_VOLTAGE_RANGE);
    		    }


    	}

    	//if cutter status is disable previously, change the status
    	if(!(l_cutterEnableOverride & CUTTER_ENABLE_BIT) && (cutterEnableStatus == 0))
    	{
    		ExpandedIOUpdate(EXPANDEDIO_PORTA, EXPANDEDIO_CUTTER_ENABLED);
    		cutterEnableStatus = 1;
    	}

    	//check for phase short
    	if( (!g_ucMotorStarted)  && (g_ucSpeedThrottle > 0))
    	{
    		if(ADCCheckShort())
    		{
    			if(phaseShortCnt++ > LIMIT_PHASE_SHORT_CNT)
    			{
    			    //do not report error is cutter is disabled
    			    if( GPIOPinRead(GPIO_PORTB_BASE, CUTTER_ENABLE_BIT) )
    			        MainSetFault(FAULT_MOTOR_SHORT);
    				phaseShortCnt = 0;
    			}
    			return;
    		}
    		else
    		{
    			phaseShortCnt = 0;
    		}
    	}

    	//check for current offset fault
    	if(g_ulFaultFlags & FAULT_CURRENT_OFFSET )
    	{
    	    return;
    	}

    	//set motor start flag only if the commanded speed is greater than zero.
    	if(g_ucSpeedThrottle > 0)
    		g_ucMotorStarted = 1;


    	// check and run the motor if trigger is pressed
    	if(((g_ucSpeedThrottle > 0 && MainIsRunning() == 0)))
    	{
    		//reset the integral gain and flag
        	g_lFAdjI = g_sParameters.lFAdjI;
        	g_ucIntegralGainChanged = 0x00;
        	g_lSpeedIntegratorOffset = 0;
        	g_ucIntegralOffsetUpdated = 0x00;

    		//clear fault first
    		MainClearFaults();

    		//update state
    		g_ucState = 1;

    		//set direction
    		UIDirectionSet();

    		// run the motor
    		MainRun();

    		// turn on/off irrigation based on user input
    		if(g_sParameters.usIrrigationLevel > 0)
    		{
    			// enable the relay the irrigation
    			if(!(l_cutterEnableOverride & CUTTER_ENABLE_BIT))
    			{
    				expandioAStatus = EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_ENABLE | EXPANDEDIO_CUTTER_ENABLED;
    			}
    			else
    			{
    				expandioAStatus = EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_ENABLE | EXPANDEDIO_CUTTER_DISABLED;
    			}
    			ExpandedIOUpdate(EXPANDEDIO_PORTA, expandioAStatus);
    		}
    		else
    		{
    			// enable the relay and disable the irrigation
    			if(!(l_cutterEnableOverride & CUTTER_ENABLE_BIT))
    			{
    				expandioAStatus = EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_ENABLE | EXPANDEDIO_CUTTER_ENABLED;
    			}
    			else
    			{
    				expandioAStatus = EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_DISABLE | EXPANDEDIO_CUTTER_DISABLED;
    			}
    			ExpandedIOUpdate(EXPANDEDIO_PORTA, expandioAStatus);
    		}
    	}
    }

    //check if stop condition is met
    if(((l_cutterEnableOverride & CUTTER_ENABLE_BIT) && (l_cutterEnableOverride & CUTTER_OVERRIDE_BIT)) || (g_ucSpeedThrottle == 0))
    {
    	if(MainIsRunning())
    	{
    		g_ucState = 0x00;
    		MainEmergencyStop();
    		g_ucMotorStarted = 0;
    		g_ucUpdateOpTime = 0x01;
    	}

    }

    // enable the relay and disable the irrigation
    if ((l_cutterEnableOverride & CUTTER_ENABLE_BIT) && (cutterEnableStatus == 1))
    {
    	expandioAStatus = EXPANDEDIO_CUTTER_DISABLED | EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_DISABLE;
    	ExpandedIOUpdate(EXPANDEDIO_PORTA, expandioAStatus);
    	cutterEnableStatus = 0;
    }

    //check if hardware motor over current happens
    if( GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_6))
        MainSetFault(FAULT_CURRENT_HIGH_HW);

    //update the trigger information status
    if(g_triggerInfo == UI_NUM_SPEED)
    {
    	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
    }
    else if(g_triggerInfo == 0)
    {
    	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);
    }

    //check irrigation current to make sure no short
    if((MainIsRunning()) && (g_sParameters.usIrrigationLevel > 0))
    {
    	l_irrCurrent = IrrReadCurrent();


    	if(l_irrCurrent > IRRIGATION_CURRENT_LIMIT)
    	{
    		irrOverCurrentCnt++;
    	}
    	else
    	{
    		irrOverCurrentCnt = 0;
    	}

    	if(irrOverCurrentCnt > IRRIGATION_CURRENT_LIMIT_COUNT)
    	{
    		MainSetFault(FAULT_IRRIGATION_SHORT);
    	}
    }


    //check irrigation level on/off and take action accordingly
    if(expandioAStatus != 0xFFFF)
    {
    	if(g_sParameters.usIrrigationLevel > 0 )
    	{
    		if(MainIsRunning())
    		{
    			if(!(expandioAStatus & EXPANDEDIO_IRRIGATION_ENABLE_BIT))
    			{
    				// enable the relay the irrigation
    				expandioAStatus ^= EXPANDEDIO_IRRIGATION_ENABLE_BIT;
    				ExpandedIOUpdate(EXPANDEDIO_PORTA, expandioAStatus);
    			}
    		}
    		else if((expandioAStatus & EXPANDEDIO_IRRIGATION_ENABLE_BIT))
    		{
    			expandioAStatus ^= EXPANDEDIO_IRRIGATION_ENABLE_BIT;
    			ExpandedIOUpdate(EXPANDEDIO_PORTA, expandioAStatus );
    		}
    	}
    	else
    	{
    		if(expandioAStatus & EXPANDEDIO_IRRIGATION_ENABLE_BIT)
    		{
    			// enable the relay and disable the irrigation
    			expandioAStatus ^= EXPANDEDIO_IRRIGATION_ENABLE_BIT;
    			ExpandedIOUpdate(EXPANDEDIO_PORTA, expandioAStatus);
    		}
    	}
    }
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
