//*****************************************************************************
//
// faults.h - Definitions for the fault conditions that can occur in the
//            Brushless DC motor drive.
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

#ifndef __FAULTS_H__
#define __FAULTS_H__

//*****************************************************************************
//
//! \page faults_intro Introduction
//!
//! There are several fault conditions that can occur during the operation of
//! the motor drive.  Those fault conditions are enumerated here and provide
//! the definition of the fault status read-only parameter and real-time data
//! item.
//!
//! The faults are:
//!
//! - Emergency stop: This occurs as a result of a command request.  An
//! emergency stop is one where the motor is stopped immediately without
//! regard for trying to maintain normal control of it (this is, without the
//! normal deceleration ramp).  From the motor drive perspective, the motor is
//! left to its own devices to stop, meaning it will coast to a stop under the
//! influence of friction unless a mechanical braking mechanism is provided.
//!
//! - DC bus under-voltage: This occurs when the voltage level of the DC bus
//! drops too low.  Typically, this is the result of the loss of mains power.
//!
//! - DC bus over-voltage: This occurs when the voltage level of the DC bus
//! rises too high.  When the motor is being decelerated, it becomes a
//! generator, increasing the voltage level of the DC bus.  If the level of
//! regeneration is more than can be controlled, the DC bus will rise to a
//! dangerous level and could damage components on the board.
//!
//! - Motor under-current: This occurs when the current through the motor drops
//! too low.  Typically, this is the result of an open connection to the motor.
//!
//! - Motor over-current: This occurs when the current through the motor rises
//! too high.  When the motor is being accelerated, more current flows through
//! the windings than when running at a set speed.  If accelerated too quickly,
//! the current through the motor may rise above the current rating of the
//! motor or of the motor drive, possibly damaging either.
//!
//! - Ambient over-temperature: This occurs when the case temperature of the
//! microcontrollers rises too high.  The motor drive generates lots of heat;
//! if in an enclosure with inadequate ventilation, the heat could rise high
//! enough to exceed the operating range of the motor drive components and/or
//! cause physical damage to the board.  Note that the temperature measurement
//! that is of more interest is directly on the heat sink where the smart
//! power module is attached, though this would require an external
//! thermocouple in order to be measured.
//!
//! - Motor Stall:  This occurs when the motor is running, and the speed
//! is detected as zero for at least 1.5 seconds.  This would typically occur
//! due to some type of mechanical interference to the operation of the motor
//! shaft.
//!
//! The definitions for the fault conditions are contained in
//! <tt>faults.h</tt>.
//
//*****************************************************************************

//*****************************************************************************
//
//! \defgroup faults_api Definitions
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! The mask for fault and warning, the lower 16 bits for faults, and the higher
//  16 bits are for warning
//
//*****************************************************************************

#define FAULT_MASK              0x0000FFFF
#define WARN_MASK               0xFFFF0000

//*****************************************************************************
//
//! The fault flag that indicates that an emergency stop operation was
//! performed.
//
//*****************************************************************************
#define FAULT_EMERGENCY_STOP    0x00000001

//*****************************************************************************
//
//! The fault flag that indicates that the DC bus voltage dropped too low.
//
//*****************************************************************************
#define FAULT_VBUS_LOW          0x00000002

//*****************************************************************************
//
//! The fault flag that indicates that the DC bus voltage rose too high.
//
//*****************************************************************************
#define FAULT_VBUS_HIGH         0x00000004

//*****************************************************************************
//
//! The fault flag that indicates that the motor current dropped too low.
//
//*****************************************************************************
#define FAULT_CURRENT_LOW       0x00000008

//*****************************************************************************
//
//! The fault flag that indicates that the motor current rose too high.
//
//*****************************************************************************
#define FAULT_CURRENT_HIGH      0x0000010

//*****************************************************************************
//
//! The fault flag that indicates that the watchdog timer expired.
//
//*****************************************************************************
#define FAULT_WATCHDOG          0x0000020

//*****************************************************************************
//
//! The fault flag that indicates that the ambient temperature rose too high.
//
//*****************************************************************************
#define FAULT_TEMPERATURE_HIGH  0x0000040

//*****************************************************************************
//
//! The fault flag that indicates that the irrigation motor circuit has a short.
//*****************************************************************************
#define FAULT_IRRIGATION_SHORT  0x00000080


//*****************************************************************************
//
//! The fault flag that indicates hardware current fault is triggered.
//
//*****************************************************************************
#define FAULT_CURRENT_HIGH_HW   0x00000100

//*****************************************************************************
//
//! The fault flag that indicates system fault is triggered.
//
//*****************************************************************************
#define FAULT_SYSTEM_ERROR      0x00000200

//*****************************************************************************
//
//! The fault flag that indicates hand piece has a connection issue
//
//*****************************************************************************

#define FAULT_HP_COMM           0x00000400

//*****************************************************************************
//
//! The fault flag that indicates the number of good hall sensors for speed control
//  is less than 2.
//
//*****************************************************************************
#define FAULT_HP_HALL           0x00000800

//*****************************************************************************
//
//! The fault flag that indicates there is a short on the motor phases or the FETs
//  this fault is not clearable
//
//*****************************************************************************
#define FAULT_MOTOR_SHORT       0x00001000

//*****************************************************************************
//
//! The fault flag that indicates the trigger is not fully released
//
//*****************************************************************************
#define FAULT_HALL_INIT         0x00002000


//*****************************************************************************
//
//! The fault flag that indicates the offset current is out of spec
//
//*****************************************************************************
#define FAULT_CURRENT_OFFSET    0x00004000

//*****************************************************************************
//
//! The fault flag that indicates the handpiece A2D reading is fault
//
//*****************************************************************************
#define FAULT_HP_A2D            0x00008000


//The following warnings are for hand piece hall sensors
//*****************************************************************************
//
//! The fault flag that indicates the difference between forward and reverse hall
//  reading exceeds the limit.
//
//*****************************************************************************
#define WARN_HP_VOLTAGE_RANGE          0x00040000
//*****************************************************************************
//
//! The fault flag that indicates the range among four speed hall sensors exceeds the limit
//
//*****************************************************************************
#define WARN_HALL_SPEED_RANGE        0x00080000
//*****************************************************************************
//
//! The fault flag that indicates the sequence of the speed hall sensors are wrong
//
//*****************************************************************************
#define WARN_HALL_SPEED_SEQUENCE     0x00100000


//*****************************************************************************
//
//! The fault flag that indicates the absolute reading of the speed hall sensor
//  exceeds the upper limit
//
//*****************************************************************************
#define WARN_HALL_SPEED_HIGH(i)      ((1<<i) << 24)
//*****************************************************************************
//
//! The fault flag that indicates the absolute reading of the speed hall sensor
//  exceeds the lower limit
//
//*****************************************************************************
#define WARN_HALL_SPEED_LOW(i)       ((1<<i) << 28)


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#endif // __FAULTS_H__
