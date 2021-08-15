/*
 * msm_runtime.h
 *
 *  Created on: 13 sie 2021
 *      Author: huber
 */

#ifndef INC_MSM_RUNTIME_H_
#define INC_MSM_RUNTIME_H_
#include "minirysboard_state_machine_utils.h"
#include "temperature_state_machine.h"
#include "comunication_state_machine.h"
#include "voltage_current_state_machine.h"

struct TStateMachineDataType TSM;
struct CommStateMachineDataType CM;
struct VCtateMachineDataType VCM;

/**
 * @brief Main of this task is to manage state machines and wight their outputs
 *
 *									Diagram of main runtime in state 2 or 3.
 *  _______________	      ____________________	     ___________________________           _____________________________	   _______________________
 * |		   |	     |			  |	    |				|	  |			        |	  |			  |
 * | Check sensors | - - - > | Assert Sensor Data | - - - > | Run Finite State Machines | - - - > | Finalize voting and execute | - - - > | Assert Data from RPI4 |
 * |_______________|         |____________________|	    |___________________________|	  |_____________________________|         |_______________________|
 * 	  ^																	      |
 *	  |___________________________________________________________________________________________________________________________________________|
 *
 * When Board Manager is in state 0 only data assertion from RPI4 is active.
 * When Board Manager is in state 1 Capability vector is limited to TOF
 *
 * This function determines state of a robot to be executed by creating a poll. In this poll following state machines say which efectors are to be shutdown.
 * IE we start with everything "on" configuration. Than temperature state machine says that motors cant be on due to overtemperature.
 * Than voltage state machine says that 5V rail cant be on because batteries are discharged.
 * At the end only efectors to which no state machine has objection is left on/turned on. For exaple temperature sensors.
 *
 ***************  Board Configuration Description  **********
 *
 * 0 - board error - all of.
 * 1 - board on external 5V - uart and analog sensor data.
 * 2 - on leaah/battery - No Fan detected.
 * 3 - on leash/battery - Fan detected.
 *
 *
 ***************  PollVector description  ********************
 * Vector has 8 bits.  Each bit corresponds to one efector
 *
 * __7___6___5___4___3___2___1___0__
 * | 0 | 0 | 0 | 1 | 1 | 1 | 1 | 1 | - initial vector - some outputs are diabled due to machine state (i e no fan detected
 *
 * __7___6___5___4___3___2___1___0__
 * | 0 | 1 | 1 | 0 | 1 | 1 | 1 | 1 | - vector from comms state machine - host dosn't want enabled Vmot
 *
 * __7___6___5___4___3___2___1___0__
 * | 0 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | - vector from temp state machine - temps are ok so no need for action
 *
 * __7___6___5___4___3___2___1___0__
 * | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 1 | - vector from voltage state machine - it detects low VBAT so it shutsdown everything but internal temp sensors and rpi
 *
 * __7___6___5___4___3___2___1___0__
 * | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 1 | - final vector - and operation is performed on vectors above. in this case we can only
 *
 * 7 - unused
 * 6 - FAN Enable
 * 5 - FAN Emergency mode enable
 * 4 - VMot Enable
 * 3 - TOF Enable
 * 2 - 5V Enable
 * 1 - BATT Enable
 * 0 - External sensors
 *
 *
 *
 */


void MSM_Runtime(struct MSM_StateDataType  * Robot_State)
    {
    static uint8_t PollVector;

    MSM_CheckAnalogSensors(Robot_State);
    switch(Robot_State->BoardConfiguration)
	{
    case 0:
	break;
    case 1:
	PollVector=0x89;// todo set execution vector to max allowed config
	CommSM_Runtime(&CM, ModbusDATA, &PollVector);//run comms state machine
	// todo run voltage/current state machine
	break;
    case 2:
	// todo set execution vector to max allowed config
	// todo run comms state machine
	// todo run voltage/current state machine
	// todo run temperature state machine
	break;
    case 3:
	// todo set execution vector to max allowed config
	// todo run comms state machine
	// todo run voltage/current state machine
	// TSM_Runtime(&TSM,(Robot_State->AnalogInputs.ADCInput), &PollVector);// todo run temperature state machine
	// todo run fan controller
	break;
	}
    // todo execute state
    // todo assert error table
    // todo assert data from RPI4
    }








#endif /* INC_MSM_RUNTIME_H_ */
