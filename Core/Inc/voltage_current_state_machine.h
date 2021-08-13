/*
 * voltage_current_state_machine.h
 *
 *  Created on: 12 sie 2021
 *      Author: huber
 */

#ifndef INC_VOLTAGE_CURRENT_STATE_MACHINE_H_
#define INC_VOLTAGE_CURRENT_STATE_MACHINE_H_
#define analog_data uint16_t

struct VCtateMachineDataType{
    uint8_t state;


};

/**
 * @brief Temperature State Machine runtime decides which perypherials must be shutdown and which are free to be active.
 * More information about PollVector ANd how it work at MSM_Runtime in MinirysboardStateMachine.h
 *
 */

void VCSM_Runtime(struct VCtateMachineDataType *TSM,analog_data * analog_inputs, uint8_t *PollVector)
    {



    }


#endif /* INC_VOLTAGE_CURRENT_STATE_MACHINE_H_ */
