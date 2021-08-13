/*
 * comunication_state_machine.h
 *
 *  Created on: 12 sie 2021
 *      Author: huber
 */

#ifndef INC_COMUNICATION_STATE_MACHINE_H_
#define INC_COMUNICATION_STATE_MACHINE_H_

struct CommtateMachineDataType{
    uint8_t state;


};

/**
 * @brief Temperature State Machine runtime decides which perypherials must be shutdown and which are free to be active.
 * More information about PollVector ANd how it work at MSM_Runtime in MinirysboardStateMachine.h
 *
 */

void CommSM_Runtime(struct CommtateMachineDataType *TSM, uint8_t *PollVector)
    {



    }

#endif /* INC_COMUNICATION_STATE_MACHINE_H_ */
