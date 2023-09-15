/*
 * PiCommsRX.c
 *
 *  Created on: Aug 17, 2023
 *      Author: Kaveet
 */


#include "Comms Inc/PiCommsRX.h"
#include "Lib Inc/state_machine.h"
#include "Lib Inc/threads.h"
#include "main.h"
#include "stm32u5xx_hal_uart.h"

//static function helpers.
static bool stopRecovery(bool * isActive);
static bool stopGPSCollection(bool * isActive);

//External variables
extern UART_HandleTypeDef huart2;
extern Thread_HandleTypeDef threads[NUM_THREADS];
extern TX_EVENT_FLAGS_GROUP state_machine_event_flags_group;

//Data buffer for UART received data
uint8_t dataBuffer[256] = {0};
uint8_t ack_byte = 0x26;

TX_EVENT_FLAGS_GROUP pi_comms_event_flags_group;


void comms_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if (dataBuffer[0] == PI_COMMS_START_CHAR){
		tx_event_flags_set(&pi_comms_event_flags_group, PI_COMMS_VALID_START_FLAG, TX_OR);
	}
	else {
		tx_event_flags_set(&pi_comms_event_flags_group, PI_COMMS_BAD_START_FLAG, TX_OR);
	}

}

void pi_comms_rx_thread_entry(ULONG thread_input){

	tx_event_flags_create(&pi_comms_event_flags_group, "Pi Comms RX Event Flags");

	//HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, comms_UART_RxCpltCallback);
	HAL_GPIO_WritePin(APRS_PD_GPIO_Port, APRS_PD_Pin, GPIO_PIN_SET);
	while (1) {

		dataBuffer[1] = 0;

		//Tell tag current mode recovery board is on
		HAL_UART_Transmit(&huart2, &ack_byte, 1, HAL_MAX_DELAY);

		//Start a non-blocking 1 byte UART read. Let the RX complete callback handle the rest.
		HAL_UART_Receive_IT(&huart2, &dataBuffer[0], 2); //(uint8_t *)

		HAL_Delay(1000);

		//Tell tag current mode recovery board is on
		HAL_UART_Transmit(&huart2, &dataBuffer[1], 1, HAL_MAX_DELAY);

		//Allow other threads to run while waiting for command
		tx_thread_sleep(tx_s_to_ticks(5));

		if (dataBuffer[0] == PI_COMMS_START_CHAR){
			tx_event_flags_set(&pi_comms_event_flags_group, PI_COMMS_VALID_START_FLAG, TX_OR);
		}
		else {
			tx_event_flags_set(&pi_comms_event_flags_group, PI_COMMS_BAD_START_FLAG, TX_OR);
		}

		ULONG actual_flags;

		//Wait for the Interrupt callback to fire and set the flag (telling us what to do further)
		tx_event_flags_get(&pi_comms_event_flags_group, PI_COMMS_VALID_START_FLAG | PI_COMMS_BAD_START_FLAG, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

		if (actual_flags & PI_COMMS_VALID_START_FLAG){

			//parse the message and act accordingly
			pi_comms_parse_message(dataBuffer[1], &dataBuffer[3], dataBuffer[2]);
		}

	}

}

void pi_comms_parse_message(Message_IDs message_id, uint8_t * payload_pointer, uint8_t payload_length){

	static bool isRecovery = false;
	static bool isGPSCollect = false;

	switch (message_id) {

		case START_RECOVERY:

			//Publish event flag to state machine to enter recovery
			tx_event_flags_set(&state_machine_event_flags_group, STATE_COMMS_APRS_FLAG, TX_OR);
			break;

		case STOP:

			//Publish event flag to state machine to stop (and wait for more commands)
			tx_event_flags_set(&state_machine_event_flags_group, STATE_COMMS_STOP_FLAG, TX_OR);
			break;

		case START_GPS_COLLECTION:

			//Publish event flag to state machine to enter regular GPS collection
			tx_event_flags_set(&state_machine_event_flags_group, STATE_COMMS_COLLECT_GPS_FLAG, TX_OR);
			break;
		case ENTER_CRITICAL:

			//Publish event flag to state machine to enter a critical low power state (nothing runs)
			tx_event_flags_set(&state_machine_event_flags_group, STATE_CRITICAL_LOW_BATTERY_FLAG, TX_OR);
		default:
			//Bad message ID - do nothing
			break;

	}

}
