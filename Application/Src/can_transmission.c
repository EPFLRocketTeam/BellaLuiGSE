/*
 * CAN_communication.c
 *
 * If you read a frame and there is a message, the led blinks blue.
 * If you write a frame and it fails, the led blinks red.
 * If you write a frame and if does not fail, the led blinks green.
 *
 *  Created on: Feb 23, 2019
 *      Author: Tim Lebailly
 */

#include <can_transmission.h>
#include <debug/board_io.h>
#include <debug/console.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal_can.h>
#include <stm32f4xx_hal_def.h>
#include <sys/_stdint.h>

#define CAN_BUFFER_DEPTH 256

extern CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
uint32_t              TxMailbox;

uint8_t board_id;

volatile CAN_msg can_current_msg;

CAN_msg can_buffer[CAN_BUFFER_DEPTH];
volatile int32_t can_buffer_pointer_rx = 0;
volatile int32_t can_buffer_pointer_tx = 0;

uint32_t can_readFrame(void);

uint32_t pointer_inc(uint32_t val, uint32_t size) {
	return (val + 1) % size;
}

void can_addMsg(CAN_msg msg) {
	can_buffer[can_buffer_pointer_tx] = msg;
	can_buffer_pointer_tx = pointer_inc(can_buffer_pointer_tx, CAN_BUFFER_DEPTH);

	if (can_buffer_pointer_tx == can_buffer_pointer_rx) { // indicates overflow
		can_buffer_pointer_rx = pointer_inc(can_buffer_pointer_rx, CAN_BUFFER_DEPTH); // skip one msg in the rx buffer
	}
}

/*
 * Configures CAN protocol for 250kbit/s without interrupt for reading (only polling).
 */
void CAN_Config(uint32_t id) {
	board_id = id;

    CAN_FilterTypeDef  sFilterConfig;
    
    /*##-1- Configure the CAN peripheral #######################################*/
    // Done in MX_CAN1_Init()

    /*##-2- Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        //      _Error_Handler(__FILE__, __LINE__);
    }
    
    /*##-3- Start the CAN peripheral ###########################################*/
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        //      _Error_Handler(__FILE__, __LINE__);
    }
    
    /*##-4- Activate CAN RX notification #######################################*/
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
    	//_Error_Handler(__FILE__, __LINE__);
    }
    
    /*##-5- Configure Transmission process #####################################*/
    TxHeader.StdId = id;
    TxHeader.ExtId = id; // not needed
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
}

/*
 * Sends a frame of 8 bytes (payload) on the CAN bus using our predefined protocol.
 * byte 0..3 --> some uint32_t
 * byte 4    --> data_id, see CAN_communication.h
 * byte 5..7 --> timestamp
 */
void can_setFrame(uint32_t data, uint8_t data_id, uint32_t timestamp) {
	uint8_t TxData[8] = {0};
	TxData[0] = (uint8_t) (data >> 24);
    TxData[1] = (uint8_t) (data >> 16);
    TxData[2] = (uint8_t) (data >> 8);
    TxData[3] = (uint8_t) (data >> 0);
    TxData[4] = data_id;
    TxData[5] = (uint8_t) (timestamp >> 16);
    TxData[6] = (uint8_t) (timestamp >> 8);
    TxData[7] = (uint8_t) (timestamp >> 0);

	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {} // wait for CAN to be ready

	CAN_msg message = (CAN_msg) {data, data_id, timestamp, TxHeader.StdId};

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
    	if(has_io_mode(IO_OUTPUT & IO_CAN & IO_AUTO) || message.id >= 200) {
			can_addMsg(message);
		}

    	if(has_io_mode(IO_OUTPUT & IO_CAN & IO_PIPE)) {
			uint8_t buffer[8];

			buffer[0] = message.id;
			buffer[1] = (uint8_t) (message.timestamp >> 16);
			buffer[2] = (uint8_t) (message.timestamp >> 8);
			buffer[3] = (uint8_t) (message.timestamp >> 0);
			buffer[4] = (uint8_t) (message.data >> 24);
			buffer[5] = (uint8_t) (message.data >> 16);
			buffer[6] = (uint8_t) (message.data >> 8);
			buffer[7] = (uint8_t) (message.data >> 0);

			rocket_transmit(buffer, 8);
		}

    	if(has_io_mode(IO_OUTPUT & IO_CAN & IO_DIRECT)) {
			rocket_log("%d: %08x @ %dms\n", (uint32_t) message.id, message.data, message.timestamp);
		}
    } else { // something bad happen
    	// not sure what to do
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	can_readFrame();

	if(has_io_mode(IO_INPUT & IO_CAN & IO_AUTO)) {
		can_addMsg(can_current_msg);
	}
}

uint32_t can_msgPending() {
	int32_t diff = can_buffer_pointer_tx - can_buffer_pointer_rx;
	if (diff < 0) {
		diff += CAN_BUFFER_DEPTH;
	}

	return diff;
}

CAN_msg can_readBuffer() {
	CAN_msg ret = {0};

	if (can_msgPending() > 0) {
		ret = can_buffer[can_buffer_pointer_rx];
		can_buffer_pointer_rx = pointer_inc(can_buffer_pointer_rx, CAN_BUFFER_DEPTH);
	} else { // no message actually pending
		// do nothing, will return the {0} CAN_msg
	}


	return ret;
}

/*
 * Reads the CAN bus and sets global CAN_msg current_msg struct (see CAN_communication.h)
 * Returns the fill level when the function was called
 * This way, the caller knows if the variable current_msg was updated by the funciton call or not.
 *
 * byte 0..3 --> some uint32_t
 * byte 4    --> data_id, see CAN_communication.h
 * byte 5..7 --> timestamp
 */
uint32_t can_readFrame(void) {
    uint32_t fill_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
    if (fill_level > 0) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

        can_current_msg.data = 0;
        can_current_msg.data += (uint32_t) RxData[0] << 24;
        can_current_msg.data += (uint32_t) RxData[1] << 16;
        can_current_msg.data += (uint32_t) RxData[2] << 8;
		can_current_msg.data += (uint32_t) RxData[3] << 0;

        can_current_msg.id = RxData[4];

        can_current_msg.timestamp = 0;
        can_current_msg.timestamp += (uint32_t) RxData[5] << 16;
        can_current_msg.timestamp += (uint32_t) RxData[6] << 8;
		can_current_msg.timestamp += (uint32_t) RxData[7] << 0;

        can_current_msg.id_CAN = RxHeader.StdId;
    }
    return fill_level;
}

uint8_t get_board_id() {
	return board_id;
}
