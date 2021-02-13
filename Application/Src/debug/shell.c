/*
 * shell.c
 *
 *  Created on: 5 Sep 2020
 *      Author: arion
 */

#include "debug/shell.h"
#include "debug/console.h"
#include "debug/board_io.h"

#include "can_transmission.h"
#include "sync.h"


static void (*__shell_terminal)(ShellCommand* cmd, int (*respond)(const char* format, ...));
static UART_HandleTypeDef* __shell_uart;

static uint8_t dma_buffer[CMD_BUFFER_SIZE];

static char command_buffer[CMD_BUFFER_SIZE];
static uint8_t can_buffer[8];
static uint8_t can_index = 0;

static uint8_t command_index;

static ShellCommand cmd;

static volatile int8_t bridge = -1; // No bridge by default

void shell_init(UART_HandleTypeDef* uart, void (*terminal)(ShellCommand* cmd, int (*respond)(const char* format, ...))) {
	__shell_uart = uart;
	__shell_terminal = terminal;

	cmd.components[0].component = command_buffer; // Bind command structure to buffer
}

void shell_bridge(int8_t board_id) {
	bridge = board_id;
}

int8_t get_shell_bridge() {
	return bridge;
}

void TK_shell(const void *args) {
	uint32_t lastDmaStreamIndex = 0;
	uint32_t endDmaStreamIndex = 0;

	HAL_UART_Receive_DMA(__shell_uart, dma_buffer, CMD_BUFFER_SIZE);

	while(true) {
		endDmaStreamIndex = CMD_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(__shell_uart->hdmarx);

		while(lastDmaStreamIndex != endDmaStreamIndex) {
			shell_receive_byte(dma_buffer[lastDmaStreamIndex], bridge);
			lastDmaStreamIndex = (lastDmaStreamIndex + 1) % CMD_BUFFER_SIZE;
		}
	}
}

static void __bridge_receive(uint8_t* buffer, uint8_t length) {
	buffer[length++] = '\n';

	while(length % 4 != 0) {
		buffer[length++] = '\0';
	}

	for(uint8_t i = 0; i < length / 4; i++) {
		can_setFrame(((uint32_t*) buffer)[i], DATA_ID_SHELL_INPUT, HAL_GetTick());
	}
}

void shell_receive_byte(char cbuf, int32_t bridge) {
	static CAN_msg message = { 0 };

	if(has_io_mode(IO_CAN & IO_INPUT & IO_PIPE)) {
		can_buffer[can_index++] = cbuf;

		if(can_index == 8) {

			message.id = can_buffer[0];
			message.timestamp = (can_buffer[1] << 16) | (can_buffer[2] << 8) | (can_buffer[3] << 0);
			message.data = (can_buffer[4] << 24) | (can_buffer[5] << 16) | (can_buffer[6] << 8) | (can_buffer[7] << 0);

			can_addMsg(message);
			can_index = 0;
		}

		return;
	}

	if(cbuf == '\0') {
		return;
	}

	if(cbuf != '\n' && command_index < CMD_BUFFER_SIZE) {
		command_buffer[command_index++] = cbuf;

		if(cbuf == ' ') {
			uint8_t start_index = (&cmd.components[cmd.num_components].component[0] - &command_buffer[0]);

			cmd.components[cmd.num_components].length = command_index - start_index - 1;
			cmd.num_components++;
			cmd.components[cmd.num_components].component = &command_buffer[command_index];
		}
	} else {
		uint8_t start_index = (&cmd.components[cmd.num_components].component[0] - &command_buffer[0]);
		cmd.components[cmd.num_components].length = command_index - start_index;

		if(command_index - start_index > 0) {
			cmd.num_components++;
		}

		if(bridge == -1) {
			__shell_terminal(&cmd, &rocket_boot_log);
		} else {
			__bridge_receive((uint8_t*) command_buffer, command_index);
		}

		command_index = 0;
		cmd.num_components = 0;
		cmd.components[0].length = 0;
		cmd.components[0].component = command_buffer;
	}
}

bool component_matches(struct CommandComponent* comp, const char* target) {
	uint8_t i;

	for(i = 0; i < comp->length; i++) {
		if(target[i] != comp->component[i] || target[i] == '\0') {
			return false;
		}
	}

	return target[i] == '\0';
}
