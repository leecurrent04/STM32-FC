/*
 * AeroAngleSensor.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Hyeonje
 */

#include <main.h>

#include <FC_Serial/MiniLink/MiniLink.h>


typedef struct __attribute__((packed)){
	uint8_t length;
	uint16_t angle_raw;
	float angle;
	float temp;
} TX_PACKET;

volatile TX_PACKET data;
#define I2C_SLAVE_ADDRESS (0x27 << 1)

void AAS_getValue()
{

	uint8_t i2c_rx_buffer[sizeof(TX_PACKET)] = {0};
	uint8_t register_to_read = 0x10;

	// --- 1. 레지스터 주소 쓰기 (기존과 동일) ---
	while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {}
	LL_I2C_GenerateStartCondition(I2C1);
	while(!LL_I2C_IsActiveFlag_SB(I2C1)) {}
	LL_I2C_TransmitData8(I2C1, I2C_SLAVE_ADDRESS & ~0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1)) {}
	LL_I2C_ClearFlag_ADDR(I2C1);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1)) {}
	LL_I2C_TransmitData8(I2C1, register_to_read);
	while(!LL_I2C_IsActiveFlag_BTF(I2C1)) {}

	// --- 2. 4바이트 데이터 읽기 (기존과 동일) ---
	LL_I2C_GenerateStartCondition(I2C1);
	while(!LL_I2C_IsActiveFlag_SB(I2C1)) {}
	LL_I2C_TransmitData8(I2C1, I2C_SLAVE_ADDRESS | 0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1)) {}
	LL_I2C_ClearFlag_ADDR(I2C1);

	for(int i=0; i<sizeof(i2c_rx_buffer); i++)
	{
		if(i < sizeof(i2c_rx_buffer) - 1)
		{
			// 마지막 바이트가 아니면 ACK
			LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
		}
		else
		{
			// 마지막 바이트를 수신하기 전에 NACK과 STOP을 미리 설정
			LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
			LL_I2C_GenerateStopCondition(I2C1);
		}

		while(!LL_I2C_IsActiveFlag_RXNE(I2C1)) {}
		i2c_rx_buffer[i] = LL_I2C_ReceiveData8(I2C1);
	}

	while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {}

	TX_PACKET *p = (TX_PACKET*)&i2c_rx_buffer[0];
	msg.scaled_pressure.time_boot_ms = msg.system_time.time_boot_ms;
	msg.scaled_pressure.temperature = p->angle_raw;

	msg.scaled_pressure.press_abs = p->angle;
	msg.scaled_pressure.press_diff = p->temp;
}

