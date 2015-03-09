/*
===============================================================================
 Name        : LPC_sample.c
 Author      : Riccardo Miccini
 Version     : 0.01
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/


#include "chip.h"
#include "board.h"
#include <cr_section_macros.h>
#include "string.h"
#include <stdlib.h>


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];


#define DEFAULT_I2C          I2C0

#define DEFAULT_MPU6050_ADDR 0x68

#define PWR_MGMT_1           0x6B
#define ACCEL_XOUT_H         0x3B
/* in order:
ACCEL_XOUT_H
ACCEL_XOUT_L
ACCEL_YOUT_H
ACCEL_YOUT_L
ACCEL_ZOUT_H
ACCEL_ZOUT_L
TEMP_OUT_H
TEMP_OUT_L
GYRO_XOUT_H
GYRO_XOUT_L
GYRO_YOUT_H
GYRO_YOUT_L
GYRO_ZOUT_H
GYRO_ZOUT_L */

#define I2C_DEFAULT_SPEED    100000
#define I2C_FASTPLUS_BIT     0

static int mode_poll;
static I2C_ID_T i2cDev = DEFAULT_I2C;


/*****************************************************************************
 * Private functions
 ****************************************************************************/

static void Init_PinMux(void)
{
	// check http://www.nxp.com/documents/user_manual/UM10398.pdf
	/* UART */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */

	/* I2C */
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC1 | I2C_FASTPLUS_BIT);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC1 | I2C_FASTPLUS_BIT);
}


/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	}
	else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if (!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(I2C0_IRQn);
	}
	else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(I2C0_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}


/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UART_IRQHandler(void)
{
	Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
}

/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

/**
 * @brief	Main UART program body
 * @return	Always returns 1
 */
int main(void) {
	uint8_t key = 0;
	uint8_t buff[14] = {0};

	SystemCoreClockUpdate();
    Board_Init();
	Init_PinMux();

	i2c_app_init(I2C0, I2C_DEFAULT_SPEED);

	/* Setup UART for 115.2K8N1 */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);

	/* Before using the ring buffers, initialize them using the ring
	   buffer init function */
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(UART0_IRQn, 1);
	NVIC_EnableIRQ(UART0_IRQn);


	// sets PWR_MGMT_1 register to 0 (no-sleep mode)
	uint8_t pwr_mgmt_data[] = {PWR_MGMT_1, 0};
	Chip_I2C_MasterSend(i2cDev, DEFAULT_MPU6050_ADDR, pwr_mgmt_data, 2);


	while (key != 'q') {
		Chip_UART_ReadRB(LPC_USART, &rxring, &key, 1);
		if (key == 'i') {
			//  send first register
			uint8_t temp_data[] = {ACCEL_XOUT_H};
			Chip_I2C_MasterSend(i2cDev, DEFAULT_MPU6050_ADDR, temp_data, 1);

			//  read response
			Chip_I2C_MasterRead(i2cDev, DEFAULT_MPU6050_ADDR, buff, 14);

			int16_t accX = (buff[0] << 8 | buff[1]);
			int16_t accY = (buff[2] << 8 | buff[3]);
			int16_t accZ = (buff[4] << 8 | buff[5]);
			int16_t temp = (buff[6] << 8 | buff[7]);
			temp = temp / 340 + 36.53;
			int16_t girX = (buff[8] << 8 | buff[9]);
			int16_t girY = (buff[10] << 8 | buff[11]);
			int16_t girZ = (buff[12] << 8 | buff[13]);

			char out_txt[UART_SRB_SIZE];
			char str[] =
					"Acceleration:\n"
					"X\t\tY\t\tZ\n"
					"%-8d%-8d%-8d\n\n"
					"Rotation:\n"
					"X\t\tY\t\tZ\n"
					"%-8d%-8d%-8d\n\n"
					"Temperature: %d *C\n\n";

			uint8_t len = sprintf(out_txt, str, accX, accY, accZ, girX, girY, girZ, temp);
			Chip_UART_SendRB(LPC_USART, &txring, &out_txt, len);
		}
	}

	/* DeInitialize UART0 peripheral */
	NVIC_DisableIRQ(UART0_IRQn);
	Chip_UART_DeInit(LPC_USART);

    volatile static int i = 0;
    while(1) {i++;}
    return 0;
}


