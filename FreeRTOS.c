/*
 *
 *
 * ****************************************************************************
 *					ECE 544 - PROJECT TWO : PID
 * 				Author : Kiyasul Arif | Rahul Marathe
 *							VERSION : 3.3
 *						Including Watchdog
 * ****************************************************************************
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?".  Have you defined configASSERT()?  *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *   Investing in training allows your team to be as productive as       *
     *   possible as early as possible, lowering your overall development    *
     *   cost, and enabling you to bring a more robust product to market     *
     *   earlier than would otherwise be possible.  Richard Barry is both    *
     *   the architect and key author of FreeRTOS, and so also the world's   *
     *   leading authority on what is the world's most popular real time     *
     *   kernel for deeply embedded MCU designs.  Obtaining your training    *
     *   from Richard ensures your team will gain directly from his in-depth *
     *   product knowledge and years of usage experience.  Contact Real Time *
     *   Engineers Ltd to enquire about the FreeRTOS Masterclass, presented  *
     *   by Richard Barry:  http://www.FreeRTOS.org/contact
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    You are receiving this top quality software for free.  Please play *
     *    fair and reciprocate by reporting any suspected issues and         *
     *    participating in the community forum:                              *
     *    http://www.FreeRTOS.org/support                                    *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/
/****************************** Includes ***********************************/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
/* Std includes. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/***************************** Include Files *********************************/
/* Project Two includes. */
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "stdbool.h"
#include "math.h"
#include "xil_types.h"
#include "xstatus.h"
#include "xparameters.h"
#include "xtmrctr.h"
#include "xwdttb.h"
/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters 0 : FreeRTOS
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0

// AXI timer parameters 1 : PWM OUT
#define AXI_TIMER1_DEVICE_ID	XPAR_AXI_TIMER_1_DEVICE_ID
#define AXI_TIMER1_BASEADDR		XPAR_AXI_TIMER_1_BASEADDR
#define AXI_TIMER1_HIGHADDR		XPAR_AXI_TIMER_1_HIGHADDR
#define TmrCtrNumber1			0

// AXI timer parameters 2 : Clk Nexys4IO
#define AXI_TIMER2_DEVICE_ID	XPAR_AXI_TIMER_2_DEVICE_ID
#define AXI_TIMER2_BASEADDR		XPAR_AXI_TIMER_2_BASEADDR
#define AXI_TIMER2_HIGHADDR		XPAR_AXI_TIMER_2_HIGHADDR
#define TmrCtrNumber2			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID			XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR			XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR			XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ		CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ			40000
#define FIT_COUNT					(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC				40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_OUTPUT_0_CHANNEL		1

// GPIO parameters
#define GPIO_1_DEVICE_ID			XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_1_INPUT_0_CHANNEL		1

// Interrupt Controller parameters
#define INTC_DEVICE_ID				XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID			XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR
#define RPM_INTERRUPT_ID			XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR
#define WDT_INTERRUPT_ID			XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR

/**************************** Type Definitions ******************************/
#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9
#define mainQUEUE_LENGTH	( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

/************************** Constant Definitions *****************************/
#define PWM_TIMER_WIDTH		32
#define PWM_MAXCNT			4294967295.00

#define PWM_PERIOD_TIMER	0
#define PWM_DUTY_TIMER		1

#define COMMON_LSB_MASK_0	( 0x0000000F )
#define COMMON_LSB_MASK_1	( 0x000000F0 )
#define PMOD_SP_MASK		( 0x00000003 )
#define PMOD_PC_MASK		( 0x0000000C )
#define PMOD_SPC_MASK		( 0x00000030 )

#define ZERO				( 0 )
/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
uint64_t 	timestamp = 0L;

/******************************** Pmods *************************************/
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;

/******************************** Gpios *************************************/
XGpio		xOutputGPIOInstance;					// GPIO 0 instance
XGpio		xInputGPIOInstance;					    // GPIO 1 instance

/************************* Interrupts and Timer ******************************/
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM clock timer instance for Nexys4IO
XTmrCtr		PWMTimerInst;				// PWM motor control timer counter instance
XWdtTb 		*InstancePtr;
/************************** Semaphores and Queues ****************************/
//Declare a Sempahore for the PID thread
xSemaphoreHandle xPID = 0;

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueueDisplayRPM = 0;
static xQueueHandle xQueueDisplayKp  = 0;
static xQueueHandle xQueueDisplayKi  = 0;
static xQueueHandle xQueueDisplayKd  = 0;

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
volatile u32			gpio1_rpm_in;			// GPIO rpm input port
volatile u32			gpio1_rpm_in_p;			// GPIO rpm input port - previous

/************************** Variable Definitions *****************************/
float clock_frequency;		// clock frequency for the timer.  Usually the AXI bus clock
u32 *frequency, *dutyfactor;

/************************** Function Prototypes ******************************/
int PWM_Initialize(XTmrCtr *InstancePtr, u16 DeviceId, bool EnableInterrupts, u32 clkfreq);
int PWM_Start(XTmrCtr *InstancePtr);
int PWM_Stop(XTmrCtr *InstancePtr);
int PWM_SetParams(XTmrCtr *InstancePtr, u32 freq, u32 dutyfactor);
int PWM_GetParams(XTmrCtr *InstancePtr, u32 *freq, u32 *dutyfactor);

/************************** Variable Definitions *****************************/
/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void RPM_Handler(void);										// rpm interrupt handler
void WDT_Handler(void);										// wdt interrupt handlerb
void watchdog_timer(void);									// wdt initialize

void init_struct();											// init struct variables
void init_display();										// init display variables

int AXI_Timer0_initialize(void);
int AXI_Timer1_initialize(void);
int AXI_Timer2_initialize(void);
/*-----------------------------------------------------------*/

/* The threads/tasks as described at the top of this file. */
//static void master_thread( void *pvParameters );

static void input_thread( void *pvParameters );
static void display_thread( void *pvParameters );
static void pid_thread( void *pvParameters );
/*-----------------------------------------------------------*/
/************* Value Event Change Listeners ******************/
void pmodENC_valueEventChangeListener(void);
void boardButtons_valueEventChangeListener(void);

void rpm_to_duty(int);
void averageout_rpm(void);
void switch_leds(void);
/*************************************************************/
// Tired of changing the driver every time
int ENC_getRotation(u32 state, u32 laststate, u32 ticks, u16 switchstate);

/************************ PWM *******************************/
typedef struct {

	int DUTY;
    u32 PERIOD;

}PWM;

/*-----------------------------------------------------------*/
/************************ PID ********************************/
typedef struct {

	u8 Kp;
	u8 Ki;
	u8 Kd;

	int current_rpm;
	int target_rpm;

	double error;
	double prev_error;
	double integral;
	double derivative;

	u32 rpm_counter;
	u32 led_value;			// Based on PID lit up LEDs

}PIDControl;
void PID_Handler(PIDControl* pid, int);						    // PID algorithm
/*-----------------------------------------------------------*/
typedef struct {

	u32 state; //comparing current and previous state to detect edges on GPIO pins.
	u32 laststate;

	int ticks;
	int lastticks;
	int directionstate;
	int directionlaststate;

	u16 switchstate;		// Common switch state
	u16 speedstate; 		// Switch [1:0] : Motor speed +-1 ; +-5 ; +-10
	u16 proportionstate; 	// Switch [3:2] : Control Proportion Constants
	u16 proportionincstate; // Switch [5:4] : Control Proportion Speed speed +-1 ; +-5 ; +-10

}PmodSENC;

// Instances for the structure
PWM 		MOTOR;
PIDControl 	PID;
PmodSENC	pmodENC;

void init_struct()
{
	// ENC structure
	pmodENC.ticks 	  = 0;
	pmodENC.lastticks = 1;

	// PID structure
	PID.Kp 			  = 0;
	PID.Ki			  =	0;
	PID.Kd            = 0;

	PID.led_value	  = 0;
	PID.prev_error	  = 0;

	PID.current_rpm   = 0;
	PID.target_rpm    = 0;
}

void init_display()
{
	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kp: 0");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Ki: 0");
	// To display 0 initially : KA
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kd: 0");
	// To display 0 initially : KA
	// To show the Prof/TA whether HW/SW detection is selected
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"RPM: 0");

	// blank the display digits and turn off the decimal points
	NX410_SSEG_setAllDigits(SSEGLO, CC_0, CC_0, CC_0, CC_0, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_0, CC_0, CC_0, CC_0, DP_NONE);
}

int main( void )
{
	init_struct();

	uint32_t sts; // status check variable

	xil_printf( "Hello from FreeRTOS example main\r\n" );

	// Initialize all the peripherals and interrupts
	sts = do_init();
	if(XST_SUCCESS != sts) {
		exit(1);
	}

	microblaze_enable_interrupts();

	// Init OLED
	init_display();

	/* Create the queue */
	xQueueDisplayRPM = xQueueCreate( mainQUEUE_LENGTH, sizeof( int ) );
	xQueueDisplayKp  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	xQueueDisplayKi  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	xQueueDisplayKd  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );

	/* Sanity check that the queue was created. */
	configASSERT( xQueueDisplayRPM );
	configASSERT( xQueueDisplayKp );
	configASSERT( xQueueDisplayKi );
	configASSERT( xQueueDisplayKd );

	//Create Semaphore
	vSemaphoreCreateBinary( xPID );

	// Create Input Thread
	xTaskCreate( input_thread,
	   ( const char * ) "IT",
						2048,
						NULL,
						1,
						NULL );

	// Create PID Thread
	xTaskCreate( pid_thread,
	   ( const char * ) "PT",
						2048,
						NULL,
						3,
						NULL );

	//Create Display Thread
	xTaskCreate( display_thread,
	   ( const char * ) "DT",
						2048,
						NULL,
						2,
						NULL );

	//Start the Scheduler
	vTaskStartScheduler();

	return -1;

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details.*/
	for( ;; );
}


/****************************************************************************/
/**
* The input thread reads the buttons and switches state on the Nexy4DDR
*       
* Based on the state of the physical buttons and switches appropriate messages is
* sent to the display that. But before that the higher priority task PID thread is 
* given the semaphore to perform the motor control algorithm
*
* @param void pointer (points to nothing) can be used to pass params
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/ 
void input_thread( void *pvParameters ) {
	// get the previous state
	pmodENC.laststate = ENC_getState( &pmodENC_inst );
	pmodENC.directionlaststate = ENC_switchOn( pmodENC.state );
	for( ;; ) {

		pmodENC_valueEventChangeListener();
		boardButtons_valueEventChangeListener();

		averageout_rpm();
		switch_leds();

		if( PID.current_rpm != PID.target_rpm ) xSemaphoreGive( xPID );

		/* Delay for 1 second. */
		vTaskDelay( 1 );
	}
}

/****************************************************************************/
/**
* The display thread recieves messages from the two other threads and 
* updates the variables on the PmodOLEDrgb 
*       
* The target RPM, Kp, Ki and Kd constants are updated
*
* @param void pointer (points to nothing) can be used to pass params
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void display_thread( void *pvParameters ) {

	int RPM = 0;
	int Kp = 0, Ki = 0, Kd = 0;
	for( ;; ) {

		if( xQueueReceive( xQueueDisplayRPM, &RPM, mainDONT_BLOCK ) ) {
			xil_printf("Entered DISPLAY RPM THREAD \n");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 7);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 7);
			PMDIO_putnum(&pmodOLEDrgb_inst, RPM, 10);
		}

		if( xQueueReceive( xQueueDisplayKp, &Kp, mainDONT_BLOCK ) ) {
			xil_printf("Entered DISPLAY KP THREAD \n");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
			PMDIO_putnum(&pmodOLEDrgb_inst, Kp, 10);
		}
		if( xQueueReceive( xQueueDisplayKi, &Ki, mainDONT_BLOCK ) ) {
			xil_printf("Entered DISPLAY KI THREAD \n");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
			PMDIO_putnum(&pmodOLEDrgb_inst, Ki, 10);
		}
		if( xQueueReceive( xQueueDisplayKd, &Kd, mainDONT_BLOCK ) ) {
			xil_printf("Entered DISPLAY KD THREAD \n");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
			PMDIO_putnum(&pmodOLEDrgb_inst, Kd, 10);
		}
		/* Delay for 1 second. */
		vTaskDelay( 1 );
	}
}

/****************************************************************************/
/**
* The PID thread is locked using a binary semaphore, the lock is given to this thread
* when there is a change on the input thread.
*       
* The PID thread perfomrs the PID algorithm to reach the target speed / setpoint.
*
* @param void pointer (points to nothing) can be used to pass params
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void pid_thread( void *pvParameters ) {

	PWM_Initialize(&PWMTimerInst, AXI_TIMER1_DEVICE_ID, false, CPU_CLOCK_FREQ_HZ);

	for( ;; ) {

		if( xSemaphoreTake( xPID, portMAX_DELAY ) ) {

			// Update the duty cycle based on PID algorithm
			if( PID.Kp != ZERO || PID.Ki != ZERO || PID.Kd != ZERO ) {
				PID_Handler(&PID, pmodENC.ticks);
				PWM_SetParams(&PWMTimerInst, (u32) 4000, (u32) MOTOR.DUTY);
				PWM_Start(&PWMTimerInst);
			}

			else {
				rpm_to_duty(pmodENC.ticks);
				xil_printf("The current rpm is %d \n", PID.current_rpm);
				xil_printf("The target rpm is %d \n", PID.target_rpm);
				PWM_SetParams(&PWMTimerInst, (u32) 4000, (u32) MOTOR.DUTY);
				PWM_Start(&PWMTimerInst);
			}
		}
		/* Delay for 1 second. */
		vTaskDelay(1);
	}
}

/****************************************************************************/
/**
* This function is used to read the encoder state change values
*       
* The target rpm increases when the encoder is rotated clockwise and vice-versa
* The switch on the encoder changes the direction of the motor
*
* @param   *NONE*
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void pmodENC_valueEventChangeListener( void ) {

	// switch state to determine speed of motor
	pmodENC.switchstate = NX4IO_getSwitches();

	// Switch [1:0]
	pmodENC.speedstate = ( ( pmodENC.switchstate & COMMON_LSB_MASK_0 ) & PMOD_SP_MASK );

	// get the PmodENC state
	pmodENC.state  		   = ENC_getState( &pmodENC_inst );
	pmodENC.directionstate = ENC_switchOn( pmodENC.state );

	/************************************ TODO *************************************************/
	/*****************************  DIRECTION CHANGE *******************************************/
	if( pmodENC.directionstate != pmodENC.directionlaststate ) {

		// Set motor speed to zero 0
		pmodENC.ticks = 0;
		// Give semaphore back to PID thread
		xSemaphoreGive( xPID );

		// Wait till the current motor speed is zero and then jump
		while ( XGpio_DiscreteRead(&xInputGPIOInstance, GPIO_1_INPUT_0_CHANNEL) != ZERO ) {
			xil_printf("The current ticks is %d \n", XGpio_DiscreteRead(&xInputGPIOInstance, GPIO_1_INPUT_0_CHANNEL));
		}

		if( pmodENC.directionstate != 8 ) {
			// Set direction to rotate in counter clock
			xil_printf("Value 1 is written");
			XGpio_DiscreteWrite( &xOutputGPIOInstance, GPIO_0_OUTPUT_0_CHANNEL, (u32) 1 );
		}
		else {
			// Set direction to rotate in counter clock
			xil_printf("Value 0 is written");
			XGpio_DiscreteWrite( &xOutputGPIOInstance, GPIO_0_OUTPUT_0_CHANNEL, (u32) 0 );
		}
		// previous value but different direction
		pmodENC.ticks = pmodENC.lastticks;
		// Give semaphore back to PID thread
		xSemaphoreGive( xPID );

		// Send message to button thread
		if(!xQueueSend( xQueueDisplayRPM, &pmodENC.ticks, portMAX_DELAY )) {
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}
	}

	// BTNU is not pressed so increment count
	pmodENC.ticks += ENC_getRotation(pmodENC.state, pmodENC.laststate, pmodENC.ticks, pmodENC.speedstate);

	if (pmodENC.ticks != pmodENC.lastticks) {
		if(pmodENC.ticks >= 0 && pmodENC.ticks <= 15000) {// Send message to display thread to show latest duty factor
			if(!xQueueSend( xQueueDisplayRPM, &pmodENC.ticks, portMAX_DELAY )) {
				xil_printf("Failed to send message to DISPLAY THREAD \n");
			}
			xSemaphoreGive( xPID );
		}
	}

	// Update the previous and last values
	pmodENC.laststate = pmodENC.state;
	pmodENC.lastticks = pmodENC.ticks;
	pmodENC.directionlaststate = pmodENC.directionstate;
}

/****************************************************************************/
/**
* This function is used to read the buttons state on the board
*       
* The primary function of the up and down buttons increments/decrements the 
* contants (Kp,Ki,Kd) and the center button stops the motor (brings it to a halt)
*
* @param   *NONE*
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void boardButtons_valueEventChangeListener( void ) {

	// Switch [3:2]
	pmodENC.proportionstate    = ( ( pmodENC.switchstate & COMMON_LSB_MASK_0 ) & PMOD_PC_MASK  ) >> 2;
	// Switch [5:4]
	pmodENC.proportionincstate = ( ( pmodENC.switchstate & COMMON_LSB_MASK_1 ) & PMOD_SPC_MASK ) >> 4;

	//xil_printf("The proportion inc state is %d \n", pmodENC.proportionincstate);

	// Center button functionality
	if( NX4IO_isPressed(BTNC) ) {

		// Set motor speed to zero 0
		pmodENC.ticks = 0;
		// Give semaphore back to PID thread
		xSemaphoreGive( xPID );
	}

	// Up and Down button functionality
	if( NX4IO_isPressed(BTNU) ) {

		switch(pmodENC.proportionstate) {

			case 0 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						++PID.Kp;
						break;
					case 1:
						PID.Kp += 5;
						break;
					case 2:
						PID.Kp += 10;
						break;
					case 3:
						PID.Kp += 10;
						break;
					default:
						++PID.Kp;
						break;
				}
				if(PID.Kp >= 100) {
					PID.Kp = 100;
				}
				if(!xQueueSend( xQueueDisplayKp,  &PID.Kp, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			case 1 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						++PID.Ki;
						break;
					case 1:
						PID.Ki += 5;
						break;
					case 2:
						PID.Ki += 10;
						break;
					case 3:
						PID.Ki += 10;
						break;
					default:
						++PID.Ki;
						break;
				}
				if(PID.Ki >= 100) {
					PID.Ki = 100;
				}
				if(!xQueueSend( xQueueDisplayKi,  &PID.Ki, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			case 2 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						++PID.Kd;
						break;
					case 1:
						PID.Kd += 5;
						break;
					case 2:
						PID.Kd += 10;
						break;
					case 3:
						PID.Kd += 10;
						break;
					default:
						++PID.Kd;
						break;
				}
				if(PID.Kd >= 100) {
					PID.Kd = 100;
				}
				if(!xQueueSend( xQueueDisplayKd,  &PID.Kd, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			case 3 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						++PID.Kd;
						break;
					case 1:
						PID.Kd += 5;
						break;
					case 2:
						PID.Kd += 10;
						break;
					case 3:
						PID.Kd += 10;
						break;
					default:
						++PID.Kd;
						break;
				}
				if(PID.Kd >= 100) {
					PID.Kd = 100;
				}
				if(!xQueueSend( xQueueDisplayKd,  &PID.Kd, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			default :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						++PID.Kp;
						break;
					case 1:
						PID.Kp += 5;
						break;
					case 2:
						PID.Kp += 10;
						break;
					case 3:
						PID.Kp += 10;
						break;
					default:
						++PID.Kp;
						break;
				}
				if(PID.Kp >= 100) {
					PID.Kp = 100;
				}
				if(!xQueueSend( xQueueDisplayKp,  &PID.Kp, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
		}
		// Give semaphore back to PID thread
		xSemaphoreGive( xPID );
	}
	else if( NX4IO_isPressed(BTND) ) {

		switch(pmodENC.proportionstate) {

			case 0 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						--PID.Kp;
						break;
					case 1:
						PID.Kp -= 5;
						break;
					case 2:
						PID.Kp -= 10;
						break;
					case 3:
						PID.Kp -= 10;
						break;
					default:
						--PID.Kp;
						break;
				}
				if(PID.Kp >= 100 ) {
					PID.Kp = 0;
				}
				if(!xQueueSend( xQueueDisplayKp,  &PID.Kp, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			case 1 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						--PID.Ki;
						break;
					case 1:
						PID.Ki -= 5;
						break;
					case 2:
						PID.Ki -= 10;
						break;
					case 3:
						PID.Ki -= 10;
						break;
					default:
						--PID.Ki;
						break;
				}
				if(PID.Ki >= 100) {
					PID.Ki = 0;
				}
				if(!xQueueSend( xQueueDisplayKi,  &PID.Ki, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			case 2 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						--PID.Kd;
						break;
					case 1:
						PID.Kd -= 5;
						break;
					case 2:
						PID.Kd -= 10;
						break;
					case 3:
						PID.Kd -= 10;
						break;
					default:
						--PID.Kd;
						break;
				}
				if(PID.Kd >= 100) {
					PID.Kd = 0;
				}
				if(!xQueueSend( xQueueDisplayKd,  &PID.Kd, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			case 3 :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						--PID.Kd;
						break;
					case 1:
						PID.Kd -= 5;
						break;
					case 2:
						PID.Kd -= 10;
						break;
					case 3:
						PID.Kd -= 10;
						break;
					default:
						--PID.Kd;
						break;
				}
				if(PID.Kd >= 100) {
					PID.Kd = 0;
				}
				if(!xQueueSend( xQueueDisplayKd,  &PID.Kd, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
			default :
				switch ( pmodENC.proportionincstate ) {
					case 0:
						--PID.Kp;
						break;
					case 1:
						PID.Kp -= 5;
						break;
					case 2:
						PID.Kp -= 10;
						break;
					case 3:
						PID.Kp -= 10;
						break;
					default:
						--PID.Kp;
						break;
				}
				if(PID.Kp >= 100) {
					PID.Kp = 0;
				}
				if(!xQueueSend( xQueueDisplayKp,  &PID.Kp, portMAX_DELAY )) {
					xil_printf("Failed to send message to DISPLAY THREAD \n");
				}
				break;
		}
		// Give semaphore back to PID thread
		xSemaphoreGive( xPID );
	}
}

/****************************************************************************/
/**
* This function is used to perform the PID algorithm
*       
* The references are taken from Prof.Roy's lecture slides
*
* @param   PIDControl * - a pointer to the struct which contains all the necessary 
*						  variabels for the algorithm
*		   target_speed - the target speed of the motor
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void PID_Handler( PIDControl * pid, int target_speed ) {

	double temp_kp = 0, temp_ki = 0, temp_kd = 0;
	int    temp_duty = 0;

	pid->target_rpm = target_speed;
	xil_printf("The target rpm is %d \n", pid->target_rpm);

	// Calculate the error
	pid->error = (double)( pid->target_rpm - pid->current_rpm );

	// Calculate the integral
	pid->integral += pid->error;

	// Calculate the derivative
	pid->derivative = pid->error - pid->prev_error;

	// boundary check for the integral : limit the values
	if( pid->integral >  1000) pid->integral =  1000;
	if( pid->integral < -1000) pid->integral = -1000;

	temp_kp = (double)pid->Kp / (double)100;
	temp_ki = (double)pid->Ki / (double)100;
	temp_kd = (double)pid->Kd / (double)100;

	// Calculate the control variable
	MOTOR.DUTY = (int) (( temp_kp * pid->error ) + (temp_ki * pid->integral ) + ( temp_kd  * pid->derivative ));

	// Limit the control variable to within +- 100
	if      ( MOTOR.DUTY > 100 || MOTOR.DUTY  < -100 ) {
		MOTOR.DUTY = 99;
	}
	else if ( MOTOR.DUTY < 0   ) {
		MOTOR.DUTY = -(MOTOR.DUTY);
	}
	else
		xil_printf("The duty within bounds %d \n", MOTOR.DUTY);

	// Update the previous state
	pid->prev_error = pid->error;
}

/****************************************************************************/
/**
* This function is used to scale the target rpm to duty cycle 
* Sweep mode without the PID algorithm
*       
* Python scripts were run and a scaling factor was found for the entire 
* range of the rpm.
*
* @param   target_rpm - the target speed of the motor
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void rpm_to_duty( int target_rpm ) {

	// scaled factors for the sweep mode
	if(target_rpm == 1000) MOTOR.DUTY = 60;
	else if (target_rpm > 1000 && target_rpm < 4000)
	MOTOR.DUTY = (u32)((double)((double)target_rpm * 0.006 ) +  60);
	else if (target_rpm > 4000 && target_rpm < 7200)
	MOTOR.DUTY = (u32)((double)((double)target_rpm * 0.0026 ) + 75);
	else if (target_rpm > 7200 && target_rpm < 11000)
	MOTOR.DUTY = (u32)((double)((double)target_rpm * 0.0025 ) + 80);
	else if (target_rpm > 11000)
	MOTOR.DUTY = (u32)((double)((double)target_rpm * 0.0024 ) + 90);

	PID.target_rpm = target_rpm;
}

/****************************************************************************/
/**
* This function is used to calculate the moving average of the rpm from the
* hall sensor ouptut. Primarily used for debug (xil_printf) purposes
*
* @param   *NONE*
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void averageout_rpm( void ) {

	// Read the GPIO port One to read back the generated PWM signal for RGB led's
	PID.current_rpm = XGpio_DiscreteRead(&xInputGPIOInstance, GPIO_1_INPUT_0_CHANNEL);
	temp    += gpio1_rpm_in;
	++PID.rpm_counter;

	if(PID.rpm_counter == 100) {
		temp /= 100;
		xil_printf("The RPM from the hall sensor is %d \n", temp);
		PID.rpm_counter = 0;
	}
}

/****************************************************************************/
/**
* This function is used to switch on the on board LEDs[2:0] based on the active
* proportion constants
*
* @param   *NONE*
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void switch_leds( void ) {

	// Debug purposes
	if( PID.Kp != ZERO)  PID.led_value = PID.led_value | 0x00000004;
	else			     PID.led_value = PID.led_value & 0x00000003;
	if( PID.Ki != ZERO)  PID.led_value = PID.led_value | 0x00000002;
	else			     PID.led_value = PID.led_value & 0x00000005;
	if( PID.Kd != ZERO)  PID.led_value = PID.led_value | 0x00000001;
	else		         PID.led_value = PID.led_value & 0x00000006;

	NX4IO_setLEDs( PID.led_value );
}

/****************************************************************************/
/**
* This function is used initialize the watchdog timer 
*       
* The steps were followed based on the documentatio provided by Digilent 
* watchdog timebase api
*
* @param   *NONE*
*
* @return  *NONE*
*
* @note    *NONE*
*****************************************************************************/
void watchdog_timer(void) {

	u32 ControlStatusRegister0;
	u32 Mask;
	u32 RegValue;
	u32 SecWindow;

	/***************************************** STEP ONE ***************************************/
	/* Read the current contents */
	ControlStatusRegister0 =
		XWdtTb_ReadReg(InstancePtr->Config.BaseAddr,
			XWT_TWCSR0_OFFSET);
	/* The watchdog has expired if either of the bits are set */
	Mask = XWT_CSR0_WRS_MASK | XWT_CSR0_WDS_MASK;

	/* Check whether state and reset status */
	if ((ControlStatusRegister0 & Mask) != (u32)0) {
		Mask = (u32)TRUE;
	}
	else {
		Mask = (u32)FALSE;
	}

	/***************************************** STEP TWO ****************************************/
	/*
	 * Read the current contents of TCSR0 so that subsequent writes
	 * to the register won't destroy any other bits
	 */
	ControlStatusRegister0 =
		XWdtTb_ReadReg(InstancePtr->Config.BaseAddr,
			XWT_TWCSR0_OFFSET);
	/*
	 * Clear the bit that indicates the reason for the last
	 * system reset, WRS and the WDS bit, if set, by writing
	 * 1's to TCSR0
	 */
	ControlStatusRegister0 |= ((u32)XWT_CSR0_WRS_MASK |
		(u32)XWT_CSR0_WDS_MASK);

	/* Indicate that the device is started before we enable it */
	InstancePtr->IsStarted = XIL_COMPONENT_IS_STARTED;

	/*
	 * Set the registers to enable the watchdog timer, both enable
	 * bits in TCSR0 and TCSR1 need to be set to enable it
	 */
	XWdtTb_WriteReg(InstancePtr->Config.BaseAddr,
		XWT_TWCSR0_OFFSET, (ControlStatusRegister0 |
			(u32)XWT_CSR0_EWDT1_MASK));

	/***************************************** STEP THREE ****************************************/
	/*
	 * Clear the bit that indicates the reason for the last
	 * system reset, WRS and the WDS bit, if set, by writing
	 * 1's to TCSR0
	 */
	ControlStatusRegister0 |= ((u32)XWT_CSR0_WRS_MASK |
		(u32)XWT_CSR0_WDS_MASK);

	XWdtTb_WriteReg(InstancePtr->Config.BaseAddr,
		XWT_TWCSR0_OFFSET, ControlStatusRegister0);

	/****************************************** STEP FOUR *******************************************/

	/* Read enable status register and update WINT bit */
	RegValue = XWdtTb_ReadReg(InstancePtr->Config.BaseAddr,
		XWT_ESR_OFFSET) | XWT_ESR_WINT_MASK;

	SecWindow = (RegValue & XWT_ESR_WSW_MASK) >> XWT_ESR_WSW_SHIFT;

	/*
	 * Check WDT in second window. If WDT is in second window, toggle WSW
	 * bit to avoid restart kick before clearing interrupt programmed
	 * point
	 */
	if (SecWindow == (u32)1) {
		RegValue &= ~((u32)XWT_ESR_WSW_MASK);
	}

	/* Write enable status register with updated WINT and WSW bit */
	XWdtTb_WriteReg(InstancePtr->Config.BaseAddr, XWT_ESR_OFFSET,
		RegValue);
}








































































/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/
int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&xOutputGPIOInstance, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the GPIO instances
	status = XGpio_Initialize(&xInputGPIOInstance, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// GPIO 0 output DIR to Pmod HB3
	XGpio_SetDataDirection(&xOutputGPIOInstance, GPIO_0_OUTPUT_0_CHANNEL, 0x00000000);

	// GPIO 1 input RPM DETECT from Hardware module
	XGpio_SetDataDirection(&xInputGPIOInstance,  GPIO_1_INPUT_0_CHANNEL,  0xFFFFFFFF);

	status = AXI_Timer2_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}

	/*********** TODO **************/
	//watchdog_timer();

	// connect the rpm interval timer (RPM) handler to the interrupt
/*	status = XIntc_Connect(&IntrptCtlrInst, RPM_INTERRUPT_ID,
						   (XInterruptHandler)RPM_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// connect the rpm interval timer (RPM) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, WDT_INTERRUPT_ID,
						   (XInterruptHandler)WDT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}*/

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the RPM interrupt
	XIntc_Enable(&IntrptCtlrInst, RPM_INTERRUPT_ID);

	// enable the RPM interrupt
	XIntc_Enable(&IntrptCtlrInst, WDT_INTERRUPT_ID);

	return XST_SUCCESS;
}


/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer2_initialize(void){

	uint32_t status;    // status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER2_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}

	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber2);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}

	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER2_BASEADDR, TmrCtrNumber2,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER2_BASEADDR, TmrCtrNumber2, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER2_BASEADDR, TmrCtrNumber2);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER2_BASEADDR, TmrCtrNumber2);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER2_BASEADDR, TmrCtrNumber2, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER2_BASEADDR, TmrCtrNumber2);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER2_BASEADDR, TmrCtrNumber2, ctlsts);

	XTmrCtr_Enable(AXI_TIMER2_BASEADDR, TmrCtrNumber2);
	return XST_SUCCESS;
}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/*************************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion,
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
***************************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;

  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;

    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/*****************************************************************************/
/**
* Initializes a  timer/counter instance/driver for PWM use.
*
* Initialize fields of the XTmrCtr structure and set the control bits for PWM usage.
* Uses both high level and low level tmrctr driver functions
*
* @param    InstancePtr is a pointer to the XTmrCtr instance to be used for PWM.
* @param    DeviceId is the unique id of the device controlled by this XTmrCtr
*           component.  Passing in a device id associates the generic XTmrCtr
*           component to a specific device, as chosen by the caller or
*           application developer.
* @param	EnableInterrupts is a boolean indicating whether the interrupt for this
*			timer should be enabled (true) or not (false)
* @param	clkfreq is the input clock frequency for the timer
*
* @return
*
*   - XST_SUCCESS if initialization was successful
*   - XST_DEVICE_IS_STARTED if the device has already been started
*   - XST_DEVICE_NOT_FOUND if the device doesn't exist
*
******************************************************************************/
int PWM_Initialize(XTmrCtr *InstancePtr, u16 DeviceId, bool EnableInterrupts, u32 clkfreq)
{
    int StatusReg;
    u32		PWM_BaseAddress;
    u32		ctlbits;

    // Initialize the timer/counter instance
    // This clears  both timer registers and any pending interrupts
    StatusReg = XTmrCtr_Initialize(InstancePtr, DeviceId);
    if (StatusReg != XST_SUCCESS) // failed to initialize.  Return the reason
    {
	    return StatusReg;
    }

    // successfully initialized the timer/ctr instance
	// initialize timer to PWM mode with interrupts enabled (or not)
	PWM_BaseAddress = InstancePtr->BaseAddress;
	if (EnableInterrupts)
	{
		ctlbits = XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK  | XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_DOWN_COUNT_MASK | XTC_CSR_ENABLE_INT_MASK;
	}
	else
	{
		ctlbits = XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK  | XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_DOWN_COUNT_MASK;
	}
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER, ctlbits);
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_DUTY_TIMER, ctlbits);

	// save the timer clock frequency
	clock_frequency = (float) clkfreq;

	return XST_SUCCESS;
}


/*****************************************************************************/
/**
* Starts the specified PWM timer
*
* Starts the specified PWM instance of the device such that it starts running.
* The timer counter is reset before it is started and the reset value is
* loaded into the timer counter.  Assumes that the PWM timer instance has been
* initialized successfully and that the compare (Load) registers have been
* loaded with the period (TLR0) and duty cycle (TLR1)
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
*
* @return
*
*   - XST_SUCCESS if the PWM timers were started
*   - XST_FAILURE if the PWM instance is not initialized
*
******************************************************************************/
int PWM_Start(XTmrCtr *InstancePtr)
{
	u32		ctlbits;
    u32		PWM_BaseAddress;

    if (InstancePtr->IsReady != XIL_COMPONENT_IS_READY) // check that timer instance is initialized
    {
	    return XST_FAILURE;
    }

    // instance was initialized - reset (load TLRx) the timers
    PWM_BaseAddress = InstancePtr->BaseAddress;
    XTmrCtr_LoadTimerCounterReg(PWM_BaseAddress, PWM_PERIOD_TIMER);
	ctlbits = XTmrCtr_GetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER) & 0xFFFFFFDF;  // clear load bits
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER, ctlbits);

    XTmrCtr_LoadTimerCounterReg(PWM_BaseAddress, PWM_DUTY_TIMER);
	ctlbits = XTmrCtr_GetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER) & 0xFFFFFFDF;  // clear load bits
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_DUTY_TIMER, ctlbits);

	// and enable (start) both timers  - ENABLE-ALL is shadowed in both TCSR registers
	ctlbits = XTmrCtr_GetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER);
	ctlbits |= XTC_CSR_ENABLE_ALL_MASK;
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER, ctlbits);
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* PWM_Stop() - Stops the specified PWM instance
*
* Stops the specified PWM instance of the device. Assumes that the PWM timer
* instance has been initialized successfully.
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
*
* @return
*
*   - XST_SUCCESS if the PWM timers were stopped
*   - XST_FAILURE if the PWM instance is not initialized
*
******************************************************************************/
int PWM_Stop(XTmrCtr *InstancePtr)
{
    u32		PWM_BaseAddress;

    if (InstancePtr->IsReady != XIL_COMPONENT_IS_READY) // check that instance is initialized
    {
	    return XST_FAILURE;
    }

    // instance was initialized - stop the timers
    PWM_BaseAddress = InstancePtr->BaseAddress;
	XTmrCtr_Disable(PWM_BaseAddress, PWM_PERIOD_TIMER);
	XTmrCtr_Disable(PWM_BaseAddress, PWM_DUTY_TIMER);
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* PWM_SetParams() - Set the PWM parameters
*
* Sets the frequency and duty cycle for the PWM.  Stops the PWM timers but does not
* restart them.  Assumes that the PWM timer instance has been initialized and that the
* timer is running at "clock_frequency" Hz (which was passed in during initialization)
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
* @param    PWM frequency (in Hz).
* @param	PWM high time (in pct of PWM period - 0 to 100)
*
* @return
*
*   - XST_SUCCESS if the PWM parameters were loaded
*   - XST_FAILURE if the PWM instance is not initialized
*	- XST_INVALID_PARAM if one or both of the parameters is invalid
*
* @note
* Formulas for calculating counts (PWM counters are configured as down counters):
* 	TLR0 (PWM period count) = (PWM_PERIOD / TIMER_CLOCK_PERIOD) - 2
* 	TLR1 (PWM duty cycle count) = MAX( 0, (((PWM_PERIOD * (DUTY CYCLE / 100)) / TIMER_CLOCK_PERIOD) - 2) )
*
******************************************************************************/
int PWM_SetParams(XTmrCtr *InstancePtr, u32 freq, u32 dutyfactor)
{
	u32		PWM_BaseAddress;
	float	timer_clock_period,
			pwm_period,
			pwm_dc,
			tlr0,
			tlr1;

    if (InstancePtr->IsReady != XIL_COMPONENT_IS_READY) // check that instance is initialized
    {
	    return XST_FAILURE;
    }

    // calculate the PWM period and high time
	timer_clock_period = 1.0 / clock_frequency;
	pwm_period = 1.0 / freq;
	tlr0 = (pwm_period / timer_clock_period) - 2;

	pwm_dc = dutyfactor / 100.00;
	tlr1 = ((pwm_period * pwm_dc) / timer_clock_period) - 2;
	if (tlr1 < 0)   // duty cycle cannot be less than 0%
	{
		tlr1 = 0.0;
	}

	// check to see if parameters are valid
    if (dutyfactor > 100)  // cannot have a duty cylce > 100%
    {
	   return XST_INVALID_PARAM;
	}
	if ((tlr0 > PWM_MAXCNT) || (tlr1 > PWM_MAXCNT))  // period or high time is too big for the timer/counter registers
	{
		return XST_INVALID_PARAM;
	}

	// period and duty cycle are within range of timer - stop timer and write values to load registers
    PWM_Stop(InstancePtr);
    PWM_BaseAddress = InstancePtr->BaseAddress;
    XTmrCtr_SetLoadReg(PWM_BaseAddress, PWM_PERIOD_TIMER, (u32) tlr0);
  	XTmrCtr_SetLoadReg(PWM_BaseAddress, PWM_DUTY_TIMER, (u32) tlr1);
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* PWM_GetParams() - Get the PWM parameters
*
* Returns the frequency (Hz) and duty cycle (%) for the PWM.  Stops the PWM timers but does not
* restart them.  Assumes that the PWM timer instance has been initialized and that the
* timer is running at the PLB bus frequency (PLB_CLOCK_FREQ_HZ) as defined in pwm_tmrctr.h
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
* @param    pointer to PWM frequency (in Hz).
* @param	pointer to PWM high time (in pct of PWM period - 0 to 100)
*
* @return
*
*   - XST_SUCCESS if the PWM parameters were loaded
*   - XST_FAILURE if the PWM instance is not initialized
*	- XST_INVALID_PARAM if one or both of the parameters is invalid
*
* @note
*
* Formulas for calculating counts (PWM counters are configured as down counters):
*		TIMER_CLOCK_PERIOD = 1 / TIMER_CLOCK_FREQ
*		PWM_PERIOD = (TLR0 + 2) x (1 / TIMER_CLOCK_FREQ)
*		PWM_HIGH_TIME = (TLR1 + 2) x (1 / TIMER_CLOCK_FREQ)
*
******************************************************************************/
int PWM_GetParams(XTmrCtr *InstancePtr, u32 *freq, u32 *dutyfactor)
{
	u32		PWM_BaseAddress;
	float	timer_clock_period,
			pwm_period,
			pwm_dc,
			tlr0,
			tlr1;

	u32		tlr;

    if (InstancePtr->IsReady != XIL_COMPONENT_IS_READY) // check that instance is initialized
    {
	    return XST_FAILURE;
    }

    // first stop the PWM timers and get Base Address of timer registers
	PWM_Stop(InstancePtr);
	PWM_BaseAddress = InstancePtr->BaseAddress;

	// next read the load registers to get the period and high time
 	tlr = XTmrCtr_GetLoadReg(PWM_BaseAddress, PWM_PERIOD_TIMER);
 	tlr0 = (float) tlr;
 	tlr = XTmrCtr_GetLoadReg(PWM_BaseAddress, PWM_DUTY_TIMER);
 	tlr1 = (float) tlr;

    // calculate the PWM period and high time
	timer_clock_period = 1.0 / clock_frequency;
	pwm_period = (tlr0 + 2) * timer_clock_period;
	pwm_dc = tlr1 / tlr0;

	// round the values and return them
	*freq = lroundf(1.00 / pwm_period);
	*dutyfactor = lroundf(pwm_dc * 100.00);
	return XST_SUCCESS;
}






/***************************************************************************************/
/* ------------------------------------------------------------ */
/***	int ENC_getRotation(u32 state, u32 laststate)
**
**	Parameters:
**		state, the most recent GPIO Pin state
**		laststate, the second most recent GPIO Pin state
**
**	Return Value:
**		The direction of motion of the encoder shaft
**
**	Errors:
**		Sign is currently arbitrary, not mapped onto a physical clockwise/counterclockwise format
**
**	Description:
**		checks states to see if there has been a positive edge on pin A, if not, no movement
**		if so, checks pin B to see whether it's waveform is leading or following pin A's
**
*/
int ENC_getRotation(u32 state, u32 laststate, u32 ticks, u16 switchstate)
{
	//	if posedge(pinA), then
	//		if B is low, return RIGHT
	//		if B is high, return LEFT
	if ((state & ENC_GPIO_PIN_A) != 0 && (laststate & ENC_GPIO_PIN_A) == 0)
	{
		if ((state & ENC_GPIO_PIN_B) != 0)
		{
			switch (switchstate) {

			case 0 :
				return 1;
				break;
			case 1 :
				return 5;
				break;
			case 2 :
				return 10;
				break;
			case 3:
				return 10;
				break;
			default :
				return 1;
				break;
			}

		}
		else
		{
			if( ticks <= 0)
			return 0;
			else {

				switch (switchstate) {

				case 0 :
					return -1;
					break;
			    case 1 :
			    	return -5;
			    	break;
				case 2 :
					return -10;
					break;
				case 3:
					return -10;
					break;
				default :
					return -1;
					break;
				}
			}
		}
	}
	else
	{
		return 0;
	}
}


















/******************************* Some useful functions *********************************/
/******************************* Can be used later *************************************/
/*const TickType_t x10seconds = pdMS_TO_TICKS( DELAY_10_SECONDS );
static TimerHandle_t xTimer = NULL;
static void vTimerCallback( TimerHandle_t pxTimer );

 Create the two tasks.  The Tx task is given a lower priority than the
	Rx task, so the Rx task will leave the Blocked state and pre-empt the Tx
	task as soon as the Tx task places an item in the queue.
	xTaskCreate( 	 prvTxTask, 					 The function that implements the task.
					 ( const char * ) "Tx", 		 Text name for the task, provided to assist debugging only.
					 configMINIMAL_STACK_SIZE, 	 The stack allocated to the task.
					 NULL, 						 The task parameter is not used, so set to NULL.
					 tskIDLE_PRIORITY,			 The task runs at the idle priority.
					 &xTxTask );

	xTaskCreate(	 prvRxTask,
				 	 ( const char * ) "Rx",
					 configMINIMAL_STACK_SIZE,
					 NULL,
					 tskIDLE_PRIORITY + 1,
					 &xRxTask );

	 Create the queue used by the tasks.  The Rx task has a higher priority
	than the Tx task, so will preempt the Tx task and remove values from the
	queue as soon as the Tx task writes to the queue - therefore the queue can
	never have more than one item in it.
	xQueue = xQueueCreate( 	1,						 There is only one space in the queue.
							sizeof( HWstring ) );	 Each space in the queue is large enough to hold a uint32_t.

	 Check the queue was created.
	configASSERT( xQueue );

	 Start the tasks and timer running.
	vTaskStartScheduler();
*/
/* Create a timer with a timer expiry of 10 seconds. The timer would expire
	 after 10 seconds and the timer call back would get called. In the timer call back
	 checks are done to ensure that the tasks have been running properly till then.
	 The tasks are deleted in the timer call back and a message is printed to convey that
	 the example has run successfully.
	 The timer expiry is set to 10 seconds and the timer set to not auto reload.
	xTimer = xTimerCreate( (const char *) "Timer",
							x10seconds,
							pdFALSE,
							(void *) TIMER_ID,
							vTimerCallback);
	 Check the timer was created.
	configASSERT( xTimer );

	 start the timer with a block time of 0 ticks. This means as soon
	   as the schedule starts the timer will start running and will expire after
	   10 seconds
	xTimerStart( xTimer, 0 );


-----------------------------------------------------------
static void vTimerCallback( TimerHandle_t pxTimer )
{
	long lTimerId;
	configASSERT( pxTimer );

	lTimerId = ( long ) pvTimerGetTimerID( pxTimer );

	if (lTimerId != TIMER_ID) {
		xil_printf("FreeRTOS Hello World Example FAILED");
	}

	 If the RxtaskCntr is updated every time the Rx task is called. The
	 Rx task is called every time the Tx task sends a message. The Tx task
	 sends a message every 1 second.
	 The timer expires after 10 seconds. We expect the RxtaskCntr to at least
	 have a value of 9 (TIMER_CHECK_THRESHOLD) when the timer expires.
	if (RxtaskCntr >= TIMER_CHECK_THRESHOLD) {
		xil_printf("FreeRTOS Hello World Example PASSED");
	} else {
		xil_printf("FreeRTOS Hello World Example FAILED");
	}

	vTaskDelete( xRxTask );
	vTaskDelete( xTxTask );
}*/


/******************************* Some useful functions *********************************/
/******************************* Can be used later *************************************/


/********************************************************************************

*************************** INTERRUPT HANDLERS ********************************

*******************************************************************************
*******************************************************************************
*
* RPM interval timer interrupt handler
*
* Reads the GPIO port which reads the hall sensor SA and throws an interrupt
* which is handled over here
 ****************************************************************************/
void RPM_Handler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// Read the GPIO port One to read back the generated PWM signal for RGB led's
	gpio1_rpm_in = XGpio_DiscreteRead(&xInputGPIOInstance, GPIO_1_INPUT_0_CHANNEL);

	xil_printf("The RPM value read from the interrupt handler is %d \n", gpio1_rpm_in);

	// Update the previous value with the current value at the end of each cycle
	gpio1_rpm_in_p = gpio1_rpm_in;

	if( gpio1_rpm_in != gpio1_rpm_in_p ) {

	}

/*	 The event has occurred, use the semaphore to unblock the task so the task
	can process the event.*/
	xSemaphoreGiveFromISR( xPID, &xHigherPriorityTaskWoken );
	xil_printf( "FIT \n" );
/*     Clear the interrupt here.
	 Now the task has been unblocked a context switch should be performed if
	xHigherPriorityTaskWoken is equal to pdTRUE. NOTE: The syntax required to perform
	a context switch from an ISR varies from port to port, and from compiler to
	compiler. Check the web documentation and examples for the port being used to
	find the syntax required for your application.*/
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void WDT_Handler(void) {
	// indefinite loop 
	// WDT currently disabled due to improper behaviour of reset
}
