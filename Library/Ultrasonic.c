/*
===============================================================================
 Name        : finalProject.c
 Author      : Xihan Liu, Joyce Li, Yingchao Zhu
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "chip.h"
#include <time.h>
#include "Ultrasonic.h"
#include "Timer.h"
#include "GPIO.h"
#include <cr_section_macros.h>
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

// GPIO pin for interrupt/Trig p0.9
#define GPIO_Trig_PIN     9           // GPIO pin number	GPIO 23
#define GPIO_Trig_PORT    0            // GPIO port number	J5 18
// GPIO pin for interrupt/Echo	p0.24
#define GPIO_Echo_PIN     24            // GPIO pin number	GPIO 38
#define GPIO_Echo_PORT    0                // GPIO port number	J3 27
#define GPIO_IRQ_HANDLER              GPIO_IRQHandler/* GPIO interrupt IRQ function name */
#define GPIO_INTERRUPT_NVIC_NAME    GPIO_IRQn    /* GPIO interrupt NVIC interrupt name */

#define UART_SELECTION     LPC_UART0
#define IRQ_SELECTION     UART0_IRQn
#define HANDLER_NAME     UART0_IRQHandler

//#define TIMER0_IRQ_HANDLER                TIMER0_IRQHandler  // TIMER0 interrupt IRQ function name
//#define TIMER0_INTERRUPT_NVIC_NAME        TIMER0_IRQn        // TIMER0 interrupt NVIC interrupt name

#define DATA_SIZE 0x400
/* Work variables */
static volatile uint8_t channelTC, dmaChannelNum;
static volatile uint8_t DAC_Interrupt_Done_Flag, Interrupt_Continue_Flag;
static uint32_t DMAbuffer;

/* DAC sample rate request time */
#define DAC_TIMEOUT 0x3FF


/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 1024    /* Send */
#define UART_RRB_SIZE 1024    /* Receive */
#define BUFFER_FULL 0
#define BUFFER_EMPTY 1
#define BUFFER_AVAILABLE 2

typedef struct ring_buff {
	uint32_t buffer[256];
	uint8_t read_index;
	uint8_t write_index;
} Ring_Buffer_t;

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];




/* DMA routine for DAC example */
static void App_DMA_Test(uint8_t freq_sound)
{
	uint32_t tmp = 0;
	uint8_t uart_buffer = 0;

	DEBUGOUT("DMA mode selected\r\n");
	//DEBUGOUT(DirectionMenu);

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);

	/* Setup GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);

	/* Get the free channel for DMA transfer */
	dmaChannelNum = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_DAC);

	/* Output DAC value until get 'x' character */
	while (uart_buffer != 'x') {
//		uart_buffer = DEBUGIN();
//		if (uart_buffer == 'p') {
//			freq_sound++;
//		}
//		else if (uart_buffer == 'o') {
//			freq_sound--;
//		}

		/* Start D/A conversion */
		tmp += (freq_sound % DATA_SIZE);
		if (tmp == (DATA_SIZE - 1)) {
			tmp = 0;
		}

		/* pre-format the data to DACR register */
		DMAbuffer = (uint32_t) (DAC_VALUE(tmp) | DAC_BIAS_EN);
		channelTC = 0;
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
						  (uint32_t) &DMAbuffer,
						  GPDMA_CONN_DAC,
						  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
						  1);

		/* Waiting for writing DAC value completed */
		while (channelTC == 0) {}
	}

	/* Disable interrupts, release DMA channel */
	Chip_GPDMA_Stop(LPC_GPDMA, dmaChannelNum);
	NVIC_DisableIRQ(DMA_IRQn);
}


/**
 * @brief	DMA interrupt handler sub-routine
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
		channelTC++;
	}
	else {
		/* Error, do nothing */
	}
}


void HANDLER_NAME(void)
{
    /* Want to handle any errors? Do it here. */

    /* Use default ring buffer handler. Override this with your own
       code if you need more capability. */
    Chip_UART_IRQRBHandler(UART_SELECTION, &rxring, &txring);
}




// void delay(int milli_seconds)
// {
//     // Converting time into milli_seconds
//     //int milli_seconds = 1000 * number_of_seconds;
//
//     // Stroing start time
//     clock_t start_time = clock();
//
//     // looping till required time is not acheived
//     while (clock() < start_time + milli_seconds)
//         ;
// }
//
//void setup(){
//    Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN);    //sets the trigPin as an output
//    Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_Echo_PORT, GPIO_Echo_PIN);        //sets the echoPin as an input
//    //Serial.begin(9600);
//    Board_Init();
//}
//
//int getDistance(){
////	Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN);    //sets the trigPin as an output
////	Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_Echo_PORT, GPIO_Echo_PIN);        //sets the echoPin as an input
//	Chip_IOCON_PinMux(LPC_GPIO, GPIO_Trig_PORT,GPIO_Trig_PIN, IOCON_FUNC0, IOCON_MODE_PULLUP);	//configure pin as GPIO w/ pullup resistor
//    long duration = 0;	//unit ms
//    int distance;	//unit cm
//    bool state = false;
//    //bool s;
//    int PrescaleValue = 60000;  // Clock cycle / 1000 (set to ms increments)
//    //clear the trigPin
//    //true for high, false for low
//    Chip_GPIO_SetPinState(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN, false);
//   //s = Chip_GPIO_GetPinState(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN);
//   //printf("state for trigger: %d\n",s);
//    //delay for 2 ms
//    delay(2);
//    //set the trigPin on HIGH for 10 ms
//    Chip_GPIO_SetPinState(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN, false);
//    //s = Chip_GPIO_GetPinState(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN);
//    //printf("state for trigger: %d\n",s);
//    //delay for 10 ms
//    delay(10);
//    Chip_GPIO_SetPinState(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN, false);
//   // s = Chip_GPIO_GetPinState(LPC_GPIO, GPIO_Trig_PORT, GPIO_Trig_PIN);
//    //printf("state for trigger: %d\n",s);
//
//    //Caluculate the duration
//    state = Chip_GPIO_GetPinState(LPC_GPIO, GPIO_Echo_PORT, GPIO_Echo_PIN);
//    while(state == false){
//    	 state = Chip_GPIO_GetPinState(LPC_GPIO, GPIO_Echo_PORT, GPIO_Echo_PIN);
//    }
//    // Initialize TIMER0
//    Chip_TIMER_Init(LPC_TIMER0);                       // Initialize TIMER0
//    Chip_TIMER_PrescaleSet(LPC_TIMER0,PrescaleValue);  // Set prescale value
//    Chip_TIMER_Reset(LPC_TIMER0);
//    Chip_TIMER_Enable(LPC_TIMER0);
//    while(state == true){
//    	state = Chip_GPIO_GetPinState(LPC_GPIO, GPIO_Echo_PORT, GPIO_Echo_PIN);
//    	printf("state in true : %d\n",state);
//    }
//    duration = Chip_TIMER_ReadCount(LPC_TIMER0);
//    Chip_TIMER_Reset(LPC_TIMER0);
//    printf("duration: %d ms \n", duration);
//
//    //calculate distance
//
//    distance = duration*1000/58.82;
//    printf("Distance: %d cm \n", distance);
//    return distance;
//
//}



uint8_t isUltrasonicSensorTriggerEnded = 0;
uint8_t ultrasonicSensorEdgeCount      = 0;

void Ultrasonic_Trigger_Timer_Init() {
	IOCON_TRIGGER |= IOCON_TRIGGER_FUNC;

	PCONP |= 1 << 22;

	TIMER2->CTCR = 0x00;

	TIMER2->TCR &= ~(1 << 0);

	TIMER2->TCR |= (1 << 1);

	TIMER2->PR = PERIPHERAL_CLOCK_FREQUENCY / 1000000 - 1;

	//Write the Correct Configuration for EMR (Toggle Output Value and Initial value is HIGH)
	TIMER2->EMR |= ((1 << 3) | (1 << 11) | (1 << 10));

	//Enable TIMER2_IRQn (Interrupt Request).
	NVIC_EnableIRQ(TIMER2_IRQn);

	//Set Priority Timer2 IRQ as 5.
	NVIC_SetPriority(TIMER2_IRQn, 5);

	//Clear pendings for Timer2.
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
}

void Ultrasonic_Capture_Timer_Init() {
	IOCON_ECHO |= IOCON_ECHO_FUNC;

	PCONP |= 1 << 23;

	TIMER3->CTCR = 0x00;

	TIMER3->TCR &= ~(1 << 0);

	TIMER3->TCR |= (1 << 1);

	TIMER3->PR = PERIPHERAL_CLOCK_FREQUENCY / 1000000 - 1;

	TIMER3->CCR = (1 << 3) | (1 << 4) | (1 << 5);

	TIMER3->TCR &= ~(1 << 1);

	TIMER3->TCR |= (1 << 0);
}

void Ultrasonic_Start_Trigger() {
	//Give correct value to MR3 Register for 10 microsecond
	TIMER2->MR3 = 10;

	//Enable interrupt for MR3 register, if MR3 register matches the TC.
	TIMER2->MCR |= (1 << 9);

	//Remove the reset on counters of Timer2.
	TIMER2->TCR &= ~(1 << 1);

	//Enable Timer2 Counter and Prescale Counter for counting.
	TIMER2->TCR |= (1 << 0);
}

int Ultrasonic_Get_Distance() {
	ultrasonicSensorDuration = ultrasonicSensorFallingTime - ultrasonicSensorRisingTime;
	printf("fallingtime: %d",ultrasonicSensorFallingTime);
	printf("risingtime: %d", ultrasonicSensorRisingTime);
	return ultrasonicSensorDuration / 58;
}

void TIMER2_IRQHandler() {
	if (isUltrasonicSensorTriggerEnded == 0) {
		//Change MR3 Register Value for Suggested Waiting
		TIMER2->MR3 = 60000;

		isUltrasonicSensorTriggerEnded = 1;

		ultrasonicSensorEdgeCount = 0;

		//Clear pendings for Timer3.
		NVIC_ClearPendingIRQ(TIMER3_IRQn);

		//Enable TIMER3_IRQn (Interrupt Request).
		NVIC_EnableIRQ(TIMER3_IRQn);
	} else {
		TIMER2->MR3 = 10;
		isUltrasonicSensorTriggerEnded = 0;
	}

	//Clear IR Register Flag for Corresponding Interrupt
	TIMER2->IR |= (1 << 3);

	TIMER2->TC = 0;
}

uint32_t ultrasonicSensorRisingTime  = 0;
uint32_t ultrasonicSensorFallingTime = 0;
uint32_t ultrasonicSensorDuration    = 0;
uint32_t ultrasonicSensorDistance    = 0;

void TIMER3_IRQHandler() {
	if (ultrasonicSensorEdgeCount == 0) {
		ultrasonicSensorRisingTime = TIMER3->CR1;

		ultrasonicSensorEdgeCount = 1;

		NVIC_ClearPendingIRQ(TIMER3_IRQn);
	} else if (ultrasonicSensorEdgeCount == 1) {
		ultrasonicSensorFallingTime = TIMER3->CR1;

		ultrasonicSensorEdgeCount = 2;

		//Clear pendings for Timer3.
		NVIC_ClearPendingIRQ(TIMER3_IRQn);

		//Disable TIMER3_IRQn (Interrupt Request).
		NVIC_DisableIRQ(TIMER3_IRQn);
	}

	TIMER3->IR = 1 << 5;
}


void DAC_Init(){
	 uint8_t bufferUART;
	 uint32_t dacClk;
	 bool end_Flag = false;
	 /* Setup DAC pins for board and common CHIP code */
	 	Chip_DAC_Init(LPC_DAC);

	 	/* Setup DAC timeout for polled and DMA modes to 0x3FF */
	 #if defined(CHIP_LPC175X_6X)
	 	/* 175x/6x devices have a DAC divider, set it to 1 */
	 	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_DAC, SYSCTL_CLKDIV_1);
	 #endif
	 	Chip_DAC_SetDMATimeOut(LPC_DAC, DAC_TIMEOUT);

	 	/* Compute and show estimated DAC request time */
	 #if defined(CHIP_LPC175X_6X)
	 	dacClk = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_DAC);
	 #else
	 	dacClk = Chip_Clock_GetPeripheralClockRate();
	 #endif

	 	/* Enable count and DMA support */
	 	Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_CNT_ENA | DAC_DMA_ENA);

}

void UART_Init(){
	/* Setup UART for 115.2K8N1 */
	    Chip_UART_Init(UART_SELECTION);
	    Chip_UART_SetBaud(UART_SELECTION, 115200);
	    Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT)); //length 8 bits and 1 stop bit
	    Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	    Chip_UART_TXEnable(UART_SELECTION);

	    /* Before using the ring buffers, initialize them using the ring
	          buffer init function */
	    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
	    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

	    /* Enable receive data and line status interrupt */
	    Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

	    NVIC_SetPriority(IRQ_SELECTION, 1);
	    NVIC_EnableIRQ(IRQ_SELECTION);

}

 int main (void){

    Board_Init();
    /* Generic Initialization */
    SystemCoreClockUpdate();
    Board_UART_Init(UART_SELECTION);
    DAC_Init();
    UART_Init();


    //send msg to user about starting the sensor
    //setup();
    Ultrasonic_Trigger_Timer_Init();
    Ultrasonic_Capture_Timer_Init();
    Ultrasonic_Start_Trigger();
    Chip_UART_SendRB(UART_SELECTION, &txring, "Sensor starts working\n", sizeof("Sensor starts working\n")-1);

    while(1){
        Ultrasonic_Trigger_Timer_Init();
        Ultrasonic_Capture_Timer_Init();
        Ultrasonic_Start_Trigger();
	//    //get the distance
		uint32_t distanceResult;
		//distanceResult = 120;
		distanceResult = Ultrasonic_Get_Distance();
		//distanceResult = getDistance();
		printf("Distance: %d \n", distanceResult);

	//    //found the distance is greater than 50
		//send msg to the user
		if(distanceResult > 20){
			Chip_UART_SendRB(UART_SELECTION, &txring, "Alarm!!!\n", sizeof("Alarm!!!\n")-1);
			Chip_UART_SendRB(UART_SELECTION, &txring, "Invader!!!\n", sizeof("Invader!!!\n")-1);
			// speaker warning
		   // App_DMA_Test(0x88);
		}
		else{
			Chip_UART_SendRB(UART_SELECTION, &txring, "You are safe\n", sizeof("You are safe\n")-1);
		}
    }

    return 0;
}
