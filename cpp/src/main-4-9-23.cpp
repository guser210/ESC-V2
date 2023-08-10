/*
 * main.cpp
 * ESC_V1.4_TEST_Variable
 *  Created on: July 18, 2022
 *      Author: gvargas
 */

#include "main.h"

extern "C"
{
extern TIM_HandleTypeDef htim1;		// Signal input capture.
extern TIM_HandleTypeDef htim3;		// PWM driver.
extern TIM_HandleTypeDef htim7;		// Generic timer.

extern TIM_HandleTypeDef htim6;		// RPM.

extern TIM_HandleTypeDef htim16;	// commutator delay.
extern TIM_HandleTypeDef htim17;	// commutator bump timer.
extern  ADC_HandleTypeDef hadc1;	// battery voltages.
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel6;
extern DMA_HandleTypeDef hdma_tim1_ch3;

extern DMA_HandleTypeDef hdma_tim3_ch2;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4;

extern volatile uint8_t initComplete;


}

volatile uint8_t disableBump;

volatile uint32_t eRPM= 0;
volatile uint32_t RPM = 0;
volatile uint32_t RPMOld = 0;
volatile uint32_t maxRPM = 0;

volatile uint8_t newDmaSignal = 0;
volatile uint16_t dmaSignal[32] = {0};
volatile uint16_t dmaSignalESC[32] = {0};
volatile uint16_t dmaSignalMem2Mem[32] = {0};
volatile int16_t dmaSignalDiff[33] = {0};
volatile uint16_t dmaSignalNormalized[32] = {0};
volatile uint16_t dmaESCSignal[33] = {0};
volatile uint16_t motorSpeed = 0;
volatile uint16_t motorSpeedCurrent = 0;

//void beepNoSPin(uint16_t ms ,uint16_t ps );
void beepNoSPin(uint16_t ms = 40,uint16_t ps = 24);
inline uint16_t map(int x, int in_min, int in_max, int out_min, int out_max);
//////////////////////////////////////////////////////////////////////
const uint16_t dmaPulse =  1;
const uint16_t dmaPulseReload = 1024;//2048;//1024;//512;
//const uint16_t dmaPulse = 2;
//const uint16_t dmaPulseReload = 512;
volatile uint16_t dmaBuffer[dmaPulse] = {0};
volatile uint16_t resolution = dmaPulse * dmaPulseReload;
void setDutyCycle(uint16_t dc)
{
	if( dc > resolution) dc = resolution;

//	int16_t half = dc>>1;
//	int16_t one = (dc & 1);
//	dmaBuffer[0] = half + one;// (dc & 1) + (dc>>1);
//	dmaBuffer[1] = half;

	if( dc > resolution) dc = resolution;
	dmaBuffer[0] = dc;
}


const uint32_t zcOff[6] = {//
		ZC_B_Pin, // B
		ZC_A_Pin, // A
		ZC_C_Pin, // C
		ZC_B_Pin, // B
		ZC_A_Pin, // A
		ZC_C_Pin, // C
};

const uint32_t odLow[6] = {//
		OD_C_Pin, // C
		OD_C_Pin, // C
		OD_A_Pin, // A
		OD_A_Pin, // A
		OD_B_Pin, // B
		OD_B_Pin, // B
};
const uint32_t odHigh[6] = {//
		OD_A_Pin, // A
		OD_B_Pin, // B
		OD_B_Pin, // B
		OD_C_Pin, // C
		OD_C_Pin, // C
		OD_A_Pin, // A
};

const uint32_t odOff[6] = {//
		OD_B_Pin, // B
		OD_A_Pin, // A
		OD_C_Pin, // C
		OD_B_Pin, // B
		OD_A_Pin, // A
		OD_C_Pin, // C
};
const DMA_HandleTypeDef dmaOff[6] = {
		(DMA_HandleTypeDef)hdma_tim3_ch3, // B
		(DMA_HandleTypeDef)hdma_tim3_ch2, // A
		(DMA_HandleTypeDef)hdma_tim3_ch4, // C
		(DMA_HandleTypeDef)hdma_tim3_ch3, // B
		(DMA_HandleTypeDef)hdma_tim3_ch2, // A
		(DMA_HandleTypeDef)hdma_tim3_ch4, // C
};


const uint32_t ccOff[6] = {// TIME3->CCER For pwm output, RM0454 page 495.
		1<<8, // B
		1<<4, // A
		1<<12, // C
		1<<8, // B
		1<<4, // A
		1<<12, // C
};

const uint32_t diOff[6] = {// TIM3->DIER for PWM output RM0454 page .
		1<<11, // B
		1<<10, // A
		1<<12, // C
		1<<11, // B
		1<<10, // A
		1<<12, // C
};
const uint32_t ccOn[6] = {// TIME3->CCER For pwm output, RM0454 page 495.
		1<<4, // A
		1<<8, // B
		1<<8, // B
		1<<12, // C
		1<<12, // C
		1<<4, // A
};

const uint32_t diOn[6] = {// TIM3->DIER for PWM output RM0454 page .
		1<<10, // A
		1<<11, // B
		1<<11, // B
		1<<12, // C
		1<<12, // C
		1<<10, // A
};

const GPIO_TypeDef* zcPortOff[6] = {//
		(GPIO_TypeDef*)ZC_B_GPIO_Port, // B
		(GPIO_TypeDef*)ZC_A_GPIO_Port, // A
		(GPIO_TypeDef*)ZC_C_GPIO_Port, // C
		(GPIO_TypeDef*)ZC_B_GPIO_Port, // B
		(GPIO_TypeDef*)ZC_A_GPIO_Port, // A
		(GPIO_TypeDef*)ZC_C_GPIO_Port, // C
};



volatile uint32_t resetImrFlags = 0;
volatile uint8_t powerStep = 0;
volatile uint8_t powerStepCurrent = 0;
volatile uint8_t powerStepNext = 0;

const uint8_t rising[2][6] = {{1,0,1,0,1,0},
							  {0,1,0,1,0,1}};

volatile uint8_t reverse = 0;

#define zcPinOn ZC_A_Pin | ZC_B_Pin | ZC_C_Pin
volatile uint16_t commutationCNT  = 0;
volatile uint16_t commutationCntPrev  = 0;
volatile uint16_t commutation6Step  = 0;
volatile uint8_t motorSpinning = 0;

//volatile uint8_t falsePositives = 0;

inline void commutate()
{
	// go to next power step.

	powerStep = powerStepNext;

	static uint16_t odr = 0;
	/*
	Step 0  1   2  3  4  5      0  1  2  3  4  5

	High A  B   B  C  C  A      A  B  B  C  C  A
	Off  B  A   C  B  A  C      B  A  C  B  A  C
	Low  C  C   A  A  B  B      C  C  A  A  B  B
	*/


	if( rising[reverse][powerStep])
	{

//		GPIOA->BRR = odOff[powerStep];
//
//		TIM3->CCER &= ~ccOff[powerStep];
//		TIM3->DIER &= ~diOff[powerStep];
//
//		TIM3->CCER |= ccOn[powerStep];
//		TIM3->DIER |= diOn[powerStep];
//
//		///GPIOA->BSRR = odLow[powerStep];
//		GPIOA->BSRR = odHigh[powerStep];
//		GPIOA->BSRR = odLow[powerStep];
//
//		EXTI->FTSR1 = 0;
//		EXTI->RTSR1 = zcOff[powerStep];

	// if zc rising
	//		enable zc rising interrupt
	//		enable odLow


		//GPIOA->BRR = odOff[powerStep]; // disable odOff
		//GPIOA->BSRR = odLow[powerStep];
		//GPIOA->ODR//
		odr = GPIOA->ODR;
		odr &= ~odOff[powerStep];
		odr |= odLow[powerStep];
		GPIOA->ODR = odr;

		TIM3->CCER |= (ccOff[powerStep]);
	//		enable dma on timer 3.
		TIM3->DIER |= (diOff[powerStep]);

		EXTI->FTSR1 = 0;
		EXTI->RTSR1 = zcOff[powerStep];

	}else
	{
		/*
		Step 0  1   2  3  4  5      0  1  2  3  4  5

		High A  B   B  C  C  A      A  B  B  C  C  A
		Off  B  A   C  B  A  C      B  A  C  B  A  C
		Low  C  C   A  A  B  B      C  C  A  A  B  B
		*/

//		GPIOA->BRR = odOff[powerStep];
//
//		TIM3->CCER &= ~(ccOff[powerStep]);
//		TIM3->DIER &= ~(diOff[powerStep]);
//
//		TIM3->CCER |= ccOn[powerStep];
//		TIM3->DIER |= diOn[powerStep];
//
//		//GPIOA->BSRR = odLow[powerStep];
//		////GPIOA->BSRR = odHigh[powerStep];
//		GPIOA->BSRR = odLow[powerStep];
//
//		EXTI->RTSR1 = 0;
//		EXTI->FTSR1 = zcOff[powerStep];



	// else
	//		enable zc falling int.
	//		enable odHigh.
//		GPIOA->BRR = odOff[powerStep]; // disable odOff
//		GPIOA->BSRR = odHigh[powerStep];

		odr = GPIOA->ODR;
		odr &= ~odOff[powerStep];
		odr |= odHigh[powerStep];
		GPIOA->ODR = odr;


		TIM3->CCER &= ~(ccOff[powerStep]);
		//		disable dma on timer 3
		TIM3->DIER &= ~(diOff[powerStep]);

		EXTI->RTSR1 = 0;
		EXTI->FTSR1 = zcOff[powerStep];
	}

	powerStepCurrent++;
	powerStepCurrent %= 6;

//	if( motorSpeedCurrent == 0)
//		motorSpinning = 0;
	if( TIM7->ARR == 0) //|| motorSpeedCurrent == 0)
	{
		HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	}

	if( reverse)
		powerStepNext = 5 - powerStepCurrent;
	else
		powerStepNext = powerStepCurrent; // forward.

	if( powerStepNext == 0)
	{
		commutationCNT = TIM6->CNT;
		TIM6->CNT = 0;
	}

	if( RPM < 5000 && initComplete > 0)
	{ //2-25-2023 - delay to compensate for ringing, only needed below 500 and 3uS are enough,
		// in this case more is better but only at low or startup, the 5000RPM is for fast throttle ups.

		TIM7->CNT = 0;
			while( TIM7->CNT < 128);// 512);
	}
	// enable wakeup with interrupt.
	EXTI->IMR1 = resetImrFlags | zcOff[powerStep];
	// clear any zc pending interrupts.
	EXTI->FPR1 = zcPinOn; ;
	EXTI->RPR1 = zcPinOn;  ;
	// reset timer17 for bump motor.
	TIM17->CNT = 0;
}


volatile uint32_t newARR = 2047;//3076;
volatile uint16_t checkRising = 0;
volatile uint16_t checkFalling = 0;



void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{

	TIM7->CNT = 0;
	while( TIM7->CNT < checkRising)
		if( (zcPortOff[powerStep]->IDR & zcOff[powerStep]) == 0)
			return;

//	falsePositives = 0;
//	if( powerStep == 1 || powerStep == 4)
//		signal_GPIO_Port->BSRR = signal_Pin;

	commutate();
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{


	TIM7->CNT = 0;
	while( TIM7->CNT < checkFalling)
		if( (zcPortOff[powerStep]->IDR & zcOff[powerStep]) )
			return;

//	falsePositives = 0;
//	if( powerStep == 1 || powerStep == 4)
//		signal_GPIO_Port->BRR = signal_Pin;

	commutate();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim17 )
	{

//		if( reverse == 0)
//			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		if( motorSpeedCurrent > 0)
		{// Bump
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			commutate();
		}
	}

}

//////////////////////////////////////////////////////////////////////


uint16_t divClosest( uint16_t a, uint16_t b)
{
	return (a + b/ 2) / b;
}


volatile uint16_t pwm[3] = {0};

volatile uint16_t pwmRising[5] = {0};
volatile uint16_t pwmFalling[5] = {0};
volatile uint8_t pwmAvgCounter = 0;


volatile uint8_t transmitterDirection = 0;
volatile uint8_t masterDirection = 0;
volatile uint16_t pwm1  = 0;
volatile uint16_t pwm2  = 0;

volatile uint8_t directionValidator = 0;



//volatile uint16_t throttle = 0;
//volatile uint16_t throttlePlusTel = 0;
//volatile uint16_t crcIn = 0;
//volatile uint16_t crc = 0;
//volatile uint16_t telemetryBit = 0;

volatile uint32_t numberOfTimesDshotProcessed = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotMiddleOfFrame = 0; // Debug variable.
volatile uint32_t numberOfTimesDshotCRCError = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotThrottleOver = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotTimeAdvace = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotResync = 0; // Debug Variable

volatile uint16_t debugCounter = 0;

volatile uint16_t dmaSignalNormalized2[16] = {0};
static volatile uint8_t errorReads = 0;

static volatile uint16_t throttle = 0;
static volatile uint16_t dshotCommand = 0;
static volatile int16_t throttlePlusTel = 0;
static volatile uint16_t crcIn = 0;
static volatile uint16_t crc = 0;
static volatile uint16_t telemetryBit = 0;
static volatile uint16_t signal[16] = {0};



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{ // guser210

	if( htim == &htim17 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{

		TIM17->CNT = 0;
		if(initComplete > 0 && motorSpeedCurrent > 0 )
		{// Bump
			LED_GPIO_Port->ODR ^= LED_Pin;
			commutate();
		}else
			if( motorSpeedCurrent == 0 )

		{
//			GPIOA->BRR = odOff[0];
//			GPIOA->BRR = odOff[1];
//			GPIOA->BRR = odOff[2];
//			EXTI->FTSR1 = 0;
//			EXTI->RTSR1 = 0;

		}
	}
	else if( htim == &htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		TIM1->DIER &= ~((1<<1) | (1<<4)); // Disable interrupts for channel 1 and 4.

		// Enable Signal DMA.
		TIM1->CNT = 0;
		hdma_tim1_ch3.Instance->CNDTR = 32;// First set number of bytes to transfer
		hdma_tim1_ch3.Instance->CCR |= 1; // second enable DMA channel.

		numberOfTimesDshotResync++;

	}
}
void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *htim)
{

	if(htim == &htim1)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(newDmaSignal == 0)
			{
				//MEM2MEM launch.
				hdma_memtomem_dma1_channel6.Instance->CNDTR = 32;
				hdma_memtomem_dma1_channel6.Instance->CCR |= 1;
			}
			newDmaSignal = 1;

			// reset CNT  to prevent early overflow.
			TIM1->CNT = 0;
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			TIM1->CCR1 = TIM1->CNT + 120; // advance OC of channel 1 if rising edge.

			// increment debug counter.
			numberOfTimesDshotTimeAdvace++;
		}
	}
}
volatile int motorSpeedDirection = 1;
volatile uint32_t beepCounter = 0;
volatile uint8_t direction = 0;
volatile uint8_t DShotCommandCount = 0;
volatile uint8_t DShotCommandList[128] ={0};



// TODO:Step 3

inline void processDmaSignalDShot()
{

	/* Gap 0 = 1.07, 1 = 504
	 * Protocol Type 		Dshot 150 	Dshot 300	Dshot 600	Dshot 1200
	 * Period time			6.67 µs 	3.33 µs 	1.67 µs 	0.833 µs
	 * Zero high time (T0H) 2.50 µs 	1.25 µs 	0.625 µs 	0.313 µs
	 * One high time (T1H) 	5.00 µs 	2.50 µs 	1.25 µs 	0.625 µs
	 * https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
	 */

	numberOfTimesDshotProcessed++;

	/*
	 * wait for mem2mem to complete, this is unlikley to happen since it takes time
	 * to get to this funciton. is a safety check.
	 */

	while(hdma_memtomem_dma1_channel6.Instance->CNDTR > 0);
	// Disble mem2mem channel... this is required for re-activation.
	hdma_memtomem_dma1_channel6.Instance->CCR &= ~(1);

	//check if middle of frame.
	if( dmaSignalESC[0] > dmaSignalESC[31])// || dmaSignalESC[15] > dmaSignalESC[31])
	{
		TIM1->CNT = 0; // reset counter to 0 to prevent overflows.
		TIM1->CCR1 = 50;
		hdma_tim1_ch3.Instance->CCR &= ~(1);// Disable Signal DMA
		TIM1->DIER |= (1<<1) | (1<<4); // enable interrupts for Channels 1 and 4.

		numberOfTimesDshotMiddleOfFrame++;
		newDmaSignal = 0;
		return; // invalid data exit function.
	}


	static uint8_t ShiftBy = 5;


	signal[ 0] = (dmaSignalESC[ 1] - dmaSignalESC[ 0])>>ShiftBy;
	signal[ 1] = (dmaSignalESC[ 3] - dmaSignalESC[ 2])>>ShiftBy;
	signal[ 2] = (dmaSignalESC[ 5] - dmaSignalESC[ 4])>>ShiftBy;
	signal[ 3] = (dmaSignalESC[ 7] - dmaSignalESC[ 6])>>ShiftBy;
	signal[ 4] = (dmaSignalESC[ 9] - dmaSignalESC[ 8])>>ShiftBy;
	signal[ 5] = (dmaSignalESC[11] - dmaSignalESC[10])>>ShiftBy;
	signal[ 6] = (dmaSignalESC[13] - dmaSignalESC[12])>>ShiftBy;
	signal[ 7] = (dmaSignalESC[15] - dmaSignalESC[14])>>ShiftBy;
	signal[ 8] = (dmaSignalESC[17] - dmaSignalESC[16])>>ShiftBy;
	signal[ 9] = (dmaSignalESC[19] - dmaSignalESC[18])>>ShiftBy;
	signal[10] = (dmaSignalESC[21] - dmaSignalESC[20])>>ShiftBy;
	signal[11] = (dmaSignalESC[23] - dmaSignalESC[22])>>ShiftBy;
	signal[12] = (dmaSignalESC[25] - dmaSignalESC[24])>>ShiftBy;
	signal[13] = (dmaSignalESC[27] - dmaSignalESC[26])>>ShiftBy;
	signal[14] = (dmaSignalESC[29] - dmaSignalESC[28])>>ShiftBy;
	signal[15] = (dmaSignalESC[31] - dmaSignalESC[30])>>ShiftBy;

	newDmaSignal = 0;

	throttlePlusTel  = (signal[0] )<<11
			| (signal[ 1] )<<10
			| (signal[ 2] )<<9
			| (signal[ 3] )<<8
			| (signal[ 4] )<<7
			| (signal[ 5] )<<6
			| (signal[ 6] )<<5
			| (signal[ 7] )<<4
			| (signal[ 8] )<<3
			| (signal[ 9] )<<2
			| (signal[10] )<<1
			| (signal[11] );


	throttle = throttlePlusTel>>2;
	dshotCommand = throttlePlusTel>>1;

	crcIn = signal[12]<<3
			| signal[13]<<2
	 	 	| signal[14]<<1
	 	 	| signal[15];

	// Calculate CRC from 12 MSB.
	crc = (throttlePlusTel ^ (throttlePlusTel >> 4) ^ (throttlePlusTel >> 8)) & 0x0F;

	if( crc != crcIn  || throttle > 1024 ) {
		if( crc != crcIn)
			numberOfTimesDshotCRCError++; // Increment debug counter.
		if( throttle > 1024)
			numberOfTimesDshotThrottleOver++; // increment debug counter.
		if(errorReads++ > 10)
		{

			if( motorSpeed > 1)
				motorSpeed--;
			errorReads = 0;
		}

		TIM1->CNT = 0; // reset counter to 0 to prevent overflows.
		TIM1->CCR1 = 120;
		hdma_tim1_ch3.Instance->CCR &= ~(1);// Disable Signal DMA
		TIM1->DIER |= (1<<1) | (1<<4); // enable interrupts for Channels 1 and 4.

		newDmaSignal = 0;
		return; // invalid data exit function.
	}

	// get telemetry bi.
	telemetryBit = signal[11];

	if(dshotCommand > 0 && dshotCommand < 48 )
	{
		DShotCommandList[DShotCommandCount++] = dshotCommand;
		DShotCommandCount %= 128;

		switch(dshotCommand)
		{
		case 21: // Motor direction: Reverse
			if(masterDirection == direction)
			{
				if( direction == 1)
					direction = 0;
				else
					direction = 1;
			}

			break;
		case 20: // Motor direction: Normal
			direction = masterDirection;
			break;
		case 1: case 2: // beep 1 and 2
			//beepNoSPin(260);
			break;
		case 3: case 4: // beep 3 and 4
			//beepNoSPin(280);
			break;
		case 5: // beep 5
			//beepNoSPin(1020);
			break;
		default:
			break;
		}

	}

	errorReads = 0;
	newDmaSignal = 0;
	motorSpeed = throttle > 47 ? throttle + 1 : 0;// throttle;
}


void processDmaSignal()
{
	newDmaSignal = 0;


	for( int index = 0; index < 15; index++)
		dmaSignalNormalized[index] = divClosest( dmaSignal[index + 1] - dmaSignal[index], 8) - 8;




//
	if(dmaSignalNormalized[0] > 1000 &&
			dmaSignalNormalized[1] > 1000 &&
			dmaSignalNormalized[2] > 1000)
	{
		return;
	}

	for( int index = 0; index <= 10; index++)
	{
		if( dmaSignalNormalized[index] < 100 )
			continue;
		if( dmaSignalNormalized[index + 1] > 3000)
			continue;

		uint16_t motorSpeedNewValue = dmaSignalNormalized[index + 1]<<6 |
					 dmaSignalNormalized[index + 3]<<2 |
					 dmaSignalNormalized[index + 5]>>2;
		if( motorSpeedNewValue < 1026)
			motorSpeed = motorSpeedNewValue;


		break;
	}

}


void stopMotors()
{
//	HAL_Delay(20);
//	GPIOA->BRR = odOff[0];
//	GPIOA->BRR = odOff[1];
//	GPIOA->BRR = odOff[2];
//	EXTI->FTSR1 = 0;
//	EXTI->RTSR1 = 0;

//
//	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
//	TIM3->CCR2 = 0;
//	TIM3->CCR3 = 0;
//	TIM3->CCR4 = 0;
//	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);



}
void startMotors()
{
//	GPIOA->BRR = odOff[0];
//	GPIOA->BRR = odOff[1];
//	GPIOA->BRR = odOff[2];
//	EXTI->FTSR1 = 0;
//	EXTI->RTSR1 = 0;
//	TIM3->CCR2 = 0;
//	TIM3->CCR3 = 0;
//	TIM3->CCR4 = 0;
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//
//	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
//
//	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t*)dmaBuffer, dmaPulse);
//	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*)dmaBuffer, dmaPulse);
//	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t*)dmaBuffer, dmaPulse);

}
void setup()
{
	GPIOA->BRR = odOff[0];
	GPIOA->BRR = odOff[1];
	GPIOA->BRR = odOff[2];
	EXTI->FTSR1 = 0;
	EXTI->RTSR1 = 0;
	//HAL_Delay(1000);
	resetImrFlags = EXTI->IMR1;
	resetImrFlags &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin );
	EXTI->IMR1 = resetImrFlags;

//	HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dmaSignal, 16);



	if( HAL_TIM_Base_Start(&htim3) != HAL_OK)
		Error_Handler();
//	if( HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
//		Error_Handler();

	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t*)dmaBuffer, dmaPulse);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*)dmaBuffer, dmaPulse);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t*)dmaBuffer, dmaPulse);


//	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,150);
	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,50);
	HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1);
	HAL_Delay(100);

}


inline uint16_t map(int x, int in_min, int in_max, int out_min, int out_max)
{
	if (x < in_min)	x = in_min;

	if (x > in_max) x = in_max;

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

volatile double batteryVoltages = 0.0f;
volatile double adcValue = 0;
volatile uint16_t adcValues[4] = {0};
volatile uint16_t fallingPaddingValue = 0;
volatile double tempC =0;
volatile double tempF = 0;
volatile const uint8_t maxTempF = 180;

volatile uint8_t memorySettings[32] = {0};
volatile int sizeOfSettings = sizeof(memorySettings);
void readMemory(volatile unsigned char* data, int size, int location){
	location *= 8;

	for( int index = 0; index < size; index++){
		data[index] =  (unsigned int)(*(uint64_t*)(0x0801F800 + location));
		location += 8;
	}

}


void writeMemory(volatile unsigned char* data, int size, int location){
	uint8_t memorySettings[32] = {0};
	readMemory(memorySettings, sizeOfSettings, 0);

	size = size > sizeOfSettings ? sizeOfSettings : size;
	location = location >= sizeOfSettings ? sizeOfSettings : location;

	for( int index = 0; index < size; index++){
		location = location >= sizeOfSettings ? sizeOfSettings : location;
		memorySettings[location++] = data[index];
	}

	FLASH_EraseInitTypeDef epage;
	epage.TypeErase = FLASH_TYPEERASE_PAGES; // FLASH_TYPEERASE_PAGES
	epage.Page = 63;
	epage.NbPages = 1;


	uint32_t error = 0;
//	HAL_StatusTypeDef ret =
	HAL_FLASH_Unlock();

//	ret =
	HAL_FLASHEx_Erase(&epage, &error);
	for( int index = 0; index < sizeOfSettings; index++){
		//ret =
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F800 + (index * 8), memorySettings[index]);
	}

	//ret =
	HAL_FLASH_Unlock();

}

volatile uint16_t frequencyProbability[2] = {0,0};
volatile uint16_t signalValue[4] = {0};
void detechFrequency()
{

	signalValue[0] = divClosest(dmaSignal[1] - dmaSignal[0],8);
	signalValue[1] = divClosest(dmaSignal[3] - dmaSignal[2],8);
	if( signalValue[0] >= 100 && signalValue[0] < 200 && signalValue[1] > 0 && signalValue[1] < 5)
		frequencyProbability[0] += 1;

	if( signalValue[0] > 500 && signalValue[1] > 500)
		frequencyProbability[1] += 1;

}
void beepNoSPin(uint16_t ms ,uint16_t ps)
{
	uint32_t psc = TIM3->PSC;

		TIM3->PSC = ps;
		LED_GPIO_Port->BSRR = LED_Pin;
		setDutyCycle(10);
			HAL_Delay(ms);
		LED_GPIO_Port->BRR = LED_Pin;
		setDutyCycle(0);
		commutate();

		TIM3->PSC = psc;
		commutate();

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcValue = adcValues[0];
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcValue = adcValues[0];
}





void maincpp()
{




	setup();


	HAL_Delay(1000);

	for( int i= 0; i < 10; i++)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(100);

	}

	HAL_ADCEx_Calibration_Start(&hadc1);
	//HAL_ADC_Start(&hadc1);
	if( HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 2) != HAL_OK)
		Error_Handler();

	if( HAL_TIM_Base_Start(&htim7) != HAL_OK)
		Error_Handler();

	if( HAL_TIM_Base_Start(&htim6) != HAL_OK) // RPM
		Error_Handler();

	//setup();



	readMemory(memorySettings, sizeOfSettings, 0);
	if( memorySettings[0] != 1)
		memorySettings[0] = 0;

	masterDirection = memorySettings[0];
	direction = masterDirection;
	reverse = masterDirection;

	//frequencyType = 0;

	uint8_t testMotor = 0;

	//uint32_t RPMold = 0;
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	for( int i = 0; i < 7; i++)
		commutate();


	initComplete = 1;

	beepNoSPin(100);
	HAL_Delay(25);


	/*https://stackoverflow.com/questions/44985993/stm32-dma-memory-to-memory-transfer-only-fires-once
	 * enable DMA at start of main() on timer 1 to capture signal data frame
	 * of 32 bits, DShot is a 16 bit data frame and need to detect
	 * each pair of rising and falling edges.
	 */
	HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_3); // Stop DMA, force fresh DMA start
	TIM1->CNT = 0; // Reset counter to prevent overflow on first data sample.
	HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dmaSignal, 32);
//
	/*
	 * Initial launch of re-sync helper interrupts.
	 * keep both channels ready in case of desycn.
	 */
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
	TIM1->DIER &= ~(1<<1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	TIM1->DIER &= ~(1<<4);
//
//
//	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
//	TIM1->CCER &=  ~(1<<12);


	dmaSignal[0] = 55;
	HAL_DMA_Start(&hdma_memtomem_dma1_channel6, (uint32_t)dmaSignal, (uint32_t)dmaSignalESC, 32);

	uint8_t testCounter = 0;
//	uint16_t checkRising1 =  400;//map(RPM ,400,70000,400,1);
//	uint16_t checkFalling1 =  800;//map(RPM,400,70000,800,2);


	uint16_t commutationChange = 0;
	while(1)
	{
		beepCounter++;
		if (motorSpeedCurrent > 0)
			beepCounter = 0;
		if (beepCounter > 1000000 && motorSpeedCurrent == 0) {
			beepCounter = 0;
			//beepNoSPin();
			//HAL_Delay(25);
			//beepNoSPin(260);

		}
//		TIM3->ARR = map(RPM,0,2000,3072-1,512-1);

		if( commutationCNT > 1 && motorSpeedCurrent > 0)
		{
			if( newDmaSignal == 0)
			{
				eRPM = (60000000 / commutationCNT);// * 60 ; // prescale 64-1
				RPM = eRPM / 7;

				if( RPM > RPMOld)
				{
					if( (RPM - RPMOld) > 5000)
						motorSpeedCurrent = 0;
				}
				else
				{
					if( (RPMOld - RPM ) > 5000)
						motorSpeedCurrent = 0;

				}
				RPMOld = RPM;
				if( RPM > maxRPM)
					maxRPM = RPM;
			}
			else
				processDmaSignalDShot();
		}
		else if( motorSpeedCurrent == 0)
		{
			commutationCNT = 0;
			eRPM = 0;
			RPM = 0;
			maxRPM = 0;
		}
		else
		{

			eRPM = 0;// (60000000 / 1);// * 60 ;
			RPM = 0;//eRPM / 7;
		}


		if((REV_GPIO_Port->IDR & REV_Pin) == 0)
		{// switch direction when button pressed.

			LED_GPIO_Port->BSRR = LED_Pin;
			HAL_Delay(500);
			LED_GPIO_Port->BRR = LED_Pin;

			if( masterDirection == 1)
				direction = 0;
			else
				direction = 1;


			memorySettings[0] = direction;
			masterDirection = direction;
			writeMemory(memorySettings, 1, 0);
			readMemory(memorySettings, sizeOfSettings, 0);
			if(testMotor )
			{
				motorSpeed = 0;
				motorSpeedCurrent = 0;

				setDutyCycle(0);
			}
			testCounter = 0;
			testMotor = 0;

			while((REV_GPIO_Port->IDR & REV_Pin) == 0)
			{

				if( testCounter++ > 30)
				{

					testMotor = 1;
					while((REV_GPIO_Port->IDR & REV_Pin) == 0)
					{
						HAL_Delay(50);
						HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					}
				}

				HAL_Delay(50);
			}
			LED_GPIO_Port->BRR = LED_Pin;
		}

		if( reverse == 1)
			LED_GPIO_Port->BSRR = LED_Pin;
		else
			LED_GPIO_Port->BRR = LED_Pin;



		if( newDmaSignal || testMotor )
		{
			if( !testMotor)
			{

				processDmaSignalDShot();
			}
			else
			{
				motorSpeed += motorSpeedDirection;
				HAL_Delay(30);

				if( motorSpeed > 300 || motorSpeed <= 0)
				{

					motorSpeedDirection *= -1;
					if( motorSpeedDirection == 1)
					{
						if( direction == 1)
							direction = 0;
						else
							direction = 1;
					}
				}

			}

			if( direction != reverse )
			{
				motorSpeedCurrent = 0;

				setDutyCycle(motorSpeedCurrent);
				reverse = direction;
				//motorSpinning = 0;
				//HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
				HAL_Delay(100);
			}

			adcValue = adcValues[0];// HAL_ADC_GetValue(&hadc1);

			if( newDmaSignal == 0)
			{
				tempC = (0.000806*adcValues[1]-.76)/.0025+25;
				tempF = (tempC *1.8) + 32;

			}
			else
				processDmaSignalDShot();

			if( adcValue > 0)
			{
				//											rb/rt
				if( newDmaSignal == 0)
				{
					//batteryVoltages = adcValue*(3.3 / 4095.0)/(10000.0/110000.0);
					batteryVoltages = adcValue * 0.008907975;
				}
				else
					processDmaSignalDShot();
				if( newDmaSignal == 0)
					fallingPaddingValue = map(batteryVoltages,8,25,1,400);
				else
					processDmaSignalDShot();
			}

			if( motorSpeed != motorSpeedCurrent || tempF > maxTempF)
			{
				if( tempF > maxTempF )
				{
					if( motorSpeedCurrent > 250)
						motorSpeedCurrent -= 2;

				}
				else
				{
					if(motorSpeedCurrent == 0)
					{
						commutate();
					}
					if( motorSpeedCurrent < 2000 )
					{
						if ( motorSpeed > motorSpeedCurrent)
						{
							motorSpeedCurrent++;

						}
						else
						{
							motorSpeedCurrent = motorSpeed;
						}
					}else
					{
						if ((motorSpeed > motorSpeedCurrent))
						{
							motorSpeedCurrent += 1;
						}
						else
						{
							motorSpeedCurrent = motorSpeed;
						}

					}

				}

				setDutyCycle(motorSpeedCurrent);

//				if (motorSpeedCurrent == 0 && motorSpinning) { // allows to remove motor spinning check on interrupt, prevents volt spikes on cutoff
//					motorSpinning = 0;
//					stopMotors();
////					HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
//				} else if (motorSpeedCurrent > 0 && !motorSpinning) {
//					motorSpinning = 1;
//					startMotors();
//					//for( int i = 0; i < 7; i++)
//						//	commutate();
////					HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
////					HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
//
//				}
//

				if( newDmaSignal == 0)
				{
					checkRising =  map(RPM ,400,70000,400,1);
					checkFalling =  map(RPM,400,70000,800,1);
				}
				else
					processDmaSignalDShot();

			}


		}

	}
}


