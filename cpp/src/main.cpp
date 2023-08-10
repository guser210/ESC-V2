/*
 * main.cpp
 * ESC_V1.4_TEST_Variable
 *  Created on: July 18, 2022
 *      Author: gvargas
 */

#include "main.h"

extern "C" {
extern TIM_HandleTypeDef htim1;		// Signal input capture.
extern TIM_HandleTypeDef htim3;		// PWM driver.
extern TIM_HandleTypeDef htim7;		// Generic timer.

extern TIM_HandleTypeDef htim6;		// RPM.
extern TIM_HandleTypeDef htim14;		// RPM.
extern TIM_HandleTypeDef htim15;		// Low Bat warning.

extern TIM_HandleTypeDef htim16;	// commutator delay.
extern TIM_HandleTypeDef htim17;	// commutator bump timer.
extern ADC_HandleTypeDef hadc1;	// battery voltages.
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel6;
extern DMA_HandleTypeDef hdma_tim1_ch3;


extern volatile uint8_t initComplete;

}

volatile uint8_t disableBump;

volatile uint32_t eRPM = 0;
volatile uint32_t RPM = 0;
volatile uint32_t RPMOld = 0;
volatile uint32_t maxRPM = 0;

volatile uint8_t newDmaSignal = 0;
volatile uint16_t dmaSignal[32] = { 0 };
volatile uint16_t dmaSignalESC[32] = { 0 };
volatile uint16_t dmaSignalMem2Mem[32] = { 0 };
volatile int16_t dmaSignalDiff[33] = { 0 };
volatile uint16_t dmaSignalNormalized[32] = { 0 };
volatile uint16_t dmaESCSignal[33] = { 0 };
volatile uint16_t motorSpeed = 0;
volatile uint16_t motorSpeedCurrent = 0;
volatile uint16_t motorSpeedCurrentOld = 0;
//void beepNoSPin(uint16_t ms ,uint16_t ps );
void beepNoSPin(uint16_t ms = 40, uint16_t ps = 24);
inline uint16_t map(int x, int in_min, int in_max, int out_min, int out_max);
//////////////////////////////////////////////////////////////////////
const uint16_t dmaPulse = 1;
const uint16_t dmaPulseReload = 1024;	//2048;//1024;//512;
//const uint16_t dmaPulse = 2;
//const uint16_t dmaPulseReload = 512;
volatile uint16_t dmaBuffer[dmaPulse] = { 0 };
volatile uint16_t dmaBufferA[dmaPulse] = { 0 };
volatile uint16_t dmaBufferB[dmaPulse] = { 0 };
volatile uint16_t dmaBufferC[dmaPulse] = { 0 };
volatile uint16_t resolution = dmaPulse * dmaPulseReload;

inline void startMotors();
inline void stopMotors();

const uint32_t zcOff[6] = {	//
		ZC_B_Pin, // B
		ZC_A_Pin, // A
		ZC_C_Pin, // C
		ZC_B_Pin, // B
		ZC_A_Pin, // A
		ZC_C_Pin, // C
	};

const uint32_t odLow[6] = { //
		OD_C_Pin, // C
		OD_C_Pin, // C
		OD_A_Pin, // A
		OD_A_Pin, // A
		OD_B_Pin, // B
		OD_B_Pin, // B
	};
const uint32_t odHigh[6] = { //
		OD_A_Pin, // A
		OD_B_Pin, // B
		OD_B_Pin, // B
		OD_C_Pin, // C
		OD_C_Pin, // C
		OD_A_Pin, // A
	};

const uint32_t odOff[6] = { //
		OD_B_Pin, // B
		OD_A_Pin, // A
		OD_C_Pin, // C
		OD_B_Pin, // B
		OD_A_Pin, // A
		OD_C_Pin, // C
	};
//const DMA_HandleTypeDef dmaOff[6] = { (DMA_HandleTypeDef) hdma_tim3_ch3, // B
//		(DMA_HandleTypeDef) hdma_tim3_ch2, // A
//		(DMA_HandleTypeDef) hdma_tim3_ch4, // C
//		(DMA_HandleTypeDef) hdma_tim3_ch3, // B
//		(DMA_HandleTypeDef) hdma_tim3_ch2, // A
//		(DMA_HandleTypeDef) hdma_tim3_ch4, // C
//		};

const uint32_t diOff[6] = { // TIM3->DIER for PWM output RM0454 page .
		1 << 12, // B
		1 <<  9, // A
		1 << 10, // C
		1 << 12, // B
		1 <<  9, // A
		1 << 10, // C
		};
const uint32_t ccOff[6] = { // TIME3->CCER For pwm output, RM0454 page 495.
		1 << 12, // B
		1 <<  0, // A
		1 <<  4, // C
		1 << 12, // B
		1 <<  0, // A
		1 <<  4, // C
		};
const uint32_t ccOn[6] = { // TIME3->CCER For pwm output, RM0454 page 495.
		1 <<  0, // A
		1 << 12, // B
		1 << 12, // B
		1 <<  4, // C
		1 <<  4, // C
		1 <<  0, // A
		};

const uint32_t diOn[6] = { // TIM3->DIER for PWM output RM0454 page .
		1 <<  9, // A
		1 << 12, // B
		1 << 12, // B
		1 << 10, // C
		1 << 10, // C
		1 <<  9, // A
		};
GPIO_TypeDef *odrOff[6] = { //
		(GPIO_TypeDef*) OD_B_GPIO_Port, // B
		(GPIO_TypeDef*) OD_A_GPIO_Port, // A
		(GPIO_TypeDef*) OD_C_GPIO_Port, // C
		(GPIO_TypeDef*) OD_B_GPIO_Port, // B
		(GPIO_TypeDef*) OD_A_GPIO_Port, // A
		(GPIO_TypeDef*) OD_C_GPIO_Port, // C
	};
GPIO_TypeDef *odrLow[6] = { //
		(GPIO_TypeDef*) OD_C_GPIO_Port, // C
		(GPIO_TypeDef*) OD_C_GPIO_Port, // C
		(GPIO_TypeDef*) OD_A_GPIO_Port, // A
		(GPIO_TypeDef*) OD_A_GPIO_Port, // A
		(GPIO_TypeDef*) OD_B_GPIO_Port, // B
		(GPIO_TypeDef*) OD_B_GPIO_Port, // B
	};
GPIO_TypeDef *odrHigh[6] = { //
		(GPIO_TypeDef*) OD_A_GPIO_Port, // A
		(GPIO_TypeDef*) OD_B_GPIO_Port, // B
		(GPIO_TypeDef*) OD_B_GPIO_Port, // B
		(GPIO_TypeDef*) OD_C_GPIO_Port, // C
		(GPIO_TypeDef*) OD_C_GPIO_Port, // C
		(GPIO_TypeDef*) OD_A_GPIO_Port, // A
	};
__IO uint32_t *ccrHigh[6] = { //
		&TIM3->CCR1, // A
		&TIM3->CCR4, // B
		&TIM3->CCR4, // B
		&TIM3->CCR2, // C
		&TIM3->CCR2, // C
		&TIM3->CCR1, // A
};
__IO uint32_t *ccrOff[6] = { //
		&TIM3->CCR4, // B
		&TIM3->CCR1, // A
		&TIM3->CCR2, // C
		&TIM3->CCR4, // B
		&TIM3->CCR1, // A
		&TIM3->CCR2, // C
};
__IO uint32_t *ccrLow[6] = { //
		&TIM3->CCR2, // C
		&TIM3->CCR2, // C
		&TIM3->CCR1, // A
		&TIM3->CCR1, // A
		&TIM3->CCR4, // B
		&TIM3->CCR4, // B
};

const GPIO_TypeDef *zcPortOff[6] = { //
		(GPIO_TypeDef*) ZC_B_GPIO_Port, // B
				(GPIO_TypeDef*) ZC_A_GPIO_Port, // A
				(GPIO_TypeDef*) ZC_C_GPIO_Port, // C
				(GPIO_TypeDef*) ZC_B_GPIO_Port, // B
				(GPIO_TypeDef*) ZC_A_GPIO_Port, // A
				(GPIO_TypeDef*) ZC_C_GPIO_Port, // C
		};

volatile uint32_t resetImrFlags = 0;
volatile uint8_t powerStep = 0;
volatile uint8_t powerStepCurrent = 0;
volatile uint8_t powerStepNext = 0;

const uint8_t rising[2][6] = { { 1, 0, 1, 0, 1, 0 }, { 0, 1, 0, 1, 0, 1 } };

volatile uint8_t reverse = 0;

#define zcPinOn ZC_A_Pin | ZC_B_Pin | ZC_C_Pin
volatile uint16_t commutationCNT = 0;
volatile uint16_t commutationCNT14 = 0;
volatile uint16_t commutationCntPrev = 0;
volatile uint16_t commutation6Step = 0;
volatile uint8_t motorSpinning = 0;


void setDutyCycle(uint16_t dc) {
	//return;
	if (dc > resolution)
		dc = resolution;

	if (dc > resolution)
		dc = resolution;

	dmaBuffer[0] = dc;

	__disable_irq();
	*ccrHigh[powerStep] = dc;
	__enable_irq();


}


volatile uint32_t fallingCount = 0;
volatile uint32_t risingCount = 0;
volatile uint32_t commutatingCount = 0;

inline void commutate() {
	// go to next power step.
	commutatingCount++;
	powerStep = powerStepNext;

	static uint16_t odr = 0;

	/*
	 Step 0  1   2  3  4  5      0  1  2  3  4  5
	 High A  B   B  C  C  A      A  B  B  C  C  A
	 Off  B  A   C  B  A  C      B  A  C  B  A  C
	 Low  C  C   A  A  B  B      C  C  A  A  B  B
	 */
//		reverse = 0;

	odrOff[powerStep]->BRR = odOff[powerStep];
	*ccrHigh[powerStep] = dmaBuffer[0];
	*ccrOff[powerStep] = 0;
	odrLow[powerStep]->BSRR = odLow[powerStep];
	odrHigh[powerStep]->BSRR = odHigh[powerStep];

	if (rising[reverse][powerStep]) {

		EXTI->FTSR1 = 0;
		EXTI->RTSR1 = zcOff[powerStep];

	} else {
		EXTI->RTSR1 = 0;
		EXTI->FTSR1 = zcOff[powerStep];

	}

	TIM16->CNT = 0;
	powerStepCurrent++;
	powerStepCurrent %= 6;

//	if( motorSpeedCurrent == 0)
//		motorSpinning = 0;
	if ( initComplete == 0) //|| motorSpeedCurrent == 0)
	{
		HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	}

	if (reverse)
		powerStepNext = 5 - powerStepCurrent;
	else
		powerStepNext = powerStepCurrent; // forward.

	if (powerStepNext == 0) {
		commutationCNT = TIM6->CNT;
		TIM6->CNT = 0;
	} else if (powerStepNext == 3) {
		commutationCNT14 = TIM14->CNT;
		TIM14->CNT = 0;
	}

	// enable wakeup with interrupt.
	EXTI->IMR1 = resetImrFlags | zcOff[powerStep];
	// clear any zc pending interrupts.
	EXTI->FPR1 = zcPinOn;
	;
	EXTI->RPR1 = zcPinOn;
	;
	// reset timer17 for bump motor.
	TIM17->CNT = 0;

}

volatile uint32_t newARR = 2047; //3076;
volatile uint16_t checkRising = 0;
volatile uint16_t checkFalling = 0;


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

	if( TIM16->CNT > 300)
	{
		TIM16->CNT = 0;
		EXTI->IMR1 = resetImrFlags ;
		risingCount++;
		motorSpeedCurrent >>=1;
		return;
	}
	TIM7->CNT = 0;
	while ( TIM7->CNT < checkRising)
		if ((zcPortOff[powerStep]->IDR & zcOff[powerStep]) == 0)
			return;


	commutate();
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {


	if( TIM16->CNT > 300)
	{
		TIM16->CNT = 0;
		EXTI->IMR1 = resetImrFlags ;
		fallingCount++;
		motorSpeedCurrent >>=1;
		return;
	}

	TIM7->CNT = 0;
	while ( TIM7->CNT < checkFalling)
		if ((zcPortOff[powerStep]->IDR & zcOff[powerStep]))
			return;

	commutate();
}

volatile uint8_t lowBattery = 0;

//////////////////////////////////////////////////////////////////////

uint16_t divClosest(uint16_t a, uint16_t b) {
	return (a + b / 2) / b;
}

volatile uint16_t pwm[3] = { 0 };

volatile uint16_t pwmRising[5] = { 0 };
volatile uint16_t pwmFalling[5] = { 0 };
volatile uint8_t pwmAvgCounter = 0;

volatile uint8_t transmitterDirection = 0;
volatile uint8_t masterDirection = 0;
volatile uint16_t pwm1 = 0;
volatile uint16_t pwm2 = 0;

volatile uint8_t directionValidator = 0;

volatile uint32_t numberOfTimesDshotProcessed = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotMiddleOfFrame = 0; // Debug variable.
volatile uint32_t numberOfTimesDshotCRCError = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotThrottleOver = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotTimeAdvace = 0; // Debug Variable
volatile uint32_t numberOfTimesDshotResync = 0; // Debug Variable

volatile uint16_t debugCounter = 0;

volatile uint16_t dmaSignalNormalized2[16] = { 0 };
static volatile uint8_t errorReads = 0;

static volatile uint16_t throttle = 0;
static volatile uint16_t dshotCommand = 0;
static volatile int16_t throttlePlusTel = 0;
static volatile uint16_t crcIn = 0;
static volatile uint16_t crc = 0;
static volatile uint16_t telemetryBit = 0;
static volatile uint16_t signal[16] = { 0 };

volatile uint16_t bumps = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim15) //&& htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
		if (lowBattery || motorSpeedCurrent > 0) {
			LED_GPIO_Port->BRR = LED_Pin;
		}
	} else if (htim == &htim17) {

	}

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) { // guser210

	if (htim == &htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		TIM1->DIER &= ~((1 << 1) | (1 << 4)); // Disable interrupts for channel 1 and 4.

		// Enable Signal DMA.
		TIM1->CNT = 0;
		hdma_tim1_ch3.Instance->CNDTR = 32; // First set number of bytes to transfer
		hdma_tim1_ch3.Instance->CCR |= 1; // second enable DMA channel.

		numberOfTimesDshotResync++;

	} else if (htim == &htim17 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

		TIM17->CNT = 0;
		if (initComplete > 0 && motorSpeedCurrent > 0) { // Bump
			LED_GPIO_Port->ODR ^= LED_Pin;
			bumps++;
			commutate();
		} else if (motorSpeedCurrent == 0)

		{

		}
	} else if (htim == &htim15 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (lowBattery) {
			LED_GPIO_Port->BSRR = LED_Pin;
		}
	}

}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (newDmaSignal == 0) {
				//MEM2MEM launch.
				hdma_memtomem_dma1_channel6.Instance->CNDTR = 32;
				hdma_memtomem_dma1_channel6.Instance->CCR |= 1;
				newDmaSignal = 1;
			}

			// reset CNT  to prevent early overflow.
			TIM1->CNT = 0;
		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
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
volatile uint8_t DShotCommandList[128] = { 0 };

// TODO:Step 3

inline void processDmaSignalDShot() {

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

	while (hdma_memtomem_dma1_channel6.Instance->CNDTR > 0)
		;
	// Disble mem2mem channel... this is required for re-activation.
	hdma_memtomem_dma1_channel6.Instance->CCR &= ~(1);

	//check if middle of frame.
	if (dmaSignalESC[0] > dmaSignalESC[31]) // || dmaSignalESC[15] > dmaSignalESC[31])
			{
		TIM1->CNT = 0; // reset counter to 0 to prevent overflows.
		TIM1->CCR1 = 50;
		hdma_tim1_ch3.Instance->CCR &= ~(1); // Disable Signal DMA
		TIM1->DIER |= (1 << 1) | (1 << 4); // enable interrupts for Channels 1 and 4.

		numberOfTimesDshotMiddleOfFrame++;
		newDmaSignal = 0;
		return; // invalid data exit function.
	}

	static uint8_t ShiftBy = 5;

	signal[0] = (dmaSignalESC[1] - dmaSignalESC[0]) >> ShiftBy;
	signal[1] = (dmaSignalESC[3] - dmaSignalESC[2]) >> ShiftBy;
	signal[2] = (dmaSignalESC[5] - dmaSignalESC[4]) >> ShiftBy;
	signal[3] = (dmaSignalESC[7] - dmaSignalESC[6]) >> ShiftBy;
	signal[4] = (dmaSignalESC[9] - dmaSignalESC[8]) >> ShiftBy;
	signal[5] = (dmaSignalESC[11] - dmaSignalESC[10]) >> ShiftBy;
	signal[6] = (dmaSignalESC[13] - dmaSignalESC[12]) >> ShiftBy;
	signal[7] = (dmaSignalESC[15] - dmaSignalESC[14]) >> ShiftBy;
	signal[8] = (dmaSignalESC[17] - dmaSignalESC[16]) >> ShiftBy;
	signal[9] = (dmaSignalESC[19] - dmaSignalESC[18]) >> ShiftBy;
	signal[10] = (dmaSignalESC[21] - dmaSignalESC[20]) >> ShiftBy;
	signal[11] = (dmaSignalESC[23] - dmaSignalESC[22]) >> ShiftBy;
	signal[12] = (dmaSignalESC[25] - dmaSignalESC[24]) >> ShiftBy;
	signal[13] = (dmaSignalESC[27] - dmaSignalESC[26]) >> ShiftBy;
	signal[14] = (dmaSignalESC[29] - dmaSignalESC[28]) >> ShiftBy;
	signal[15] = (dmaSignalESC[31] - dmaSignalESC[30]) >> ShiftBy;

	newDmaSignal = 0;

	throttlePlusTel = (signal[0]) << 11 | (signal[1]) << 10 | (signal[2]) << 9
			| (signal[3]) << 8 | (signal[4]) << 7 | (signal[5]) << 6
			| (signal[6]) << 5 | (signal[7]) << 4 | (signal[8]) << 3
			| (signal[9]) << 2 | (signal[10]) << 1 | (signal[11]);

	throttle = throttlePlusTel >> 2; // 1 = 2048, 2 = 1024;
	dshotCommand = throttlePlusTel >> 1;

	crcIn = signal[12] << 3 | signal[13] << 2 | signal[14] << 1 | signal[15];

	// Calculate CRC from 12 MSB.
	crc = (throttlePlusTel ^ (throttlePlusTel >> 4) ^ (throttlePlusTel >> 8))
			& 0x0F;

	if (crc != crcIn || throttle > 1024) {
		if (crc != crcIn)
			numberOfTimesDshotCRCError++; // Increment debug counter.
		if (throttle > 1024)
//		if (throttle > 2048)
			numberOfTimesDshotThrottleOver++; // increment debug counter.
		if (errorReads++ > 10) {

			if (motorSpeed > 1)
				motorSpeed--;
			errorReads = 0;
		}

		TIM1->CNT = 0; // reset counter to 0 to prevent overflows.
		TIM1->CCR1 = 120;
		hdma_tim1_ch3.Instance->CCR &= ~(1); // Disable Signal DMA
		TIM1->DIER |= (1 << 1) | (1 << 4); // enable interrupts for Channels 1 and 4.

		newDmaSignal = 0;
		return; // invalid data exit function.
	}

	telemetryBit = signal[11];

	if (dshotCommand > 0 && dshotCommand < 48 && motorSpeed == 0) {
		DShotCommandList[DShotCommandCount++] = dshotCommand;
		DShotCommandCount %= 128;

		switch (dshotCommand) {
		case 21: // Motor direction: Reverse
			if (masterDirection == direction) {
				//if( directionCounter++ > 3)
				{
//					directionCounter = 0;
					if (direction == 1)
						direction = 0;
					else
						direction = 1;
				}
			}

			break;
		case 20: // Motor direction: Normal
			direction = masterDirection;
			break;
		case 1:
		case 2: // beep 1 and 2
		case 3:
		case 4: // beep 3 and 4
		case 5: // beep 5
			break;
		default:
			break;
		}
		return;

	}

	errorReads = 0;
	newDmaSignal = 0;
	motorSpeed = throttle > 0 ? throttle  : 0; // throttle;


}

void processDmaSignal() {
	newDmaSignal = 0;

	for (int index = 0; index < 15; index++)
		dmaSignalNormalized[index] = divClosest(
				dmaSignal[index + 1] - dmaSignal[index], 8) - 8;

//
	if (dmaSignalNormalized[0] > 1000 && dmaSignalNormalized[1] > 1000
			&& dmaSignalNormalized[2] > 1000) {
		return;
	}

	for (int index = 0; index <= 10; index++) {
		if (dmaSignalNormalized[index] < 100)
			continue;
		if (dmaSignalNormalized[index + 1] > 3000)
			continue;

		uint16_t motorSpeedNewValue = dmaSignalNormalized[index + 1] << 6
				| dmaSignalNormalized[index + 3] << 2
				| dmaSignalNormalized[index + 5] >> 2;
		if (motorSpeedNewValue < 1026)
			motorSpeed = motorSpeedNewValue;

		break;
	}

}

inline uint16_t map(int x, int in_min, int in_max, int out_min, int out_max) {
	if (x < in_min)
		x = in_min;

	if (x > in_max)
		x = in_max;

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const double batteryCellLow = 3.5;
const double batteryCellHigh = 4.29;
const double BatteryLow[7] = { 0, batteryCellLow * 1, batteryCellLow * 2,
		batteryCellLow * 3, batteryCellLow * 4, batteryCellLow * 5,
		batteryCellLow * 6 };

const double BatteryHigh[7] = { 0, batteryCellHigh * 1, batteryCellHigh * 2,
		batteryCellHigh * 3, batteryCellHigh * 4, batteryCellHigh * 5,
		batteryCellHigh * 6 };

volatile double batteryVoltages = 0.0f;
volatile double adcValue = 0;
volatile uint16_t adcValues[4] = { 0 };
volatile uint16_t fallingPaddingValue = 0;
volatile double tempC = 0;
volatile double tempF = 0;
volatile const uint8_t maxTempF = 190;

volatile uint8_t memorySettings[32] = { 0 };
volatile int sizeOfSettings = sizeof(memorySettings);
void readMemory(volatile unsigned char *data, int size, int location) {
	location *= 8;

	for (int index = 0; index < size; index++) {
		data[index] = (unsigned int) (*(uint64_t*) (0x0801F800 + location));
		location += 8;
	}

}

void writeMemory(volatile unsigned char *data, int size, int location) {
	uint8_t memorySettings[32] = { 0 };
	readMemory(memorySettings, sizeOfSettings, 0);

	size = size > sizeOfSettings ? sizeOfSettings : size;
	location = location >= sizeOfSettings ? sizeOfSettings : location;

	for (int index = 0; index < size; index++) {
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
	for (int index = 0; index < sizeOfSettings; index++) {
		//ret =
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
				0x0801F800 + (index * 8), memorySettings[index]);
	}

	//ret =
	HAL_FLASH_Unlock();

}

void beepNoSPin(uint16_t ms, uint16_t ps) {
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcValue = adcValues[0];
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	adcValue = adcValues[0];
}

volatile uint32_t timTest = 0;
volatile uint32_t timTest2 = 0;
volatile uint8_t throttleChange = 0;

void setup() {

	OD_A_GPIO_Port->BRR = OD_A_Pin;
	OD_B_GPIO_Port->BRR = OD_B_Pin;
	OD_C_GPIO_Port->BRR = OD_C_Pin;
	EXTI->FTSR1 = 0;
	EXTI->RTSR1 = 0;

	resetImrFlags = EXTI->IMR1;
	resetImrFlags &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);
	EXTI->IMR1 = resetImrFlags;


	if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
		Error_Handler();

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
//	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 20);
	HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1);


	HAL_TIM_Base_Start_IT(&htim15);
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 450);
	HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_1);
	TIM15->CNT = 0;
}

void maincpp() {

	setup();

	for (int i = 0; i < 10; i++) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(10);

	}

	if( HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
		Error_Handler();

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcValues, 2) != HAL_OK)
		Error_Handler();

	if (HAL_TIM_Base_Start(&htim7) != HAL_OK)
		Error_Handler();

	if (HAL_TIM_Base_Start(&htim6) != HAL_OK) // RPM
		Error_Handler();
	if (HAL_TIM_Base_Start(&htim14) != HAL_OK) // RPM
		Error_Handler();

	if (HAL_TIM_Base_Start(&htim16) != HAL_OK)
		Error_Handler();
	//setup();

	readMemory(memorySettings, sizeOfSettings, 0);
	if (memorySettings[0] != 1)
		memorySettings[0] = 0;

	masterDirection = memorySettings[0];
	direction = masterDirection;
	reverse = masterDirection;

	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	for (int i = 0; i < 7; i++)
		commutate();

	initComplete = 1;
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);


	HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_3); // Stop DMA, force fresh DMA start
	TIM1->CNT = 0; // Reset counter to prevent overflow on first data sample.
	HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*) dmaSignal, 32);
//
	/*
	 * Initial launch of re-sync helper interrupts.
	 * keep both channels ready in case of desycn.
	 */
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
	TIM1->DIER &= ~(1 << 1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	TIM1->DIER &= ~(1 << 4);


	dmaSignal[0] = 55;
	HAL_DMA_Start(&hdma_memtomem_dma1_channel6, (uint32_t) dmaSignal,
			(uint32_t) dmaSignalESC, 32);


	while (1) {
		reverse = direction;
		beepCounter++;
		if (motorSpeedCurrent > 0)
			beepCounter = 0;
		if (beepCounter > 1000000 && motorSpeedCurrent == 0) {
			beepCounter = 0;
		}


		if (commutationCNT > 1 && motorSpeedCurrent > 0) {
			if (newDmaSignal == 1)
				processDmaSignalDShot();
			checkRising = map(RPM, 400, 70000, 400, 10);

			checkFalling = map(RPM, 400, 70000, 800, 10);


			if (1 == 1)			// newDmaSignal == 0)
					{

				if (newDmaSignal == 1)
					processDmaSignalDShot();

				if (powerStepNext < 3) {

					if( commutationCNT > 0)
					{
						eRPM = (60000000 / commutationCNT);	// * 60 ; // prescale 64-1
						RPM = eRPM / 7;
					}

				} else			// if( powerStepNext == 3)
				{ // 250 clock cycles
					if( commutationCNT14 > 0)
					{
						eRPM = (60000000 / commutationCNT14); // * 60 ; // prescale 64-1
						RPM = eRPM / 7;
					}
				}
				if (newDmaSignal == 1)
					processDmaSignalDShot();

				if (RPM > RPMOld) {
					if ((RPM - RPMOld) > 10000) //&& motorSpeedCurrentOld > motorSpeedCurrent)
						motorSpeedCurrent = 0;
				} else {
					if ((RPMOld - RPM) > 10000) //&& motorSpeedCurrentOld > motorSpeedCurrent)
						motorSpeedCurrent >>= 1;

				}
				RPMOld = RPM;
				motorSpeedCurrentOld = motorSpeedCurrent;
				if (RPM > maxRPM)
					maxRPM = RPM;
			}

		} else if (motorSpeedCurrent == 0) {
			commutationCNT = 0;
			eRPM = 0;
			RPM = 0;
			maxRPM = 0;
		} else {

			eRPM = 0; // (60000000 / 1);// * 60 ;
			RPM = 0; //eRPM / 7;
		}

		// Change PWM based on RPM

		if( RPM > 0 &&  RPM < 15000 && TIM3->PSC != 3)
		{
			TIM3->PSC = 3;
		}
		else if( RPM > 17000 && RPM < 30000 && TIM3->PSC != 2)
		{
			TIM3->PSC = 2;
		}
		else if( RPM > 32000 && RPM < 480000 && TIM3->PSC != 1)
		{
			TIM3->PSC = 1;
		}
		else if( RPM > 52000 && TIM3->PSC != 0)
		{
			TIM3->PSC = 0;
		}


		if (motorSpeed == 0 && motorSpeedCurrent == 0 && (REV_GPIO_Port->IDR & REV_Pin) == 0) { // switch direction when button pressed.

			LED_GPIO_Port->BSRR = LED_Pin;
			HAL_Delay(100);
			LED_GPIO_Port->BRR = LED_Pin;

			if (masterDirection == 1)
				direction = 0;
			else
				direction = 1;

			memorySettings[0] = direction;
			masterDirection = direction;
			writeMemory(memorySettings, 1, 0);
			readMemory(memorySettings, sizeOfSettings, 0);

			while ((REV_GPIO_Port->IDR & REV_Pin) == 0) {

				HAL_Delay(50);
			}
			LED_GPIO_Port->BRR = LED_Pin;
		}

		if (motorSpeedCurrent == 0 && !lowBattery) {
			if (reverse == 1)
				LED_GPIO_Port->BSRR = LED_Pin;
			else
				LED_GPIO_Port->BRR = LED_Pin;
		}

		adcValue = adcValues[0]; // HAL_ADC_GetValue(&hadc1);

		if (newDmaSignal == 1)
			processDmaSignalDShot();

		tempC = (0.000806 * adcValues[1] - .76) / .0025 + 25;
		tempF = (tempC * 1.8) + 32;

		if (newDmaSignal == 1)
			processDmaSignalDShot();

		if (adcValue > 0) {

			batteryVoltages = adcValue * 0.008907975;
			if (batteryVoltages < BatteryLow[1]
					&& batteryVoltages > BatteryHigh[0]) {
				lowBattery = 1;

			} else if (batteryVoltages < BatteryLow[2]
					&& batteryVoltages > BatteryHigh[1]) {
				lowBattery = 1;

			} else if (batteryVoltages < BatteryLow[3]
					&& batteryVoltages > BatteryHigh[2]) {
				lowBattery = 1;

			} else if (batteryVoltages < BatteryLow[4]
					&& batteryVoltages > BatteryHigh[3]) {
				lowBattery = 1;
			} else if (batteryVoltages < BatteryLow[5]
					&& batteryVoltages > BatteryHigh[4]) {
				lowBattery = 1;
			} else if (batteryVoltages < BatteryLow[6]
					&& batteryVoltages > BatteryHigh[5]) {
				lowBattery = 1;
			} else {
				lowBattery = 0;
			}

			if (newDmaSignal == 1)
				processDmaSignalDShot();

		}

		if( motorSpeed != motorSpeedCurrent)
		{
				if ( motorSpeedCurrent < 60)// && motorSpeed > 70)
				{
					if (motorSpeed > motorSpeedCurrent) {
						if ((motorSpeed - motorSpeedCurrent) > 1)
							motorSpeedCurrent += 1;
						else
							motorSpeedCurrent = motorSpeed;


					} else {
						if ((motorSpeedCurrent - motorSpeed) > 1)
							motorSpeedCurrent -= 1;
						else
							motorSpeedCurrent = motorSpeed;

					}

					if( motorSpeedCurrent > 1024)
						motorSpeedCurrent = 1024;

					setDutyCycle(motorSpeedCurrent);
					if (newDmaSignal == 1)
						processDmaSignalDShot();

				}else
				{
					if (motorSpeed > motorSpeedCurrent) {
						if ((motorSpeed - motorSpeedCurrent) >= 10)
							motorSpeedCurrent += 10;
						else
							motorSpeedCurrent = motorSpeed;


					} else {
						if ((motorSpeedCurrent - motorSpeed) >= 10)
							motorSpeedCurrent -= 10;
						else
							motorSpeedCurrent = motorSpeed;

					}

					if( motorSpeedCurrent > 1024)
						motorSpeedCurrent = 1024;

					setDutyCycle(motorSpeedCurrent);
					if (newDmaSignal == 1)
						processDmaSignalDShot();

				}


		}

	}
}

