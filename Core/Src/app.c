#include "app.h"
#include "main.h"
#include <stdbool.h>

#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include "arm_math.h"

#define MY_SIGNAL_SIZE 90

q15_t mySignal[] = {
		0x4000,
		0x446B,
		0x48D1,
		0x4D2C,
		0x5178,
		0x55AD,
		0x59C9,
		0x5DC5,
		0x619C,
		0x654A,
		0x68CB,
		0x6C1A,
		0x6F33,
		0x7212,
		0x74B4,
		0x7716,
		0x7934,
		0x7B0D,
		0x7C9D,
		0x7DE3,
		0x7EDD,
		0x7F8B,
		0x7FEB,
		0x7FFD,
		0x7FC0,
		0x7F36,
		0x7E5E,
		0x7D3A,
		0x7BCC,
		0x7A14,
		0x7815,
		0x75D2,
		0x734C,
		0x7089,
		0x6D89,
		0x6A52,
		0x66E8,
		0x634E,
		0x5F88,
		0x5B9C,
		0x578F,
		0x5364,
		0x4F22,
		0x4ACE,
		0x466C,
		0x4202,
		0x3D96,
		0x392D,
		0x34CC,
		0x3079,
		0x2C39,
		0x2811,
		0x2406,
		0x201D,
		0x1C5C,
		0x18C6,
		0x1560,
		0x122E,
		0x0F34,
		0x0C75,
		0x09F6,
		0x07B9,
		0x05C0,
		0x040F,
		0x02A7,
		0x018A,
		0x00B9,
		0x0036,
		0x0001,
		0x001A,
		0x0081,
		0x0135,
		0x0237,
		0x0384,
		0x051A,
		0x06F9,
		0x091E,
		0x0B85,
		0x0E2D,
		0x1112,
		0x1430,
		0x1783,
		0x1B09,
		0x1EBB,
		0x2296,
		0x2695,
		0x2AB3,
		0x2EEB,
		0x3337,
		0x3794,
};

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;

#define MEMORY_SIZE 512
#define LAST_SAMPLE (MEMORY_SIZE - 1)

#define CORREATION_SIZE (MEMORY_SIZE + MEMORY_SIZE - 1)
#define SCRATCH_SIZE (MEMORY_SIZE + MY_SIGNAL_SIZE + MY_SIGNAL_SIZE)

q15_t correlation[CORREATION_SIZE];
q15_t scratch[SCRATCH_SIZE];

typedef struct
{
	q15_t detectionLevel;
	uint32_t sampleCount;
	q15_t memory[MEMORY_SIZE];
} radar_t;

void turnAntennaON()
{
	HAL_GPIO_WritePin(RADAR_Antenna_GPIO_Port, RADAR_Antenna_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
}

void turnAntennaOFF()
{
	HAL_GPIO_WritePin(RADAR_Antenna_GPIO_Port, RADAR_Antenna_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
}

void taskApp()
{
	radar_t radar;
	radar.detectionLevel = 0x3000;
	radar.sampleCount = 0;
	turnAntennaON();

	while(1)
	{
		/*  */
		if(radar.sampleCount == LAST_SAMPLE)
		{
			// process the signal
			arm_correlate_fast_opt_q15(
					radar.memory,
					(uint32_t)MEMORY_SIZE,
					mySignal,
					(uint32_t)MY_SIGNAL_SIZE,
					correlation,
					scratch);
			bool detected = false;
			for(uint32_t i = 0; i < CORREATION_SIZE; ++i)
			{
				if(correlation[i] > radar.detectionLevel)
				{
					detected = true;
					break;
				}
			}
			if(detected)
			{
				HAL_UART_Transmit(&huart3, "detected\n", 10, HAL_MAX_DELAY);
			}

			radar.sampleCount = 0;
		}
		else
		{
			/* SysTick frec 16MHz (SystemCoreClock = 16000000) => Tsys 62.5 ns */
			/* Audio frec 20kHz => Fsamp 40kHz => Tsamp 25000 ns */
			uint32_t delay = SysTick->VAL + 403;
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			radar.memory[radar.sampleCount] = (q15_t)HAL_ADC_GetValue(&hadc1);
			++radar.sampleCount;
			while(delay > SysTick->VAL);
		}
	}
}
