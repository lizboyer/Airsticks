/**
  ******************************************************************************
  * @file ADPCM/inc/adpcm.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Header file for adpcm.c
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADPCM_H
#define __ADPCM_H

#define SAMPLE_RATE_DIV 4   // 16000000/256/4 = 15625 Hz
#define ADPCMDATA_DIM 7755 

#define ADPCMD_PI		31250  //updated this to be our length
#define NUMBER_OF_AUDIO_FILES 1

/* Includes ------------------------------------------------------------------*/
//#include "stm32l1xx.h" //-db this must be a file we aren't using?
#include "stm32f4xx_hal.h"


/* Exported types ------------------------------------------------------------*/
typedef union
{
   uint8_t			uBytes[2];
   uint16_t		uShort;
} tTwoByte;

typedef struct
{
	uint32_t	AudioFiles[NUMBER_OF_AUDIO_FILES];
	uint32_t	AudioSize[NUMBER_OF_AUDIO_FILES];
} AudioElement;

/* Exported constants --------------------------------------------------------*/
extern const unsigned char afnm[ADPCMD_PI];
/* Exported macro ------------------------------------------------------------*/
#define NELEMS(x)	(sizeof(x) / sizeof((x)[0]))
//last of the things i added from tutorial part 2 -db
/* Exported functions ------------------------------------------------------- */
uint8_t ADPCM_Encode(int32_t sample);
int16_t ADPCM_Decode(uint8_t code);

#endif /* __ADPCM_H*/
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
