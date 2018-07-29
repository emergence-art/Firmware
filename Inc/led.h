/*******************************************************************************
 *
 *   EMERGENCE PROJECT (http://emergencejourney.org)
 *   Copyright (C) 2018 - David Marchaland (david.marchaland@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/


#pragma once

/* Includes ------------------------------------------------------------------*/
#include "obj.h"
#include "pio.h"

/* Exported constants --------------------------------------------------------*/

/* Parameters */
#define LED_NB_OF_CHANNELS  (16) // 16 bits per GPIO Port

/* LED channels definition */
#define LED_CHANNEL_0    ((uint16_t)0x0001U)
#define LED_CHANNEL_1    ((uint16_t)0x0002U)
#define LED_CHANNEL_2    ((uint16_t)0x0004U)
#define LED_CHANNEL_3    ((uint16_t)0x0008U)
#define LED_CHANNEL_4    ((uint16_t)0x0010U)
#define LED_CHANNEL_5    ((uint16_t)0x0020U)
#define LED_CHANNEL_6    ((uint16_t)0x0040U)
#define LED_CHANNEL_7    ((uint16_t)0x0080U)
#define LED_CHANNEL_8    ((uint16_t)0x0100U)
#define LED_CHANNEL_9    ((uint16_t)0x0200U)
#define LED_CHANNEL_10   ((uint16_t)0x0400U)
#define LED_CHANNEL_11   ((uint16_t)0x0800U)
#define LED_CHANNEL_12   ((uint16_t)0x1000U)
#define LED_CHANNEL_13   ((uint16_t)0x2000U)
#define LED_CHANNEL_14   ((uint16_t)0x4000U)
#define LED_CHANNEL_15   ((uint16_t)0x8000U)
#define LED_CHANNEL_8BL  ((uint16_t)0x00FFU)
#define LED_CHANNEL_8BH  ((uint16_t)0xFF00U)
#define LED_CHANNEL_16B  ((uint16_t)0xFFFFU)

/* Exported types ------------------------------------------------------------*/

/* Color structure definition (ARGB8888 encoding) */
typedef union {
  struct {
    uint8_t  R;
    uint8_t  G;
    uint8_t  B;
    uint8_t  A;
  };
  uint32_t  ARGB;
  uint8_t   Byte[4];
} __attribute__((packed)) color_t;

/* LED configuration structure definition */
typedef struct {
  GPIO_TypeDef       *Port;    /* GPIOx Port */
  TIM_HandleTypeDef  *htim;    /* Handle to TIM used to generate DMA requests */
  DMA_HandleTypeDef  *hdma;    /* Handle to DMA used to transfer to GPIOx Port */
  uint32_t           Channel;  /* TIM Output Compare Channel */
} LED_InitTypeDef;

/* LED Channel configuration structure definition */
typedef struct {
  uint16_t           Length;
  uint8_t            Brightness;
  PIO_HandleTypeDef  *hpioEnable;
} LED_Channel_ConfigTypeDef;

/* LED Handle structure definition */
typedef struct __LED_HandleTypeDef {
  LED_InitTypeDef            Init;
  size_t                     BufferSize;
  void                       *BufferData;
  uint16_t                   ChannelsFlag;
  LED_Channel_ConfigTypeDef  ChannelsConfig[LED_NB_OF_CHANNELS];
  size_t                     NbPixelsPerChannel;
  uint32_t                   TIMx_DIER_CCxDE;
  uint32_t                   DMA_SxPAR;
  __IO OBJ_LockTypeDef       Lock;
  __IO OBJ_StateTypeDef      State;
  /* Callbacks */
  void (*SetPixelsCallback)(struct __LED_HandleTypeDef *hled, color_t color, uint32_t position, uint32_t channels);
} LED_HandleTypeDef;

/* Exported functions --------------------------------------------------------*/

OBJ_StatusTypeDef LED_Init(LED_HandleTypeDef *hled);
OBJ_StatusTypeDef LED_Alloc(LED_HandleTypeDef *hled);
OBJ_StatusTypeDef LED_DeInit(LED_HandleTypeDef *hled);

OBJ_StatusTypeDef LED_Config(LED_HandleTypeDef *hled, LED_Channel_ConfigTypeDef *config, uint32_t channels);
OBJ_StatusTypeDef LED_Enable(LED_HandleTypeDef *hled, uint32_t channels);
OBJ_StatusTypeDef LED_Disable(LED_HandleTypeDef *hled, uint32_t channels);

OBJ_StatusTypeDef LED_SetPixels(LED_HandleTypeDef *hled, color_t color, uint32_t position, uint32_t channels);
OBJ_StatusTypeDef LED_RefreshPixels(LED_HandleTypeDef *hled);

/* Private macros ------------------------------------------------------------*/

#define IS_LED_CHANNEL(__CHANNEL__) (((__CHANNEL__) == LED_CHANNEL_0) || \
                                     ((__CHANNEL__) == LED_CHANNEL_1) || \
                                     ((__CHANNEL__) == LED_CHANNEL_2) || \
                                     ((__CHANNEL__) == LED_CHANNEL_3) || \
                                     ((__CHANNEL__) == LED_CHANNEL_4) || \
                                     ((__CHANNEL__) == LED_CHANNEL_5) || \
                                     ((__CHANNEL__) == LED_CHANNEL_6) || \
                                     ((__CHANNEL__) == LED_CHANNEL_7) || \
                                     ((__CHANNEL__) == LED_CHANNEL_8) || \
                                     ((__CHANNEL__) == LED_CHANNEL_9) || \
                                     ((__CHANNEL__) == LED_CHANNEL_10) || \
                                     ((__CHANNEL__) == LED_CHANNEL_11) || \
                                     ((__CHANNEL__) == LED_CHANNEL_12) || \
                                     ((__CHANNEL__) == LED_CHANNEL_13) || \
                                     ((__CHANNEL__) == LED_CHANNEL_14) || \
                                     ((__CHANNEL__) == LED_CHANNEL_15))
#define IS_LED_CHANNELS(__CHANNELS__) ((__CHANNELS__) > 0 && \
                                       (__CHANNELS__) < (1<<LED_NB_OF_CHANNELS))

#define IS_LED_PORT(__PORT__) IS_GPIO_ALL_INSTANCE(__PORT__)
#if defined (STM32F765xx)
#define IS_LED_DMA(__INSTANCE__) (((__INSTANCE__)->Instance == DMA2_Stream0) || \
                                  ((__INSTANCE__)->Instance == DMA2_Stream1) || \
                                  ((__INSTANCE__)->Instance == DMA2_Stream2) || \
                                  ((__INSTANCE__)->Instance == DMA2_Stream3) || \
                                  ((__INSTANCE__)->Instance == DMA2_Stream4) || \
                                  ((__INSTANCE__)->Instance == DMA2_Stream6) || \
                                  ((__INSTANCE__)->Instance == DMA2_Stream7))
#define IS_LED_TIM(__INSTANCE__) (((__INSTANCE__)->Instance == TIM1) || \
                                  ((__INSTANCE__)->Instance == TIM8))
#define IS_LED_TIM_CHANNEL(__CHANNEL__) (((__CHANNEL__) == TIM_CHANNEL_1) || \
                                         ((__CHANNEL__) == TIM_CHANNEL_2) || \
                                         ((__CHANNEL__) == TIM_CHANNEL_3) || \
                                         ((__CHANNEL__) == TIM_CHANNEL_4))
#endif /* STM32F765xx */

#define IS_LED_ACTIVE_CHANNELS(__HANDLE__, __CHANNELS__) \
  ( ((__CHANNELS__)|(__HANDLE__)->ChannelsFlag) == (__HANDLE__)->ChannelsFlag )
