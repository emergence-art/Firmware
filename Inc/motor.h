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
#define MOTOR_NB_OF_CHANNELS  (8) // 2 bits per MOTOR, 16 bits per GPIO Port

/* MOTOR channels definition */
#define MOTOR_CHANNEL_0   ((uint16_t)0x0101U)
#define MOTOR_CHANNEL_1   ((uint16_t)0x0202U)
#define MOTOR_CHANNEL_2   ((uint16_t)0x0404U)
#define MOTOR_CHANNEL_3   ((uint16_t)0x0808U)
#define MOTOR_CHANNEL_4   ((uint16_t)0x1010U)
#define MOTOR_CHANNEL_5   ((uint16_t)0x2020U)
#define MOTOR_CHANNEL_6   ((uint16_t)0x4040U)
#define MOTOR_CHANNEL_7   ((uint16_t)0x8080U)
#define MOTOR_CHANNEL_8B  ((uint16_t)0xFFFFU)

/* Exported types ------------------------------------------------------------*/

/* Motion structure definition */
typedef struct {
  double  timestamp;     // Timestamp in seconds
  double  position;      // Position in number of revolutions
  double  velocity;      // Velocity in rps
  double  acceleration;  // Acceleration in rps2
  /* Private */
  double  _fractional;   // Fractional revolution within a step
} motion_t;

/* MOTOR configuration structure definition */
typedef struct {
  GPIO_TypeDef       *Port;    /* GPIOx Port */
  TIM_HandleTypeDef  *htim;    /* Handle to TIM used to generate DMA requests */
  DMA_HandleTypeDef  *hdma;    /* Handle to DMA used to transfer to GPIOx Port */
  uint32_t           Channel;  /* TIM Output Compare Channel */
} MOTOR_InitTypeDef;

/* MOTOR Channel configuration structure definition */
typedef struct {
  PIO_HandleTypeDef  *hpioEnable;
} MOTOR_Channel_ConfigTypeDef;

/* MOTOR Handle structure definition */
typedef struct __MOTOR_HandleTypeDef {
  MOTOR_InitTypeDef            Init;
  size_t                       BufferSize;
  uint32_t                     *BufferData0;
  uint32_t                     *BufferData1;
  uint32_t                     *BufferPointer;
  uint16_t                     ChannelsFlag;
  MOTOR_Channel_ConfigTypeDef  ChannelsConfig[MOTOR_NB_OF_CHANNELS];
  uint32_t                     TIMx_DIER_CCxDE;
  uint32_t                     DMA_SxPAR;
  motion_t                     MotionCurrent[MOTOR_NB_OF_CHANNELS];
  motion_t                     MotionExpected[MOTOR_NB_OF_CHANNELS];
  __IO OBJ_LockTypeDef         Lock;
  __IO OBJ_StateTypeDef        State;
  /* Callbacks */
  void (*CookMotionCallback)(struct __MOTOR_HandleTypeDef *hmotor);
} MOTOR_HandleTypeDef;

/* Exported functions --------------------------------------------------------*/

OBJ_StatusTypeDef MOTOR_Init(MOTOR_HandleTypeDef *hmotor);
OBJ_StatusTypeDef MOTOR_Alloc(MOTOR_HandleTypeDef *hmotor);
OBJ_StatusTypeDef MOTOR_DeInit(MOTOR_HandleTypeDef *hmotor);

OBJ_StatusTypeDef MOTOR_Config(MOTOR_HandleTypeDef *hmotor, MOTOR_Channel_ConfigTypeDef *config, uint32_t channels);
OBJ_StatusTypeDef MOTOR_Enable(MOTOR_HandleTypeDef *hmotor, uint32_t channels);
OBJ_StatusTypeDef MOTOR_Disable(MOTOR_HandleTypeDef *hmotor, uint32_t channels);

OBJ_StatusTypeDef MOTOR_SetMotion(MOTOR_HandleTypeDef *hmotor, motion_t motion, uint32_t channels);

/* Private macros ------------------------------------------------------------*/

#define IS_MOTOR_CHANNEL(__CHANNEL__) (((__CHANNEL__) == MOTOR_CHANNEL_0) || \
                                       ((__CHANNEL__) == MOTOR_CHANNEL_1) || \
                                       ((__CHANNEL__) == MOTOR_CHANNEL_2) || \
                                       ((__CHANNEL__) == MOTOR_CHANNEL_3) || \
                                       ((__CHANNEL__) == MOTOR_CHANNEL_4) || \
                                       ((__CHANNEL__) == MOTOR_CHANNEL_5) || \
                                       ((__CHANNEL__) == MOTOR_CHANNEL_6) || \
                                       ((__CHANNEL__) == MOTOR_CHANNEL_7))
#define IS_MOTOR_CHANNELS(__CHANNELS__) ((__CHANNELS__) > 0 && \
                                         (__CHANNELS__) < (1<<(2*MOTOR_NB_OF_CHANNELS)))

#define IS_MOTOR_PORT(__PORT__) IS_GPIO_ALL_INSTANCE(__PORT__)
#if defined (STM32F765xx)
#define IS_MOTOR_DMA(__INSTANCE__) (((__INSTANCE__)->Instance == DMA2_Stream0) || \
                                    ((__INSTANCE__)->Instance == DMA2_Stream1) || \
                                    ((__INSTANCE__)->Instance == DMA2_Stream2) || \
                                    ((__INSTANCE__)->Instance == DMA2_Stream3) || \
                                    ((__INSTANCE__)->Instance == DMA2_Stream4) || \
                                    ((__INSTANCE__)->Instance == DMA2_Stream6) || \
                                    ((__INSTANCE__)->Instance == DMA2_Stream7))
#define IS_MOTOR_TIM(__INSTANCE__) (((__INSTANCE__)->Instance == TIM1) || \
                                    ((__INSTANCE__)->Instance == TIM8))
#define IS_MOTOR_TIM_CHANNEL(__CHANNEL__) (((__CHANNEL__) == TIM_CHANNEL_1) || \
                                           ((__CHANNEL__) == TIM_CHANNEL_2) || \
                                           ((__CHANNEL__) == TIM_CHANNEL_3) || \
                                           ((__CHANNEL__) == TIM_CHANNEL_4))
#endif /* STM32F765xx */

#define IS_MOTOR_ACTIVE_CHANNELS(__HANDLE__, __CHANNELS__) \
  ( ((__CHANNELS__)|(__HANDLE__)->ChannelsFlag) == (__HANDLE__)->ChannelsFlag )
