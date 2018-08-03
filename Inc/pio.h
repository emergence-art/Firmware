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

/* Exported constants --------------------------------------------------------*/

/* PIO polarities definition */
#define PIO_POLARITY_LOW   ((uint32_t)0x00000001U)
#define PIO_POLARITY_HIGH  ((uint32_t)0x00000002U)

/* Exported types ------------------------------------------------------------*/

/* PIO configuration structure definition */
typedef struct {
  uint16_t      Pin;
  GPIO_TypeDef  *Port;
  uint32_t      Polarity;
} PIO_InitTypeDef;

/* PIO Handle structure definition */
typedef struct {
  PIO_InitTypeDef        Init;
  __IO OBJ_LockTypeDef   Lock;
  __IO OBJ_StateTypeDef  State;
} PIO_HandleTypeDef;

/* Exported functions --------------------------------------------------------*/

OBJ_StatusTypeDef PIO_Init(PIO_HandleTypeDef *hpio);
OBJ_StatusTypeDef PIO_DeInit(PIO_HandleTypeDef *hpio);

OBJ_StatusTypeDef PIO_Enable(PIO_HandleTypeDef *hpio);
OBJ_StatusTypeDef PIO_Disable(PIO_HandleTypeDef *hpio);

/* Private macros ------------------------------------------------------------*/

#define IS_PIO_PIN(__PIN__) IS_GPIO_PIN(__PIN__)
#define IS_PIO_PORT(__PORT__) IS_GPIO_ALL_INSTANCE(__PORT__)
#define IS_PIO_POLARITY(__POLARITY__) (((__POLARITY__) == PIO_POLARITY_LOW) || \
                                       ((__POLARITY__) == PIO_POLARITY_HIGH))
