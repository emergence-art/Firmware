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
#include <stdint.h>
#include "stm32f7xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Generic OBJ status enumeration definition */
typedef enum {
  OBJ_OK       = 0x00U,
  OBJ_LOCK     = 0x01U,
  OBJ_BUSY     = 0x02U,
  OBJ_ERROR    = 0x03U,
  OBJ_TIMEOUT  = 0x04U,
} OBJ_StatusTypeDef;

/* Generic OBJ state enumeration definition */
typedef enum {
  OBJ_STATE_RESET  = 0x00U,
  OBJ_STATE_ALLOC  = 0x01U,
  OBJ_STATE_READY  = 0x02U,
  OBJ_STATE_BUSY   = 0x03U,
  OBJ_STATE_ERROR  = 0x04U,
} OBJ_StateTypeDef;

/* Generic OBJ lock enumeration definition */
typedef enum {
  OBJ_UNLOCKED  = 0x00U,
  OBJ_LOCKED    = 0x01U,
} OBJ_LockTypeDef;

/* Exported macros -----------------------------------------------------------*/

#define  __OBJ_LOCK_STATE(__HANDLE__, __STATE__) \
  do {                                           \
    /* Check handle */                           \
    if ((__HANDLE__) == NULL)                    \
      return OBJ_ERROR;                          \
    /* Update lock */                            \
    if ((__HANDLE__)->Lock == OBJ_LOCKED)        \
      return OBJ_LOCK;                           \
    else                                         \
      (__HANDLE__)->Lock = OBJ_LOCKED;           \
    /* Update state */                           \
    if ((__HANDLE__)->State == OBJ_STATE_BUSY)   \
      return OBJ_BUSY;                           \
    else if ((__HANDLE__)->State != (__STATE__)) \
      return OBJ_ERROR;                          \
    (__HANDLE__)->State = OBJ_STATE_BUSY;        \
  } while (0)

  #define __OBJ_UNLOCK_STATE(__HANDLE__, __STATE__) \
    do {                                            \
      (__HANDLE__)->State = (__STATE__);            \
      (__HANDLE__)->Lock = OBJ_UNLOCKED;            \
    } while (0)

#define __OBJ_LOCK(__HANDLE__) \
  __OBJ_LOCK_STATE(__HANDLE__, OBJ_STATE_READY)

#define __OBJ_UNLOCK(__HANDLE__) \
  __OBJ_UNLOCK_STATE(__HANDLE__, OBJ_STATE_READY)
