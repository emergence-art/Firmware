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


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "pio.h"

/* Exported functions --------------------------------------------------------*/

OBJ_StatusTypeDef PIO_Init(PIO_HandleTypeDef *hpio)
{
  /* Check PIO handle allocation */
  if (hpio == NULL)
  {
    return OBJ_ERROR;
  }

  /* Check parameters */
  assert_param(IS_PIO_PIN(hpio->Init.Pin));
  assert_param(IS_PIO_PORT(hpio->Init.Port));
  assert_param(IS_PIO_POLARITY(hpio->Init.Polarity));

  /* Unlock OBJ to READY state */
  __OBJ_UNLOCK_STATE(hpio, OBJ_STATE_READY);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef PIO_DeInit(PIO_HandleTypeDef *hpio)
{
  __OBJ_LOCK(hpio);

  /* Clear resources */
  memset(hpio, 0, sizeof(PIO_HandleTypeDef));

  __OBJ_UNLOCK_STATE(hpio, OBJ_STATE_RESET);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef PIO_Enable(PIO_HandleTypeDef *hpio)
{
  __OBJ_LOCK(hpio);

  /* Enable PIO */
  uint16_t pin = hpio->Init.Pin;
  GPIO_TypeDef *port = hpio->Init.Port;
  if (hpio->Init.Polarity == PIO_POLARITY_HIGH)
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  }

  __OBJ_UNLOCK(hpio);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef PIO_Disable(PIO_HandleTypeDef *hpio)
{
  __OBJ_LOCK(hpio);

  /* Disable PIO */
  uint16_t pin = hpio->Init.Pin;
  GPIO_TypeDef *port = hpio->Init.Port;
  if (hpio->Init.Polarity == PIO_POLARITY_HIGH)
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  }

  __OBJ_UNLOCK(hpio);

  /* Return */
  return OBJ_OK;
}
