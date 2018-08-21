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
#include <stdlib.h>
#include <string.h>
#ifdef MOTOR_DEBUG
#include <malloc.h> // For malloc_usable_size()
#endif
#include "motor.h"

/* Private defines -----------------------------------------------------------*/

/** DRV8825 + NEMA-17 parameters **
 *
 * TBD [...]
 *
 * Datasheet: http://www.ti.com/lit/ds/symlink/drv8825.pdf
 */
#define MOTOR_NB_STEPS_PER_REVOLUTION  (200)
#define MOTOR_MAX_VELOCITY_RPM         (1000)
#define MOTOR_MAX_ACCELERATION_RPM2    (10000)
#define MOTOR_STEP_SAMPLING_TIME_NS    (20000) // See TIM settings
#define MOTOR_STEPS_BUFFER_DEPTH       (8192)

#define MOTOR_MAX_VELOCITY_RPS         (MOTOR_MAX_VELOCITY_RPM/60.0)
#define MOTOR_MAX_ACCELERATION_RPS2    (MOTOR_MAX_ACCELERATION_RPM2/(60.0*60.0))

/* Private macros ------------------------------------------------------------*/

#define CHECK_BIT(var,pos)  ( (var) & (1<<(pos)) )

#define MOTION_INIT(__MOTION__)       \
  do {                                \
    (__MOTION__)->timestamp    = 0.0; \
    (__MOTION__)->position     = 0.0; \
    (__MOTION__)->velocity     = 0.0; \
    (__MOTION__)->acceleration = 0.0; \
    (__MOTION__)->_fractional  = 0.0; \
  } while(0)

/* Private functions ---------------------------------------------------------*/

static void __MOTOR_DMA_XferM0CpltCallback(DMA_HandleTypeDef *hdma)
{
  MOTOR_HandleTypeDef *hmotor = (MOTOR_HandleTypeDef *)hdma->Parent;

  /* Change DMA buffer pointing to unused one */
  hmotor->BufferPointer = hmotor->BufferData0;

  hmotor->CookMotionCallback(hmotor);
}

static void __MOTOR_DMA_XferM1CpltCallback(DMA_HandleTypeDef *hdma)
{
  MOTOR_HandleTypeDef *hmotor = (MOTOR_HandleTypeDef *)hdma->Parent;

  /* Change DMA buffer pointing to unused one */
  hmotor->BufferPointer = hmotor->BufferData1;

  hmotor->CookMotionCallback(hmotor);
}

static void __MOTOR_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma)
{
  printf("ERROR: MOTOR: Cannot complete DMA transfer...\n");
}

static void __MOTOR_HAL_TIM_Init(MOTOR_HandleTypeDef *hmotor)
{
  TIM_OC_InitTypeDef configOC;

  /* Check TIM Auto-reload value */
  uint32_t TIMx_PSC = (uint32_t)hmotor->Init.htim->Instance->PSC;
  uint32_t TIMx_ARR = (uint32_t)hmotor->Init.htim->Instance->ARR;
  uint32_t period = ( ( 1000000000 / (2*HAL_RCC_GetPCLK2Freq()) ) * (TIMx_ARR+1) * (TIMx_PSC+1) );
#ifdef MOTOR_DEBUG
  printf("MOTOR: TIMx->ARR %lu [%lu-%lu] (TIMx->PSC=%lu)\n", TIMx_ARR, step_min, step_max, TIMx_PSC);
#endif
  assert_param(period == MOTOR_STEP_SAMPLING_TIME_NS);

  /* Re-configure TIM OC channel */
  configOC.Pulse        = 0;
  configOC.OCMode       = TIM_OCMODE_TIMING;
  configOC.OCFastMode   = TIM_OCFAST_DISABLE;
  configOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  configOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  configOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  configOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(hmotor->Init.htim, &configOC, hmotor->Init.Channel) != HAL_OK)
  {
    printf("FATAL: MOTOR: Cannot re-configure TIM OC channel %lu\n", hmotor->Init.Channel);
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Start TIM OC channel */
  if (HAL_TIM_OC_Start(hmotor->Init.htim, hmotor->Init.Channel) != HAL_OK)
  {
    printf("FATAL: MOTOR: Cannot start TIM channel %lu\n", hmotor->Init.Channel);
    _Error_Handler(__FILE__, __LINE__);
  }
}

static void __MOTOR_HAL_DMA_Init(MOTOR_HandleTypeDef *hmotor)
{
  /* De-initialize DMA */
  if (HAL_DMA_DeInit(hmotor->Init.hdma) != HAL_OK)
  {
    printf("FATAL: MOTOR: Cannot de-initialize DMA\n");
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Update DMA settings */
  hmotor->Init.hdma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hmotor->Init.hdma->Init.MemInc              = DMA_MINC_ENABLE;
  hmotor->Init.hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
  hmotor->Init.hdma->Init.Mode                = DMA_CIRCULAR;
  hmotor->Init.hdma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hmotor->Init.hdma->Init.Priority            = DMA_PRIORITY_HIGH;
  hmotor->Init.hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hmotor->Init.hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;

  /* Re-initialize DMA */
  if (HAL_DMA_Init(hmotor->Init.hdma) != HAL_OK)
  {
    printf("FATAL: MOTOR: Cannot re-initialize DMA\n");
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Register DMA callbacks */
  if (HAL_DMA_RegisterCallback(hmotor->Init.hdma, HAL_DMA_XFER_CPLT_CB_ID, &__MOTOR_DMA_XferM0CpltCallback) != HAL_OK)
  {
    printf("FATAL: MOTOR: Cannot register XferM0Cplt DMA callback\n");
    _Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DMA_RegisterCallback(hmotor->Init.hdma, HAL_DMA_XFER_M1CPLT_CB_ID, &__MOTOR_DMA_XferM1CpltCallback) != HAL_OK)
  {
    printf("FATAL: MOTOR: Cannot register XferM1Cplt DMA callback\n");
    _Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DMA_RegisterCallback(hmotor->Init.hdma, HAL_DMA_XFER_ERROR_CB_ID, &__MOTOR_DMA_XferErrorCallback) != HAL_OK)
  {
    printf("FATAL: MOTOR: Cannot register XferError DMA callback\n");
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Link DMA to MOTOR handle for callbacks */
  __HAL_LINKDMA(hmotor, Init.hdma, *hmotor->Init.hdma);
  assert_param(hmotor->Init.hdma->Parent == hmotor);
}

static void __MOTOR_CookMotionMulti_BSRR32(MOTOR_HandleTypeDef *hmotor)
{
  motion_t *mc = hmotor->MotionCurrent;
  motion_t *me = hmotor->MotionExpected;

  const double dt = (double)(MOTOR_STEP_SAMPLING_TIME_NS)/(1000.0*1000.0*1000.0); // sec
  const double dT = dt*(double)(hmotor->BufferSize); // sec

  double dv[MOTOR_NB_OF_CHANNELS];

  /* Compute acceleration and dv for each active channel */
  for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(hmotor->ChannelsFlag, i))
    {
      mc[i].acceleration = ( me[i].velocity - mc[i].velocity ) / dT ;
      dv[i] = mc[i].acceleration * dt ;
    }
  }

  /* Compute steps sequence for each active channel */
  for (size_t idx=0; idx<hmotor->BufferSize; idx++)
  {
    hmotor->BufferPointer[idx] = 0x00000000;
    for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
    {
      if (CHECK_BIT(hmotor->ChannelsFlag, i))
      {
        /* Update motion parameters */
        mc[i].timestamp   += dt ;
        mc[i].velocity    += dv[i] ;
        mc[i].position    += mc[i].velocity * dt ;
        mc[i]._fractional += mc[i].velocity * dt ;
        /* Direction */
        if ( ( mc[i].velocity - dv[i] ) <= 0.0 && mc[i].velocity > 0.0 )
        {
          hmotor->BufferPointer[idx] |= 1<<(8+i); // Set
        }
        if ( ( mc[i].velocity - dv[i] ) >= 0.0 && mc[i].velocity < 0.0 )
        {
          hmotor->BufferPointer[idx] |= (1<<(8+i))<<16; // Clear
        }
        /* Step */
        if ( mc[i]._fractional >= ( 1.0 / (double)(MOTOR_NB_STEPS_PER_REVOLUTION) ) )
        {
          hmotor->BufferPointer[idx] |= 1<<(i); // Set
          mc[i]._fractional -= 1.0 / (double)(MOTOR_NB_STEPS_PER_REVOLUTION) ;
        }
        else if ( mc[i]._fractional <= ( -1.0 / (double)(MOTOR_NB_STEPS_PER_REVOLUTION) ) )
        {
          hmotor->BufferPointer[idx] |= 1<<(i); // Set
          mc[i]._fractional += 1.0 / (double)(MOTOR_NB_STEPS_PER_REVOLUTION) ;
        }
        else
        {
          hmotor->BufferPointer[idx] |= (1<<(i))<<16; // Clear
        }
      }
    }
  }
}

/* Exported functions --------------------------------------------------------*/

OBJ_StatusTypeDef MOTOR_Init(MOTOR_HandleTypeDef *hmotor)
{
  /* Check MOTOR handle allocation */
  if (hmotor == NULL)
  {
    return OBJ_ERROR;
  }

  /* Check parameters */
  assert_param(IS_MOTOR_PORT(hmotor->Init.Port));
  assert_param(IS_MOTOR_DMA(hmotor->Init.hdma));
  assert_param(IS_MOTOR_TIM(hmotor->Init.htim));
  assert_param(IS_MOTOR_TIM_CHANNEL(hmotor->Init.Channel));

  /* Initialize resources */
  hmotor->BufferSize = 0;
  hmotor->BufferData0 = NULL;
  hmotor->BufferData1 = NULL;
  hmotor->BufferPointer = NULL;
  hmotor->ChannelsFlag = 0x0;
  hmotor->CookMotionCallback = NULL;
  switch (hmotor->Init.Channel)
  {
    case TIM_CHANNEL_1:
      hmotor->TIMx_DIER_CCxDE = TIM_DMA_CC1;
      break;
    case TIM_CHANNEL_2:
      hmotor->TIMx_DIER_CCxDE = TIM_DMA_CC2;
      break;
    case TIM_CHANNEL_3:
      hmotor->TIMx_DIER_CCxDE = TIM_DMA_CC3;
      break;
    case TIM_CHANNEL_4:
      hmotor->TIMx_DIER_CCxDE = TIM_DMA_CC4;
      break;
    default:
      hmotor->TIMx_DIER_CCxDE = 0;
  }
  memset(hmotor->ChannelsConfig, 0, sizeof(MOTOR_Channel_ConfigTypeDef)*MOTOR_NB_OF_CHANNELS);

  /* Initialize channels motions */
  for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
  {
    MOTION_INIT(&hmotor->MotionCurrent[i]);
    MOTION_INIT(&hmotor->MotionExpected[i]);
  }

  /* Unlock OBJ to ALLOC state */
  __OBJ_UNLOCK_STATE(hmotor, OBJ_STATE_ALLOC);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef MOTOR_Alloc(MOTOR_HandleTypeDef *hmotor)
{
  __OBJ_LOCK_STATE(hmotor, OBJ_STATE_ALLOC);

  /* Allocate and initialize MOTOR buffer #0 */
  hmotor->BufferSize = MOTOR_STEPS_BUFFER_DEPTH;
  hmotor->BufferData0 = malloc(hmotor->BufferSize*sizeof(*hmotor->BufferData0));
  if (hmotor->BufferData0 == NULL)
  {
    printf("FATAL: MOTOR: Cannot allocate memory for BufferData #0\n");
    _Error_Handler(__FILE__, __LINE__);
  }
  memset(hmotor->BufferData0, 0, hmotor->BufferSize*sizeof(*hmotor->BufferData0));
#ifdef MOTOR_DEBUG
  printf("MOTOR: Allocated memory for BufferData #0: %u byte(s)\n", malloc_usable_size(hmotor->BufferData0));
#endif

  /* Allocate and initialize MOTOR buffer #1 */
  hmotor->BufferData1 = malloc(hmotor->BufferSize*sizeof(*hmotor->BufferData1));
  if (hmotor->BufferData1 == NULL)
  {
    printf("FATAL: MOTOR: Cannot allocate memory for BufferData #1\n");
    _Error_Handler(__FILE__, __LINE__);
  }
  memset(hmotor->BufferData1, 0, hmotor->BufferSize*sizeof(*hmotor->BufferData1));
#ifdef MOTOR_DEBUG
  printf("MOTOR: Allocated memory for BufferData #1: %u byte(s)\n", malloc_usable_size(hmotor->BufferData1));
#endif

  /* Initialize callbacks pointers */
  hmotor->CookMotionCallback = __MOTOR_CookMotionMulti_BSRR32;
  hmotor->DMA_SxPAR = (uint32_t)(&hmotor->Init.Port->BSRR);

  /* Initialize HAL */
  __MOTOR_HAL_TIM_Init(hmotor);
  __MOTOR_HAL_DMA_Init(hmotor);

  __OBJ_UNLOCK(hmotor);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef MOTOR_DeInit(MOTOR_HandleTypeDef *hmotor)
{
  __OBJ_LOCK(hmotor);

  /* Free/Clear resources */
  free(hmotor->BufferData0);
  free(hmotor->BufferData1);
  memset(hmotor, 0, sizeof(MOTOR_HandleTypeDef));

  __OBJ_UNLOCK_STATE(hmotor, OBJ_STATE_RESET);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef MOTOR_Config(MOTOR_HandleTypeDef *hmotor, MOTOR_Channel_ConfigTypeDef *config, uint32_t channels)
{
  __OBJ_LOCK_STATE(hmotor, OBJ_STATE_ALLOC);

  /* Check parameters */
  assert_param(IS_MOTOR_CHANNELS(channels));

  /* Check channels pins (STEP, DIR) parameters */
  for (size_t i=0; i<2*MOTOR_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(channels, i))
    {
#if defined (STM32F765xx)
      /* General purpose output mode - 2 bits - value @ 1 */
      assert_param(((hmotor->Init.Port->MODER >> (i*2)) & 0x3) == 1);
      /* Output push-pull - 1 bit - value @ 0 */
      assert_param(((hmotor->Init.Port->OTYPER >> (i*1)) & 0x1) == 0);
      /* No pull-up neither pull-down - 2 bits - value @ 0 */
      assert_param(((hmotor->Init.Port->PUPDR >> (i*2)) & 0x3) == 0);
#endif /* STM32F765xx */
    }
  }

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Copy channels configuration to MOTOR handle */
  for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(channels, i))
    {
      hmotor->ChannelsFlag |= 0x0101<<i;
      memcpy(&hmotor->ChannelsConfig[i], config, sizeof(MOTOR_Channel_ConfigTypeDef));
    }
  }

  /* Ensure channels are disabled and lock pins settings */
  for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(channels, i))
    {
      status |= PIO_Disable(hmotor->ChannelsConfig[i].hpioEnable);
      if (HAL_GPIO_LockPin(hmotor->Init.Port, 0x0101<<i) != HAL_OK)
      {
        status |= OBJ_ERROR;
      }
    }
  }

  __OBJ_UNLOCK_STATE(hmotor, OBJ_STATE_ALLOC);

  /* Return */
  return status;
}

OBJ_StatusTypeDef MOTOR_Enable(MOTOR_HandleTypeDef *hmotor, uint32_t channels)
{
  __OBJ_LOCK(hmotor);

  /* Check parameters */
  assert_param(IS_MOTOR_CHANNELS(channels));

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Enable configured channels */
  if (IS_MOTOR_ACTIVE_CHANNELS(hmotor, channels))
  {
    for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
    {
      if (CHECK_BIT(channels, i))
      {
        status |= PIO_Enable(hmotor->ChannelsConfig[i].hpioEnable);
      }
    }
  }
  else
  {
    printf("ERROR: MOTOR: Cannot enable unactive channel(s)\n");
    status = OBJ_ERROR;
  }

  /* Check DMA state first */
  if (hmotor->Init.hdma->State == HAL_DMA_STATE_READY)
  {
    /* Enable DMA transfer */
    if (HAL_DMAEx_MultiBufferStart_IT(hmotor->Init.hdma, (uint32_t)(hmotor->BufferData0), hmotor->DMA_SxPAR, (uint32_t)(hmotor->BufferData1), hmotor->BufferSize) != HAL_OK)
    {
      printf("ERROR: MOTOR: Cannot start DMA transfers...\n");
      status = OBJ_ERROR;
    }
    else
    {
      __HAL_TIM_ENABLE_DMA(hmotor->Init.htim, hmotor->TIMx_DIER_CCxDE);
    }
  }
  else
  {
    printf("ERROR: MOTOR: DMA not ready\n");
    status = OBJ_ERROR;
  }

  __OBJ_UNLOCK(hmotor);

  /* Return */
  return status;
}

OBJ_StatusTypeDef MOTOR_Disable(MOTOR_HandleTypeDef *hmotor, uint32_t channels)
{
  __OBJ_LOCK(hmotor);

  /* Check parameters */
  assert_param(IS_MOTOR_CHANNELS(channels));

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Disable configured channels */
  if (IS_MOTOR_ACTIVE_CHANNELS(hmotor, channels))
  {
    for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
    {
      if (CHECK_BIT(channels, i))
      {
        status |= PIO_Disable(hmotor->ChannelsConfig[i].hpioEnable);
      }
    }
  }
  else
  {
    printf("ERROR: MOTOR: Cannot disable unactive channel(s)\n");
    status = OBJ_ERROR;
  }

  /* Check DMA state first */
  if (hmotor->Init.hdma->State == HAL_DMA_STATE_BUSY)
  {
    /* Abort DMA transfer */
    if (HAL_DMA_Abort_IT(hmotor->Init.hdma) != HAL_OK)
    {
      printf("ERROR: MOTOR: Cannot abort DMA transfers...\n");
      status = OBJ_ERROR;
    }
    else
    {
      __HAL_TIM_DISABLE_DMA(hmotor->Init.htim, hmotor->TIMx_DIER_CCxDE);
      hmotor->BufferPointer = NULL;
    }
  }
  else
  {
    printf("ERROR: MOTOR: DMA not started\n");
    status = OBJ_ERROR;
  }

  __OBJ_UNLOCK(hmotor);

  /* Return */
  return status;
}

OBJ_StatusTypeDef MOTOR_SetMotion(MOTOR_HandleTypeDef *hmotor, uint64_t timestamp, int32_t position, int32_t velocity, uint32_t channels)
{
  __OBJ_LOCK(hmotor);

  /* Check parameters */
  assert_param(IS_MOTOR_CHANNELS(channels));

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Set motion */
  if (IS_MOTOR_ACTIVE_CHANNELS(hmotor, channels))
  {
    for (size_t i=0; i<MOTOR_NB_OF_CHANNELS; i++)
    {
      if (CHECK_BIT(channels, i))
      {
        hmotor->MotionExpected[i].position  = position;
        hmotor->MotionExpected[i].velocity  = velocity;
        hmotor->MotionExpected[i].timestamp = timestamp;
      }
    }
  }
  else
  {
    printf("ERROR: MOTOR: Cannot set motion on unactive channel(s)\n");
    status = OBJ_ERROR;
  }

  __OBJ_UNLOCK(hmotor);

  return status;
}
