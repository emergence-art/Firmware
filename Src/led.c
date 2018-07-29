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
#ifdef LED_DEBUG
#include <malloc.h> // For malloc_usable_size()
#endif
#include "led.h"

/* Private defines -----------------------------------------------------------*/

/** WS2812B parameters **
 *
 * We use a "3 steps" pattern to minimize memory usage.
 * {1,0,0} represents a bit "0", timings are  [220-380]@1 + [580-1600]@0
 * {1,1,0} represents a bit "1", timings are [580-1600]@1 +  [220-420]@0
 * In order to be in between in term of timings, we select a step duration of
 * 320ns that gives margins of 60ns @1 for hold and 60ns @0 for setup.
 *
 * Datasheet: http://www.world-semi.com/DownLoadFile/108
 */
#define LED_RGB_BYTES_ORDERING     {1,0,2}
#define LED_NB_BITS_PER_LED        (24)
#define LED_NB_STEPS_PER_BIT       (3)
#define LED_DATA_STEP_OFFSET       (1)
#define LED_DATA_STEP_DURATION_NS  (320)
#define LED_STEPS_INIT_PATTERN     {1,0,0}

/* Private macros ------------------------------------------------------------*/

#define CHECK_BIT(var,pos)  ( (var) & (1<<(pos)) )

#define LED_MALLOC_INIT_ODR(__PTR__, __SIZE__, __TYPE__)            \
  do {                                                              \
    int pattern[LED_NB_STEPS_PER_BIT] = LED_STEPS_INIT_PATTERN;     \
    __PTR__ = malloc((__SIZE__)*sizeof(__TYPE__));                  \
    __TYPE__ *pbuffer = (__TYPE__*)(__PTR__);                       \
    for (size_t i=0; i<(__SIZE__); i++) {                           \
      pbuffer[i] = (__TYPE__)(0UL-pattern[i%LED_NB_STEPS_PER_BIT]); \
    }                                                               \
  } while(0)

/* Private constants ---------------------------------------------------------*/

static const uint8_t BytesOrderRGB[3] = LED_RGB_BYTES_ORDERING;

/* Private functions ---------------------------------------------------------*/

static void __LED_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  LED_HandleTypeDef *hled = (LED_HandleTypeDef *)hdma->Parent;

  __HAL_TIM_DISABLE_DMA(hled->Init.htim, hled->TIMx_DIER_CCxDE);
}

static void __LED_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma)
{
  printf("LED: Error during DMA transfer\n");
}

static void __LED_HAL_TIM_Init(LED_HandleTypeDef *hled)
{
  TIM_OC_InitTypeDef configOC;

  /* Check TIM parameters */
  uint32_t TIMx_PSC = (uint32_t)hled->Init.htim->Instance->PSC;
  uint32_t TIMx_ARR = (uint32_t)hled->Init.htim->Instance->ARR;
  uint32_t period = ( 1000000000 / (2*HAL_RCC_GetPCLK2Freq()) ) * (TIMx_PSC+1) * (TIMx_ARR+1);
#ifdef LED_DEBUG
  printf("LED: TIMx->PSC %lu TIMx->ARR=%lu\n", TIMx_PSC, TIMx_ARR);
#endif
  if (period != LED_DATA_STEP_DURATION_NS)
  {
    printf("LED: Incorrect step duration: expecting %luns but %luns\n", (uint32_t)LED_DATA_STEP_DURATION_NS, period);
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Re-configure TIM OC channel */
  configOC.Pulse        = 0;
  configOC.OCMode       = TIM_OCMODE_TIMING;
  configOC.OCFastMode   = TIM_OCFAST_DISABLE;
  configOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  configOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  configOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  configOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(hled->Init.htim, &configOC, hled->Init.Channel) != HAL_OK)
  {
    printf("LED: Cannot re-configure TIM OC channel %lu\n", hled->Init.Channel);
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Start TIM OC channel */
  if (HAL_TIM_OC_Start(hled->Init.htim, hled->Init.Channel) != HAL_OK)
  {
    printf("LED: Cannot start TIM channel %lu\n", hled->Init.Channel);
    _Error_Handler(__FILE__, __LINE__);
  }
}

static void __LED_HAL_DMA_Init(LED_HandleTypeDef *hled)
{
  /* De-initialize DMA */
  if (HAL_DMA_DeInit(hled->Init.hdma) != HAL_OK)
  {
    printf("LED: Cannot de-initialize DMA\n");
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Update DMA settings */
  hled->Init.hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hled->Init.hdma->Init.MemInc    = DMA_MINC_ENABLE;
  hled->Init.hdma->Init.PeriphInc = DMA_PINC_DISABLE;
  hled->Init.hdma->Init.Mode      = DMA_NORMAL;
  hled->Init.hdma->Init.FIFOMode  = DMA_FIFOMODE_DISABLE;
  hled->Init.hdma->Init.Priority  = DMA_PRIORITY_VERY_HIGH;
  switch (hled->ChannelsFlag)
  {
    case LED_CHANNEL_8BL:
    case LED_CHANNEL_8BH:
      hled->Init.hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      hled->Init.hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      break;
    case LED_CHANNEL_16B:
      hled->Init.hdma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hled->Init.hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      break;
    default:
      hled->Init.hdma->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
      hled->Init.hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  }

  /* Re-initialize DMA */
  if (HAL_DMA_Init(hled->Init.hdma) != HAL_OK)
  {
    printf("LED: Cannot re-initialize DMA\n");
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Register DMA callbacks */
  if (HAL_DMA_RegisterCallback(hled->Init.hdma, HAL_DMA_XFER_CPLT_CB_ID, &__LED_DMA_XferCpltCallback) != HAL_OK)
  {
    printf("LED: Cannot register XferCplt DMA callback\n");
    _Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DMA_RegisterCallback(hled->Init.hdma, HAL_DMA_XFER_ERROR_CB_ID, &__LED_DMA_XferErrorCallback) != HAL_OK)
  {
    printf("LED: Cannot register XferError DMA callback\n");
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Link DMA to LED handle for callbacks */
  __HAL_LINKDMA(hled, Init.hdma, *hled->Init.hdma);
  assert_param(hled->Init.hdma->Parent == hled);
}

static void __LED_SetPixels_ODR8L(LED_HandleTypeDef *hled, color_t color, uint32_t position, uint32_t channels)
{
  size_t idx = LED_DATA_STEP_OFFSET + position*LED_NB_BITS_PER_LED*LED_NB_STEPS_PER_BIT;
  uint32_t mask = channels;

  /* Process each RGB channel */
  for (int i=0; i<3; i++)
  {
    for (int j=7; j>=0; j--)  // MSB first
    {
      int setBits = (CHECK_BIT(color.Byte[BytesOrderRGB[i]], j));  // Get bit value
      uint8_t data = ((uint8_t*)(hled->BufferData))[idx];  // Read
      data = setBits ? data|mask : data&~mask;  // Mask
      ((uint8_t*)(hled->BufferData))[idx] = data;  // Write
      idx += LED_NB_STEPS_PER_BIT; // Increment
    }
  }
}

static void __LED_SetPixels_ODR8H(LED_HandleTypeDef *hled, color_t color, uint32_t position, uint32_t channels)
{
  size_t idx = LED_DATA_STEP_OFFSET + position*LED_NB_BITS_PER_LED*LED_NB_STEPS_PER_BIT;
  uint32_t mask = channels >> 8;

  /* Process each RGB channel */
  for (int i=0; i<3; i++)
  {
    for (int j=7; j>=0; j--)  // MSB first
    {
      int setBits = (CHECK_BIT(color.Byte[BytesOrderRGB[i]], j));  // Get bit value
      uint8_t data = ((uint8_t*)(hled->BufferData))[idx];  // Read
      data = setBits ? data|mask : data&~mask;  // Mask
      ((uint8_t*)(hled->BufferData))[idx] = data;  // Write
      idx += LED_NB_STEPS_PER_BIT; // Increment
    }
  }
}

static void __LED_SetPixels_ODR16(LED_HandleTypeDef *hled, color_t color, uint32_t position, uint32_t channels)
{
  size_t idx = LED_DATA_STEP_OFFSET + position*LED_NB_BITS_PER_LED*LED_NB_STEPS_PER_BIT;
  uint32_t mask = channels;

  /* Process each RGB channel */
  for (int i=0; i<3; i++)
  {
    for (int j=7; j>=0; j--)  // MSB first
    {
      int setBits = (CHECK_BIT(color.Byte[BytesOrderRGB[i]], j));  // Get bit value
      uint16_t data = ((uint16_t*)(hled->BufferData))[idx];  // Read
      data = setBits ? data|mask : data&~mask;  // Mask
      ((uint16_t*)(hled->BufferData))[idx] = data;  // Write
      idx += LED_NB_STEPS_PER_BIT; // Increment
    }
  }
}

static void __LED_SetPixels_BSRR32(LED_HandleTypeDef *hled, color_t color, uint32_t position, uint32_t channels)
{
  size_t idx = LED_DATA_STEP_OFFSET + position*LED_NB_BITS_PER_LED*LED_NB_STEPS_PER_BIT;
  uint32_t mask = channels;

  /* Process each RGB channel */
  for (int i=0; i<3; i++)
  {
    for (int j=7; j>=0; j--)  // MSB first
    {
      int setBits = (CHECK_BIT(color.Byte[BytesOrderRGB[i]], j));  // Get bit value
      uint32_t data = ((uint32_t*)(hled->BufferData))[idx];  // Read
      data = setBits ? (data|mask)&~(mask<<16) : (data|(mask<<16))&~mask;  // Mask
      ((uint32_t*)(hled->BufferData))[idx] = data;  // Write
      idx += LED_NB_STEPS_PER_BIT; // Increment
    }
  }
}

/* Exported functions --------------------------------------------------------*/

OBJ_StatusTypeDef LED_Init(LED_HandleTypeDef *hled)
{
  /* Check LED handle allocation */
  if (hled == NULL)
  {
    return OBJ_ERROR;
  }

  /* Check parameters */
  assert_param(IS_LED_PORT(hled->Init.Port));
  assert_param(IS_LED_DMA(hled->Init.hdma));
  assert_param(IS_LED_TIM(hled->Init.htim));
  assert_param(IS_LED_TIM_CHANNEL(hled->Init.Channel));

  /* Initialize resources */
  hled->BufferSize = 0;
  hled->BufferData = NULL;
  hled->ChannelsFlag = 0x0;
  hled->SetPixelsCallback = NULL;
  switch (hled->Init.Channel)
  {
    case TIM_CHANNEL_1:
      hled->TIMx_DIER_CCxDE = TIM_DMA_CC1;
      break;
    case TIM_CHANNEL_2:
      hled->TIMx_DIER_CCxDE = TIM_DMA_CC2;
      break;
    case TIM_CHANNEL_3:
      hled->TIMx_DIER_CCxDE = TIM_DMA_CC3;
      break;
    case TIM_CHANNEL_4:
      hled->TIMx_DIER_CCxDE = TIM_DMA_CC4;
      break;
    default:
      hled->TIMx_DIER_CCxDE = 0;
  }
  memset(hled->ChannelsConfig, 0, sizeof(LED_Channel_ConfigTypeDef)*LED_NB_OF_CHANNELS);

  /* Unlock OBJ to ALLOC state */
  __OBJ_UNLOCK_STATE(hled, OBJ_STATE_ALLOC);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef LED_Alloc(LED_HandleTypeDef *hled)
{
  __OBJ_LOCK_STATE(hled, OBJ_STATE_ALLOC);

  /* Search for LED channels max length */
  uint32_t maxLength = 0;
  for (size_t i=0; i<LED_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(hled->ChannelsFlag, i))
    {
      uint32_t length = hled->ChannelsConfig[i].Length;
      maxLength = (length>maxLength) ? length : maxLength;
    }
  }
  hled->NbPixelsPerChannel = maxLength;

  /* Allocate and initialize LED buffer */
  hled->BufferSize = maxLength * LED_NB_BITS_PER_LED * LED_NB_STEPS_PER_BIT;
  switch (hled->ChannelsFlag)
  {
    case LED_CHANNEL_8BL:
      LED_MALLOC_INIT_ODR(hled->BufferData, hled->BufferSize, uint8_t);
      hled->SetPixelsCallback = __LED_SetPixels_ODR8L;
      hled->DMA_SxPAR = (uint32_t)(&hled->Init.Port->ODR);
      break;
    case LED_CHANNEL_8BH:
      LED_MALLOC_INIT_ODR(hled->BufferData, hled->BufferSize, uint8_t);
      hled->DMA_SxPAR = (uint32_t)(&hled->Init.Port->ODR)+1;
      hled->SetPixelsCallback = __LED_SetPixels_ODR8H;
      break;
    case LED_CHANNEL_16B:
      LED_MALLOC_INIT_ODR(hled->BufferData, hled->BufferSize, uint16_t);
      hled->SetPixelsCallback = __LED_SetPixels_ODR16;
      hled->DMA_SxPAR = (uint32_t)(&hled->Init.Port->ODR);
      break;
    default:
      do { /* Statement */
        int pattern[LED_NB_STEPS_PER_BIT] = LED_STEPS_INIT_PATTERN;
        hled->BufferData = malloc(hled->BufferSize*sizeof(uint32_t));
        uint32_t *pbuffer = (uint32_t*)(hled->BufferData);
        for (size_t i=0; i<hled->BufferSize; i++)
        {
          int setBits = pattern[i%LED_NB_STEPS_PER_BIT];
          pbuffer[i] = setBits ? hled->ChannelsFlag : hled->ChannelsFlag<<16;
        }
      } while(0);
      hled->SetPixelsCallback = __LED_SetPixels_BSRR32;
      hled->DMA_SxPAR = (uint32_t)(&hled->Init.Port->BSRR);
  }
  if (hled->BufferData == NULL)
  {
    // _sbrk() syscall generates an error message...
    _Error_Handler(__FILE__, __LINE__);
  }
#ifdef LED_DEBUG
  printf("LED: Allocated memory: %u byte(s)\n", malloc_usable_size(hled->BufferData));
  switch (hled->ChannelsFlag)
  {
    case LED_CHANNEL_8BL:
    case LED_CHANNEL_8BH:
      uint8_t *pbuffer = hled->BufferData;
      for (size_t i=0; i<hled->BufferSize; i++) printf("%02x\n", *pbuffer++);
      break;
    case LED_CHANNEL_16B:
      uint16_t *pbuffer = hled->BufferData;
      for (size_t i=0; i<hled->BufferSize; i++) printf("%04x\n", *pbuffer++);
      break;
    default:
      uint32_t *pbuffer = hled->BufferData;
      for (size_t i=0; i<hled->BufferSize; i++) printf("%08x\n", *pbuffer++);
  }
#endif

  /* Initialize HAL */
  __LED_HAL_TIM_Init(hled);
  __LED_HAL_DMA_Init(hled);

  __OBJ_UNLOCK(hled);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef LED_DeInit(LED_HandleTypeDef *hled)
{
  __OBJ_LOCK(hled);

  /* Free/Clear resources */
  free(hled->BufferData);
  memset(hled, 0, sizeof(LED_HandleTypeDef));

  __OBJ_UNLOCK_STATE(hled, OBJ_STATE_RESET);

  /* Return */
  return OBJ_OK;
}

OBJ_StatusTypeDef LED_Config(LED_HandleTypeDef *hled, LED_Channel_ConfigTypeDef *config, uint32_t channels)
{
  __OBJ_LOCK_STATE(hled, OBJ_STATE_ALLOC);

  /* Check parameters */
  assert_param(IS_LED_CHANNELS(channels));

  /* Check channels pin (DATA) parameters */
  for (size_t i=0; i<LED_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(channels, i))
    {
#if defined (STM32F765xx)
      /* General purpose output mode - 2 bits - value @ 1 */
      assert_param(((hled->Init.Port->MODER >> (i*2)) & 0x3) == 1);
      /* Output push-pull - 1 bit - value @ 0 */
      assert_param(((hled->Init.Port->OTYPER >> (i*1)) & 0x1) == 0);
      /* No pull-up neither pull-down - 2 bits - value @ 0 */
      assert_param(((hled->Init.Port->PUPDR >> (i*2)) & 0x3) == 0);
#endif /* STM32F765xx */
    }
  }

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Copy channels configuration to LED handle */
  for (size_t i=0; i<LED_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(channels, i))
    {
      hled->ChannelsFlag |= 1<<i;
      memcpy(&hled->ChannelsConfig[i], config, sizeof(LED_Channel_ConfigTypeDef));
    }
  }

  /* Ensure channels are disabled and lock pins settings */
  for (size_t i=0; i<LED_NB_OF_CHANNELS; i++)
  {
    if (CHECK_BIT(channels, i))
    {
      status |= PIO_Disable(hled->ChannelsConfig[i].hpioEnable);
      if (HAL_GPIO_LockPin(hled->Init.Port, 1<<i) != HAL_OK)
      {
        status |= OBJ_ERROR;
      }
    }
  }

  __OBJ_UNLOCK_STATE(hled, OBJ_STATE_ALLOC);

  /* Return */
  return status;
}

OBJ_StatusTypeDef LED_Enable(LED_HandleTypeDef *hled, uint32_t channels)
{
  __OBJ_LOCK(hled);

  /* Check parameters */
  assert_param(IS_LED_CHANNELS(channels));

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Enable configured channels */
  if (IS_LED_ACTIVE_CHANNELS(hled, channels))
  {
    for (size_t i=0; i<LED_NB_OF_CHANNELS; i++)
    {
      if (CHECK_BIT(channels, i))
      {
        status |= PIO_Enable(hled->ChannelsConfig[i].hpioEnable);
      }
    }
  }
  else
  {
    printf("LED: Cannot enable unactive channel(s)\n");
    status = OBJ_ERROR;
  }

  __OBJ_UNLOCK(hled);

  /* Return */
  return status;
}

OBJ_StatusTypeDef LED_Disable(LED_HandleTypeDef *hled, uint32_t channels)
{
  __OBJ_LOCK(hled);

  /* Check parameters */
  assert_param(IS_LED_CHANNELS(channels));

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Disable configured channels */
  if (IS_LED_ACTIVE_CHANNELS(hled, channels))
  {
    for (size_t i=0; i<LED_NB_OF_CHANNELS; i++)
    {
      if (CHECK_BIT(channels, i))
      {
        status |= PIO_Disable(hled->ChannelsConfig[i].hpioEnable);
      }
    }
  }
  else
  {
    printf("LED: Cannot disable unactive channel(s)\n");
    status = OBJ_ERROR;
  }

  __OBJ_UNLOCK(hled);

  /* Return */
  return status;
}

OBJ_StatusTypeDef LED_SetPixels(LED_HandleTypeDef *hled, color_t color, uint32_t position, uint32_t channels)
{
  __OBJ_LOCK(hled);

  /* Check parameters */
  assert_param(position<hled->NbPixelsPerChannel);
  assert_param(IS_LED_CHANNELS(channels));

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Check DMA state first */
  if (hled->Init.hdma->State == HAL_DMA_STATE_READY)
  {
    /* Call SetPixel callback */
    if (IS_LED_ACTIVE_CHANNELS(hled, channels))
    {
      hled->SetPixelsCallback(hled, color, position, channels);
    }
    else
    {
      printf("LED: Cannot set pixel on unactive channel(s)\n");
      status = OBJ_ERROR;
    }
  }
  else
  {
    printf("LED: DMA not ready\n");
    status = OBJ_ERROR;
  }

  __OBJ_UNLOCK(hled);

  return status;
}

OBJ_StatusTypeDef LED_RefreshPixels(LED_HandleTypeDef *hled)
{
  __OBJ_LOCK(hled);

  /* Local variables */
  OBJ_StatusTypeDef status = OBJ_OK;

  /* Check DMA state first */
  if (hled->Init.hdma->State == HAL_DMA_STATE_READY)
  {
    /* Enable DMA transfer */
    if (HAL_DMA_Start_IT(hled->Init.hdma, (uint32_t)(hled->BufferData), hled->DMA_SxPAR, hled->BufferSize) != HAL_OK)
    {
      printf("LED: Cannot start DMA transfer\n");
      status = OBJ_ERROR;
    }
    else
    {
      __HAL_TIM_ENABLE_DMA(hled->Init.htim, hled->TIMx_DIER_CCxDE);
    }
  }
  else
  {
    printf("LED: DMA not ready\n");
    status = OBJ_ERROR;
  }

  __OBJ_UNLOCK(hled);

  return status;
}
