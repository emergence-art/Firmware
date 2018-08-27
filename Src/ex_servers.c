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
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/stats.h"
#include "tinyosc.h"
#include "ex_leds.h"
#include "ex_motors.h"

/* Private defines -----------------------------------------------------------*/

#define UDP_PORT_SYSTEM  10000
#define UDP_PORT_MODULE  10001

#define NB_SYSTEM_CHANNELS  32

/*  Emergence OSC bundle content
 *  [ LED(25, argb) + MOTOR(1, position) + MOTOR(1, velocity) ]
 */
#define OSC_MAX_PACKET_SIZE    4096
#define OSC_NB_OF_MODULES        24
#define OSC_NB_OF_MESSAGES       27
#define OSC_LEDS_MESSAGE_INDEX    0
#define OSC_MOTOR_MESSAGE_INDEX  25

int32_t MOTOR_MIN_POSITION     =  0;
int32_t MOTOR_MAX_POSITION     =  5;
int32_t MOTOR_MAX_VELOCITY     = 10;
int32_t MOTOR_MAX_ACCELERATION =  1;

/* Private functions ---------------------------------------------------------*/

static err_t udp_server_init(const ip_addr_t *addr, u16_t port, udp_recv_fn callback)
{
  err_t err;
  struct udp_pcb *upcb;

  /* Create a new UDP control block */
  upcb = udp_new();

  if (upcb != NULL)
  {
    /* Bind the UDP control block to the UDP port */
    err = udp_bind(upcb, addr, port);

    if (err == ERR_OK)
    {
      /* Set a receive callback for the upcb */
      udp_recv(upcb, callback, NULL);
    }
    else
    {
      udp_remove(upcb);
    }
  }
  else
  {
    err = ERR_MEM;
  }

  return err;
}

/**
  * These callback functions are called when an UDP datagram has been received
  * on the selected IP address and port.
  *
  * @param arg   user supplied argument (udp_pcb.recv_arg)
  * @param pcb   the udp_pcb which received data
  * @param p     the packet buffer that was received
  * @param addr  the remote IP address from which the packet was received
  * @param port  the remote port from which the packet was received
  */

static void system_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  static _Bool status[NB_SYSTEM_CHANNELS] = {0};

  if (p->len == p->tot_len)
  {
    if (tosc_isBundle(p->payload))
    {
      tosc_bundle bundle;
      tosc_parseBundle(&bundle, p->payload, p->len);
      tosc_message osc;
      while (tosc_getNextMessage(&bundle, &osc))
      {
        // tosc_printMessage(&osc);
        for ( size_t i = 0 ; osc.format[i] != '\0' && i < NB_SYSTEM_CHANNELS ; i++ )
        {
          if (osc.format[i] == 'i')
          {
            int32_t value = tosc_getNextInt32(&osc);
            switch (i)
            {
              case 2: /* LED's Test Mode */
              // Run first to disable after the blackout
                if (value)
                {
                  if (value != status[i])
                  {
                    printf("LED's Test Mode enabled\n");
                  }
                  EX_LEDS_RunTestMode(false, 0);
                }
                else
                {
                  if (value != status[i])
                  {
                    EX_LEDS_BlackoutPixels();
                    HAL_Delay(1);
                    printf("LED's Test Mode disabled\n");
                  }
                }
                break;
              case 0: /* LED's enable/disable */
                if (value)
                {
                  if (value != status[i])
                  {
                    EX_LEDS_Enable();
                    printf("LED's enabled\n");
                  }
                }
                else
                {
                  if (value != status[i])
                  {
                    EX_LEDS_Disable();
                    printf("LED's disabled\n");
                  }
                }
                break;
              case 1: /* MOTOR's enable/disable */
                if (value)
                {
                  if (value != status[i])
                  {
                    EX_MOTORS_Enable();
                    printf("MOTOR's enabled\n");
                  }
                }
                else
                {
                  if (value != status[i])
                  {
                    EX_MOTORS_Disable();
                    printf("MOTOR's disabled\n");
                  }
                }
                break;
              case 3: /* MOTOR's Test Mode */
                if (value)
                {
                  if (value != status[i])
                  {
                    printf("MOTOR's Test Mode enabled\n");
                  }
                  EX_MOTORS_RunTestMode(false, 0);
                }
                else
                {
                  if (value != status[i])
                  {
                    printf("MOTOR's Test Mode disabled\n");
                  }
                }
                break;
              case 4: /* MOTOR MIN Position */
                MOTOR_MIN_POSITION = value;
                printf("MOTOR's minimum position set to %ld\n", value);
                break;
              case 5: /* MOTOR MAX Position */
                MOTOR_MAX_POSITION = value;
                printf("MOTOR's maximum position set to %ld\n", value);
                break;
              case 6: /* MOTOR MAX Velocity */
                MOTOR_MAX_VELOCITY = value;
                printf("MOTOR's maximum velocity set to %ld\n", value);
                break;
              case 7: /* MOTOR MAX Acceleration */
                MOTOR_MAX_ACCELERATION = value;
                printf("MOTOR's maximum acceleration set to %ld\n", value);
                break;
              case 8: /* MOTOR set motion */
                {
                  if (value != 0)
                  {
                    i++; // We assume it's Int32
                    int32_t position = tosc_getNextInt32(&osc);
                    i++; // We assume it's Int32
                    int32_t velocity = tosc_getNextInt32(&osc);
                    EX_MOTORS_SetGlobalMotion(0, position, velocity);
                    printf("MOTOR's global motion set to p=%ld v=%ld\n", position, velocity);
                  }
                }
                break;
              default:
                printf("EX: Unknown OSC system message at index %u\n", i);
            }
            status[i] = value;
          }
          else
          {
            printf("EX: Bundle OSC packet contains a non-integer value...\n");
            HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
          }
        }
      }
    }
    else
    {
      printf("EX: Bundle OSC packet expected for system control\n");
      HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
    }
  }
  else
  {
    printf("EX: System callback designed for single packet buffer only\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }

  /* Free the p buffer */
  pbuf_free(p);
}

static void module_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  static uint32_t counter = 0;
  static uint32_t timestamp = 0;
  uint32_t tick = HAL_GetTick();

  /* Static buffer to store p buffer */
  static char buffer[OSC_MAX_PACKET_SIZE];
  uint16_t length = 0;

  /* Check p buffer tot_len */
  if (p->tot_len > OSC_MAX_PACKET_SIZE)
  {
    printf("EX: Buffer overflow in module callback\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }
  else
  {
    length = p->tot_len;
  }

  /* Fill the local buffer with p buffer */
  char *ptr = buffer;
  memcpy(ptr, p->payload, p->len);
  struct pbuf *q = p;
  while (q->len != q->tot_len)
  {
    q = q->next;
    ptr += q->len;
    memcpy(ptr, q->payload, q->len);
  }

  /* Free the p buffer */
  pbuf_free(p);

  /* Parse OSC packet */
  if (tosc_isBundle(buffer))
  {
    tosc_bundle bundle;
    tosc_parseBundle(&bundle, buffer, length);
    uint64_t timetag = tosc_getTimetag(&bundle);
    tosc_message osc;
    while (tosc_getNextMessage(&bundle, &osc))
    {
      // tosc_printMessage(&osc);
      for ( size_t i = 0 ; osc.format[i] != '\0' ; i++ )
      {
        if (osc.format[i] == 'i')
        {
          uint16_t channel = i/OSC_NB_OF_MESSAGES;
          if (i == OSC_NB_OF_MESSAGES*OSC_NB_OF_MODULES)
          {
            if (tosc_getNextInt32(&osc) != 0)
            {
              EX_MOTORS_Enable();
            }
            else
            {
              EX_MOTORS_Disable();
            }
          }
          else if (i%OSC_NB_OF_MESSAGES == OSC_MOTOR_MESSAGE_INDEX)
          {
            int32_t position = tosc_getNextInt32(&osc);
            i++; // We assume it's Int32
            int32_t velocity = tosc_getNextInt32(&osc);
            EX_MOTORS_SetMotion(timetag, position, velocity, channel);
          }
          else
          {
            uint32_t argb = (uint32_t)(tosc_getNextInt32(&osc));
            EX_LEDS_SetPixel(argb, i%OSC_NB_OF_MESSAGES, channel);
          }
        }
        else
        {
          printf("EX: Bundle OSC packet contains a non-integer value...\n");
          HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
        }
      }
    }
  }
  else
  {
    printf("EX: Bundle OSC packet expected for module control\n");
    HAL_GPIO_WritePin(BRD_LED1_R_GPIO_Port, BRD_LED1_R_Pin, GPIO_PIN_SET);
  }

  /* Refresh LED's once all pixels are set */
  EX_LEDS_RefreshPixels();

  /* Print internal frames counter */
  printf("Frame #%lu (%lums)\n", counter, tick-timestamp);
  counter++;
  timestamp = tick;

  /* Display LwIP stats each 1000 frames */
#ifdef LWIP_STATS_DISPLAY
  if (counter%1000 == 0)
  {
    stats_display();
  }
#endif
}

/* Exported functions --------------------------------------------------------*/

void EX_SERVERS_Init(void)
{
  err_t err;

  /* Initialize system */
  err = udp_server_init(IP_ADDR_ANY, UDP_PORT_SYSTEM, system_callback);
  if (err != ERR_OK)
  {
    printf("EX: System server cannot be initialized - error %i\n", err);
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Initialize module */
  err = udp_server_init(IP_ADDR_ANY, UDP_PORT_MODULE, module_callback);
  if (err != ERR_OK)
  {
    printf("EX: Module server cannot be initialized - error %i\n", err);
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Success! */
  printf("EX: Servers successfully initialized!\n");
}
