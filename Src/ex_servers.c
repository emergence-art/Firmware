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
#include "lwip/pbuf.h"
#include "lwip/udp.h"

/* Private defines -----------------------------------------------------------*/
#define UDP_PORT_SYSTEM        10000
#define UDP_PORT_MODULE_BEGIN  10001
#define UDP_PORT_MODULE_END    10032

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
  /* Process [...] */
  printf("EX: System callback on port %u\n", upcb->local_port);

  /* Free the p buffer */
  pbuf_free(p);
}

static void module_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  /* Process [...] */
  printf("EX: Module callback on port %u\n", upcb->local_port);

  /* Free the p buffer */
  pbuf_free(p);
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

  /* Initialize modules */
  for ( int port = UDP_PORT_MODULE_BEGIN ; port < UDP_PORT_MODULE_END ; port++ )
  {
    err = udp_server_init(IP_ADDR_ANY, port, module_callback);
    if (err != ERR_OK)
    {
      printf("EX: Module server cannot be initialized on port %u - error %i\n", port, err);
      _Error_Handler(__FILE__, __LINE__);
    }
  }

  /* Success! */
  printf("EX: Servers successfully initialized!\n");
}
