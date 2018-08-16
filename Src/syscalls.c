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


#include <sys/unistd.h>
#include <sys/times.h>
#include <sys/stat.h>
#include <errno.h>
#include "stm32f7xx_hal.h"
#include "usbd_cdc_if.h"

extern UART_HandleTypeDef huart5;
extern USBD_HandleTypeDef hUsbDeviceFS;

#define UART_Handle   (&huart5)
#define UART_Timeout  (10)
#define USBD_Timeout  (10)

// extern int __io_getchar(void) __attribute__((weak));
// extern int __io_putchar(int ch) __attribute__((weak));

register char * stack_ptr asm("sp");

char *__env[1] = { 0 };
char **environ = __env;

/* Syscalls functions --------------------------------------------------------*/

caddr_t _sbrk(int incr)
{
  extern char end asm("end");
  static char *heap_end;
  char *prev_heap_end;

  if (heap_end == 0)
    heap_end = &end;

  prev_heap_end = heap_end;

  if (heap_end + incr > stack_ptr)
  {
    write(1, "Heap and Stack collision!\n", 26);
    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

void initialise_monitor_handles()
{
}

int _getpid(void)
{
  return 1;
}

int _kill(int pid, int sig)
{
  errno = EINVAL;
  return -1;
}

void _exit (int status)
{
  _kill(status, -1);
  while (1) {}  /* Make sure we hang here */
}

// int _read(int file, char *ptr, int len)
// {
//   int DataIdx;
//
//   for (DataIdx = 0; DataIdx < len; DataIdx++)
//   {
//     *ptr++ = __io_getchar();
//   }
//
//   return len;
// }

#ifdef USE_SERIAL_OVER_UART
int _read(int file, char *ptr, int len)
{
  /* Check file argument */
  if (file != STDIN_FILENO)
  {
    errno = EBADF;
    return -1;
  }

  /* Receive data through UART */
  HAL_StatusTypeDef status;
  status = HAL_UART_Receive(UART_Handle, (uint8_t*)ptr, len, UART_Timeout);

  /* Return # of bytes read - as best we can tell */
  return ( status == HAL_OK ? len : 0 );
}
#endif /* USE_SERIAL_OVER_UART */

#ifdef USE_SERIAL_OVER_USB
int _read(int file, char *ptr, int len)
{
  /* Check file argument */
  if (file != STDIN_FILENO)
  {
    errno = EBADF;
    return -1;
  }

  /* Operation not yet supported */
  return -1;
}
#endif /* USE_SERIAL_OVER_USB */

// int _write(int file, char *ptr, int len)
// {
//   int DataIdx;
//
//   for (DataIdx = 0; DataIdx < len; DataIdx++)
//   {
//     __io_putchar(*ptr++);
//   }
//
//   return len;
// }

#ifdef USE_SERIAL_OVER_UART
int _write(int file, char *ptr, int len)
{
  /* Check file argument */
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
  {
    errno = EBADF;
    return -1;
  }

  /* Transmit data through UART */
  HAL_StatusTypeDef status;
  status = HAL_UART_Transmit(UART_Handle, (uint8_t*)ptr, len, UART_Timeout);

  /* Return # of bytes written - as best we can tell */
  return ( status == HAL_OK ? len : 0 );
}
#endif /* USE_SERIAL_OVER_UART */

#ifdef USE_SERIAL_OVER_USB
int _write(int file, char *ptr, int len)
{
  /* Check file argument */
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
  {
    errno = EBADF;
    return -1;
  }

  /* Transmit data through USB CDC */
  USBD_StatusTypeDef status;
  status = CDC_Transmit_FS((uint8_t*)ptr, len);

  /* Wait until the end of the transfer (Blocking) */
  if (status == USBD_OK)
  {
    uint32_t tickstart = HAL_GetTick();
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    while (hcdc->TxState != 0)
    {
      /* Check for the Timeout */
      if (USBD_Timeout != HAL_MAX_DELAY)
      {
        if ( (USBD_Timeout == 0U) || ((HAL_GetTick()-tickstart) >= USBD_Timeout) )
        {
          status = USBD_FAIL;
        }
      }
    }
  }

  /* Return # of bytes written - as best we can tell */
  return ( status == USBD_OK ? len : 0 );
}
#endif /* USE_SERIAL_OVER_USB */

int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _open(char *path, int flags, ...)
{
  /* Pretend like we always fail */
  return -1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _close(int file)
{
  return -1;
}

int _wait(int *status)
{
  errno = ECHILD;
  return -1;
}

int _link(char *old, char *new)
{
  errno = EMLINK;
  return -1;
}

int _unlink(char *name)
{
  errno = ENOENT;
  return -1;
}

int _times(struct tms *buf)
{
  return -1;
}

int _stat(char *file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _fork(void)
{
  errno = EAGAIN;
  return -1;
}

int _execve(char *name, char **argv, char **env)
{
  errno = ENOMEM;
  return -1;
}
