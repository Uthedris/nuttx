/****************************************************************************
 * arch/arm/src/rp2040/rp2040_spi_slave.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/slave.h>

#include "arm_internal.h"
#include "chip.h"

#include "rp2040_gpio.h"
#include "rp2040_spi.h"

#ifdef CONFIG_SPI_SLAVE

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct rp2040_xmit_queue_s
{
  struct rp2040_xmit_queue_s *next;
  size_t                      length;
  uint8_t                     data[0];
} rp2040_xmit_queue_t;

typedef struct
{
  struct spi_slave_ctrlr_s  ctrlr;          /* Standard controller fields  */    
  struct spi_slave_dev_s   *dev;            /* Pointer to bound device     */
  rp2040_xmit_queue_t      *xmit_queue;     /* Transmit queue              */
  rp2040_xmit_queue_t      *xmit_queue_end; /* Last item in transmit queue */
  rp2040_xmit_queue_t      *xmit_active;    /* Item being transmitted      */
  size_t                    xmit_ptr;       /* Index of next xmit byte     */
  enum spi_slave_mode_e     mode;           /* Slave mode                  */
  uint8_t                   nbits;          /* transmit word size in bits  */
  uint8_t                   nbytes;         /* transmit word size in bytes */
  uint8_t                   controller;     /* Controller number (0 or 1)  */
} rp2040_spi_slave_ctrlr_t;


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

  static int spi_slave_interrupt(int irq, void *context, void *arg);

  static void     my_bind(struct spi_slave_ctrlr_s *ctrlr,
                          struct spi_slave_dev_s   *sdev,
                          enum spi_slave_mode_e     mode,
                          int                       nbits);

  static void     my_unbind(struct spi_slave_ctrlr_s *ctrlr);

  static int      my_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                             const void               *data,
                             size_t                    nwords);

  static bool     my_qfull(struct spi_slave_ctrlr_s *ctrlr);

  static void     my_qflush(struct spi_slave_ctrlr_s *ctrlr);

  static size_t   my_qpoll(struct spi_slave_ctrlr_s *ctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_slave_ctrlrops_s spi_slave_ctrlrops =
{
  .bind    = my_bind,
  .unbind  = my_unbind,
  .enqueue = my_enqueue,
  .qfull   = my_qfull,
  .qflush  = my_qflush,
  .qpoll   = my_qpoll
};

#ifdef CONFIG_RP2040_SPI0_SLAVE

static rp2040_spi_slave_ctrlr_t spi0_slave_ctrlr =
{
  .ctrlr.ops  = &spi_slave_ctrlrops,
  .controller = 0
};

#endif

#ifdef CONFIG_RP2040_SPI1_SLAVE

static rp2040_spi_slave_ctrlr_t spi1_slave_ctrlr =
{
  .ctrlr.ops  = &spi_slave_ctrlrops,
  .controller = 1
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_slave_interrupt
 ****************************************************************************/

static int spi_slave_interrupt(int irq, void *context, void *arg)
{
  rp2040_spi_slave_ctrlr_t *priv = (rp2040_spi_slave_ctrlr_t *)arg;
  uint32_t  state;

  state = getreg32(RP2040_SPI_SSPMIS(priv->controller));

  /* Transmitter is at least half empty -- send some more */
  if (state & RP2040_SPI_SSPMIS_TXMIS)
    {
      if (priv->xmit_active == NULL && priv->xmit_queue != NULL)
        {
          priv->xmit_active = priv->xmit_queue;
          priv->xmit_queue  = priv->xmit_queue->next;
          priv->xmit_ptr    = 0;
        }

      if (priv->xmit_active != NULL)
        {
          if (priv->xmit_ptr <= priv->xmit_active->length)
            {
              if (priv->nbytes == 1)
                {
                  putreg32(*(uint8_t *)priv->xmit_ptr,
                          RP2040_SPI_SSPDR(priv->controller));
                  priv->xmit_ptr += 1;
                }
              else
                {
                  putreg32(*(uint16_t *)priv->xmit_ptr,
                          RP2040_SPI_SSPDR(priv->controller));
                  priv->xmit_ptr += 2;
                }
            }

          if (priv->xmit_ptr > priv->xmit_active->length)
            {
              kmm_free(priv->xmit_active);
              priv->xmit_active = NULL;
            }
        }
    }

  if (state & RP2040_SPI_SSPMIS_RXMIS)
    {
      
    }

  if (state & RP2040_SPI_SSPMIS_RTMIS)
    {
      
    }

  if (state & RP2040_SPI_SSPMIS_RORMIS)
    {
      
    }

  return OK;
}

/****************************************************************************
 * Name: my_bind
 *
 * Description:
 *   Bind the SPI slave device interface to the SPI slave controller
 *   interface and configure the SPI interface. Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   sdev  - SPI Slave device interface instance
 *   mode  - The SPI Slave mode requested
 *   nbits - The number of bits requested.
 *           If value is greater than 0, then it implies MSB first
 *           If value is less than 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void my_bind(struct spi_slave_ctrlr_s *ctrlr,
             struct spi_slave_dev_s   *sdev,
             enum spi_slave_mode_e     mode,
             int                       nbits)
{
  rp2040_spi_slave_ctrlr_t *priv = (rp2040_spi_slave_ctrlr_t *) ctrlr;

  priv->dev   = sdev;
  priv->mode  = mode;
  priv->nbits = nbits;
}

/****************************************************************************
 * Name: my_unbind
 *
 * Description:
 *   Un-bind the SPI slave device interface from the SPI slave controller
 *   interface. Reset the SPI interface and restore the SPI slave
 *   controller driver to its initial state.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void my_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  rp2040_spi_slave_ctrlr_t *priv = (rp2040_spi_slave_ctrlr_t *) ctrlr;

  priv->dev = NULL;

  my_qflush(ctrlr);
}

/****************************************************************************
 * Name: my_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface. This adds
 *   the word the controller driver for a subsequent transfer but has no
 *   effect on any in-process or currently "committed" transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   data  - Pointer to the command/data mode data to be shifted out.
 *           The data width must be aligned to the nbits parameter which was
 *           previously provided to the bind() method. For nbits of 4 to
 *           8 we assume a data points to and array of uint8_t; for 9 to 16 
 *           we assume it points to an array or uint16_t.
 *   len   - Number of units of "nbits" wide to enqueue,
 *           "nbits" being the data width previously provided to the bind()
 *           method.
 *
 * Returned Value:
 *   Number of data items successfully queued, or a negated errno:
 *         - "len" if all the data was successfully queued
 *         - "0..len-1" if queue is full
 *         - "-errno" in any other error
 *
 ****************************************************************************/
int      my_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                    const void               *data,
                    size_t                    nwords)
{
  rp2040_spi_slave_ctrlr_t *priv = (rp2040_spi_slave_ctrlr_t *) ctrlr;
  rp2040_xmit_queue_t *item = kmm_malloc(sizeof(rp2040_xmit_queue_t)
                                         + priv->nbytes * nwords);

  if (item == NULL)
  {
    return -ENOMEM;
  }

  item->next   = NULL;
  item->length = priv->nbytes * nwords;

  memcpy(item->data, data, item->length);

  if (priv->xmit_queue == NULL)
    {
      priv->xmit_queue = item;
    }
  else
    {
      priv->xmit_queue_end->next = item;
    }

  priv->xmit_queue_end = item;

  return nwords;
}

/****************************************************************************
 * Name: my_qfull
  *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   true if the output queue is full, false otherwise.
 *
 ****************************************************************************/

bool my_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  return false;
}

/****************************************************************************
 * Name: my_qflush
  *
 * Description:
 *   Discard all saved values in the output queue. On return from this
 *   function the output queue will be empty. Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void my_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  rp2040_spi_slave_ctrlr_t *priv = (rp2040_spi_slave_ctrlr_t *) ctrlr;
  rp2040_xmit_queue_t *item      = priv->xmit_queue;
  rp2040_xmit_queue_t *next;

  priv->xmit_queue = NULL;

  while (item != NULL)
    {
      next = item->next;
      kmm_free(item);
      item = next;
    }
}

/****************************************************************************
 * Name: my_qpoll
 *
 * Description:
 *   Tell the controller to output all the receive queue data.
 *
 *   This will cause 1..n SPIS_DEV_RECEIVE calls back to the slave device,
 *   offering blocks of data to the device. From each call, the slave device
 *   driver will return the number of data units it accepted/read out.
 *
 *   The poll will return when:
 *   1. slave device returns that it received less data than what was
 *      offered to it
 *   OR
 *   2. all the buffered data has been offered to the slave device
 *
 *   If the slave device wants the poll to return and leave data into the
 *   controller driver's queue, it shall return any number less than what was
 *   offered. The controller driver will discard the amount of data that the
 *   slave device accepted. The data left to the buffers will be offered
 *   again in the next qpoll.
 *
 *   If the slave device wants the poll to return and let the controller
 *   driver throw away the buffered data it shall just return the same number
 *   of bytes that was offered to each receive call.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Number of units of width "nbits" left in the RX queue. If the device
 *   accepted all the data, the return value will be 0.
 *
 ****************************************************************************/

size_t   my_qpoll(struct spi_slave_ctrlr_s *ctrlr)
{
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPI0_SLAVE

struct spi_slave_ctrlr_s *initialize_spi0_slave_ctrlr(void)
{
  rp2040_spi_slave_ctrlr_t *priv = &spi0_slave_ctrlr;
  int ret;

  rp2040_gpio_set_function(CONFIG_RP2040_SPI0_RX_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  rp2040_gpio_set_function(CONFIG_RP2040_SPI0_CS_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  rp2040_gpio_set_function(CONFIG_RP2040_SPI0_SCK_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  rp2040_gpio_set_function(CONFIG_RP2040_SPI0_TX_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  ret = irq_attach(RP2040_SPI0_IRQ, 
                   spi_slave_interrupt,
                   &(priv->ctrlr));

  if (ret != OK)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  up_enable_irq(RP2040_SPI0_IRQ);

  return &(priv->ctrlr);
}

#endif

#ifdef CONFIG_RP2040_SPI1_SLAVE

struct spi_slave_ctrlr_s *initialize_spi1_slave_ctrlr(void)
{
  rp2040_spi_slave_ctrlr_t *priv = &spi1_slave_ctrlr;
  int ret;

  rp2040_gpio_set_function(CONFIG_RP2040_SPI1_RX_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  rp2040_gpio_set_function(CONFIG_RP2040_SPI1_CS_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  rp2040_gpio_set_function(CONFIG_RP2040_SPI1_SCK_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  rp2040_gpio_set_function(CONFIG_RP2040_SPI1_TX_GPIO,
                           RP2040_GPIO_FUNC_SPI);

  ret = irq_attach(RP2040_SPI1_IRQ, 
                   spi_slave_interrupt,
                   &(priv->ctrlr));

  if (ret != OK)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  up_enable_irq(RP2040_SPI1_IRQ);

  return &(priv->ctrlr);
}

#endif

#endif