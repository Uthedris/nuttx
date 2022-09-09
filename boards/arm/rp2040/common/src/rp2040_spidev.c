/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_spidev.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/spi/spi_transfer.h>

#include "rp2040_spi.h"

#if defined(CONFIG_SPI_SLAVE_DRIVER) && defined(CONFIG_RP2040_SPI)
#include <arch/chip/spi_slave.h>
#include <nuttx/spi/slave.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spidev_initialize
 *
 * Description:
 *   Initialize and register spi driver for the specified spi port
 *
 ****************************************************************************/

#if defined(CONFIG_RP2040_SPI) && defined(CONFIG_SPI_DRIVER)

int board_spidev_initialize(int port)
{
  int ret;
  struct spi_dev_s *spi;

  spiinfo("Initializing /dev/spi%d..\n", port);

  /* Initialize spi device */

  spi = rp2040_spibus_initialize(port);
  if (!spi)
    {
      spierr("ERROR: Failed to initialize spi%d.\n", port);
      return -ENODEV;
    }

  ret = spi_register(spi, port);
  if (ret < 0)
    {
      spierr("ERROR: Failed to register spi%d: %d\n", port, ret);
    }

  return ret;
}

#endif /* defined(CONFIG_RP2040_SPI0) || defined(CONFIG_RP2040_SPI1) */

/****************************************************************************
 * Name: board_spi_slave_dev_initialize
 *
 * Description:
 *   Initialize spi slave driver and register the /dev/spislv[n] device.
 *
 ****************************************************************************/

#if defined(CONFIG_SPI_SLAVE_DRIVER) && defined(CONFIG_RP2040_SPI)

int board_spi_slave_dev_initialize(int bus)
{
  struct spi_slave_ctrlr_s * ctrlr = NULL;

#  ifdef CONFIG_RP2040_SPI0_SLAVE

  if (bus == 0)
    {
      ctrlr = initialize_spi0_slave_ctrlr();
    }

#  endif

#  ifdef CONFIG_RP2040_SPI1_SLAVE

  if (bus == 1)
    {
      ctrlr = initialize_spi1_slave_ctrlr();
    }

#  endif

  return (ctrlr == NULL) ? -EINVAL : spi_slave_register(ctrlr, bus);
}

#endif