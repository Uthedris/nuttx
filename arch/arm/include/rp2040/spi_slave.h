/****************************************************************************
 * arch/arm/include/rp2040/spi_slave.h
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

#ifndef __ARCH_ARM_INCLUDE_RP2040_SPI_SLAVE_H
#define __ARCH_ARM_INCLUDE_RP2040_SPI_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rp2040_spi_slave_s;

/****************************************************************************
 * Public Data
 ****************************************************************************/


#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: initialize_spi0_slave_ctrlr
 *  
 * Description:
 *   This function initializes the RP2040 SPI0 controller for slave
 *   operations. 

 * Returns: 
 *   A pointer to the RP2040 private SPI slave controller for the SPI0
 *   controller.
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPI0_SLAVE

struct spi_slave_ctrlr_s *initialize_spi0_slave_ctrlr(void);

#endif

/****************************************************************************
 * Name: initialize_spi1_slave_ctrlr
 *  
 * Description:
 *   This function initializes the RP2040 SPI1 controller for slave
 *   operations. 

 * Returns: 
 *   A pointer to the RP2040 private SPI slave controller for the SPI1
 *   controller.
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPI1_SLAVE

struct spi_slave_ctrlr_s *initialize_spi1_slave_ctrlr(void);

#endif

/****************************************************************************
 * These functions allow programmatic access to the RP2040 SPI hardware in
 * slave mode.  They should not be used if an SPI device driver is bound
 * to the controller. To create an SPI slave programmatically
 * include this file (as: <arch/chip/spi_slave.h>) and use either
 * rp2040_spi0_slave_intialize or rp2040_spi1_slave_intialize to
 * initialize the SPI for slave operations.
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_spi0_intialize
 *  
 * Description:
 *   This function initializes the RP2040 SPI0 controller for slave
 *   operations. 

 * Returns: 
 *   A pointer to the RP2040 private SPI slave controller for the SPI0
 *   controller.
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPI0_SLAVE

struct rp2040_spi_slave_s *rp2040_spi0_slave_intialize(void);

#endif

/****************************************************************************
 * Name: rp2040_spi1_intialize
 *  
 * Description:
 *   This function initializes the RP2040 SPI1 controller for slave
 *   operations. 

 * Returns: 
 *   A pointer to the RP2040 private SPI slave controller for the SPI1
 *   controller.
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPI1_SLAVE

struct rp2040_spi_slave_s *rp2040_spi1_slave_intialize(void);

#endif

/****************************************************************************
 * Name: rp2040_spi_deintialize
 *  
 * Description:
 *   This function deinitializes the RP2040 SPI controller.
 *
 * Input Parameters:
 *   ctrlr - pointer to an rp2040_spi_slave_s structure.
 *
 ****************************************************************************/

void rp2040_spi_deintialize(struct rp2040_spi_slave_s *ctrlr);

/****************************************************************************
 * Name: rp2040_spi_queue_data
 *  
 * Description:
 *   This function queues data to be sent to the master.  The data will
 *   be sent synchronously with the SPI clock signal when master has
 *   asserted our chip select signal.
 * 
 *   The data transmitted depends on the bit length of the SPI "word".
 *   The RP2040 permits SPI words of four to 16 bits.  For words of
 *   8 bits or less the data paramater should point to an array or
 *   uint8_t values (or their equivalent) of for works if 9 bits or
 *   more an array of uint16_t.
 *
 * Input Parameters:
 *   ctrlr - pointer to an rp2040_spi_slave_s structure.
 *   data  - pointer to the data to send.
 *   len   - number of words to send.
 *
 * Returns:
 *    OK if the data was queued successfully or a negated errno value
 *    on failure.
 * 
 ****************************************************************************/

void rp2040_spi_queue_data(struct rp2040_spi_slave_s *ctrlr,
                           const void                *data,
                           size_t                     len);

/****************************************************************************
 * Name: rp2040_spi_receive_data
 *  
 * Description:
 *   This function waits for data to be received from the master.
 *   The request will block until the master sends us data.
 * 
 *   The data received depends on the bit length of the SPI "word".
 *   The RP2040 permits SPI words of four to 16 bits.  For words of
 *   8 bits or less the data paramater should point to an array or
 *   uint8_t values (or their equivalent) of for works if 9 bits or
 *   more an array of uint16_t.
 *
 * Input Parameters:
 *   ctrlr - pointer to an rp2040_spi_slave_s structure.
 *   data  - pointer buffer for received data.
 *   len   - number of words to receive.
 *
 * Returns:
 *    The number or words received, which may be less than "len" if the master
 *    unasserted the chip select line before sending the requested number
 *    of words; or a negative errno on error.
 * 
 ****************************************************************************/

ssize_t rp2040_spi_receive_data(struct rp2040_spi_slave_s *ctrlr,
                                void                      *data,
                                size_t                     len);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_RP2040_SPI_SLAVE_H */

