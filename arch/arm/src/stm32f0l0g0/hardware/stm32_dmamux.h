/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_dmamux.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DMAMUX_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMAMUX1 0

/* Register Offsets *********************************************************/

#define STM32_DMAMUX_CXCR_OFFSET(x)  (0x0000+0x0004*(x)) /* DMAMUX1 request line multiplexer channel x configuration register */
#define STM32_DMAMUX_C0CR_OFFSET     STM32_DMAMUX_CXCR_OFFSET(0)
#define STM32_DMAMUX_C1CR_OFFSET     STM32_DMAMUX_CXCR_OFFSET(1)
#define STM32_DMAMUX_C2CR_OFFSET     STM32_DMAMUX_CXCR_OFFSET(2)
#define STM32_DMAMUX_C3CR_OFFSET     STM32_DMAMUX_CXCR_OFFSET(3)
#define STM32_DMAMUX_C4CR_OFFSET     STM32_DMAMUX_CXCR_OFFSET(4)
#define STM32_DMAMUX_C5CR_OFFSET     STM32_DMAMUX_CXCR_OFFSET(5)
#define STM32_DMAMUX_C6CR_OFFSET     STM32_DMAMUX_CXCR_OFFSET(6)
                                                        /* 0x01C-0x07C: Reserved */
#define STM32_DMAMUX_CSR_OFFSET      0x0080             /* DMAMUX1 request line multiplexer interrupt channel status register */
#define STM32_DMAMUX_CFR_OFFSET      0x0084             /* DMAMUX1 request line multiplexer interrupt clear flag register */
                                                        /* 0x088-0x0FC: Reserved */
#define STM32_DMAMUX_RGXCR_OFFSET(x) (0x0100+0x004*(x)) /* DMAMUX1 request generator channel x configuration register */
#define STM32_DMAMUX_RG0CR_OFFSET    STM32_DMAMUX_RGXCR_OFFSET(0)
#define STM32_DMAMUX_RG1CR_OFFSET    STM32_DMAMUX_RGXCR_OFFSET(1)
#define STM32_DMAMUX_RG2CR_OFFSET    STM32_DMAMUX_RGXCR_OFFSET(2)
#define STM32_DMAMUX_RG3CR_OFFSET    STM32_DMAMUX_RGXCR_OFFSET(3)
#define STM32_DMAMUX_RGSR_OFFSET     0x0140 /* DMAMUX1 request generator interrupt status register */
#define STM32_DMAMUX_RGCFR_OFFSET    0x0144 /* DMAMUX1 request generator interrupt clear flag register */
                                            /* 0x148-0x3FC: Reserved */

/* Register Addresses *******************************************************/

#define STM32_DMAMUX1_CXCR(x)  (STM32_DMAMUX1_BASE+STM32_DMAMUX_CXCR_OFFSET(x))
#define STM32_DMAMUX1_C0CR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_C0CR_OFFSET)
#define STM32_DMAMUX1_C1CR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_C1CR_OFFSET)
#define STM32_DMAMUX1_C2CR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_C2CR_OFFSET)
#define STM32_DMAMUX1_C3CR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_C3CR_OFFSET)
#define STM32_DMAMUX1_C4CR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_C4CR_OFFSET)
#define STM32_DMAMUX1_C5CR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_C5CR_OFFSET)
#define STM32_DMAMUX1_C6CR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_C6CR_OFFSET)

#define STM32_DMAMUX1_CSR      (STM32_DMAMUX1_BASE+STM32_DMAMUX_CSR_OFFSET)
#define STM32_DMAMUX1_CFR      (STM32_DMAMUX1_BASE+STM32_DMAMUX_CFR_OFFSET)

#define STM32_DMAMUX1_RGXCR(x) (STM32_DMAMUX1_BASE+STM32_DMAMUX_RGXCR_OFFSET(x))
#define STM32_DMAMUX1_RG0CR    (STM32_DMAMUX1_BASE+STM32_DMAMUX_RG0CR_OFFSET)
#define STM32_DMAMUX1_RG1CR    (STM32_DMAMUX1_BASE+STM32_DMAMUX_RG1CR_OFFSET)
#define STM32_DMAMUX1_RG2CR    (STM32_DMAMUX1_BASE+STM32_DMAMUX_RG2CR_OFFSET)
#define STM32_DMAMUX1_RG3CR    (STM32_DMAMUX1_BASE+STM32_DMAMUX_RG3CR_OFFSET)

#define STM32_DMAMUX1_RGSR     (STM32_DMAMUX1_BASE+STM32_DMAMUX_RGSR_OFFSET)
#define STM32_DMAMUX1_RGCFR    (STM32_DMAMUX1_BASE+STM32_DMAMUX_RGCFR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* DMAMUX1 request line multiplexer channel x configuration register */

#define DMAMUX_CCR_DMAREQID_SHIFT (0)                              /* Bits 0-6: DMA request identification */
#define DMAMUX_CCR_DMAREQID_MASK  (0x7f << DMAMUX_CCR_DMAREQID_SHIFT)
#define DMAMUX_CCR_SOIE           (8)                              /* Bit 8: Synchronization overrun interrupt enable */
#define DMAMUX_CCR_EGE            (9)                              /* Bit 9: Event generation enable */
#define DMAMUX_CCR_SE             (16)                             /* Bit 16: Synchronization enable */
#define DMAMUX_CCR_SPOL_SHIFT     (17)                             /* Bits 17-18: Synchronization polarity */
#define DMAMUX_CCR_SPOL_MASK      (0x3 << DMAMUX_CCR_SPOL_SHIFT)
#  define DMAMUX_CCR_SPOL_NONE    (0x0 << DMAMUX_CCR_SPOL_SHIFT)   /* No event: No trigger detection or generation */
#  define DMAMUX_CCR_SPOL_RISING  (0x1 << DMAMUX_CCR_SPOL_SHIFT)   /* Rising edge */
#  define DMAMUX_CCR_SPOL_FALLING (0x2 << DMAMUX_CCR_SPOL_SHIFT)   /* Falling edge */
#  define DMAMUX_CCR_SPOL_BOTH    (0x3 << DMAMUX_CCR_SPOL_SHIFT)   /* Both rising and falling edges */
#define DMAMUX_CCR_NBREQ_SHIFT    (19)                             /* Bits 19-23: Number of DMA request - 1 to forward */
#define DMAMUX_CCR_NBREQ_MASK     (0x1f << DMAMUX_CCR_NBREQ_SHIFT)
#define DMAMUX_CCR_SYNCID_SHIFT   (24)                             /* Bits 24-28: Synchronization identification */
#define DMAMUX_CCR_SYNCID_MASK    (0x1f << DMAMUX_CCR_SYNCID_SHIFT)

/* DMAMUX1 request line multiplexer interrupt channel status register */

#define DMAMUX1_CSR_SOF(x)         (1 << (x)) /* Synchronization overrun event flag */

/* DMAMUX1 request line multiplexer interrupt clear flag register */

#define DMAMUX1_CFR_CSOF(x)        (1 << (x)) /* Clear synchronization overrun event flag */

/* DMAMUX1 request generator channel x configuration register */

#define DMAMUX_RGCR_SIGID_SHIFT    (0)                             /* Bits 0-4: Signal identification */
#define DMAMUX_RGCR_SIGID_MASK     (0x1f << DMAMUX_RGCR_SIGID_SHIFT)
#define DMAMUX_RGCR_OIE            (8)                             /* Bit 8: Trigger overrun interrupt enable */
#define DMAMUX_RGCR_GE             (16)                            /* Bit 16: DMA request generator channel X enable*/
#define DMAMUX_RGCR_GPOL_SHIFT     (17)                            /* Bits 17-18: DMA request generator trigger polarity */
#define DMAMUX_RGCR_GPOL_MASK      (0x3 << DMAMUX_RGCR_GPOL_SHIFT)
#  define DMAMUX_RGCR_GPOL_NONE    (0x0 << DMAMUX_RGCR_GPOL_SHIFT) /* No event: No trigger detection or generation */
#  define DMAMUX_RGCR_GPOL_RISING  (0x1 << DMAMUX_RGCR_GPOL_SHIFT) /* Rising edge */
#  define DMAMUX_RGCR_GPOL_FALLING (0x2 << DMAMUX_RGCR_GPOL_SHIFT) /* Falling edge */
#  define DMAMUX_RGCR_GPOL_BOTH    (0x3 << DMAMUX_RGCR_GPOL_SHIFT) /* Both rising and falling edges */
#define DMAMUX_RGCR_GNBREQ_SHIFT   (19)                            /* Bits 19-23: Number of DMA requests to be generated -1 */
#define DMAMUX_RGCR_GNBREQL_MASK   (0x1f << DMAMUX_RGCR_GNBREQ_SHIFT)

/* DMAMUX1 request generator interrupt status register */

#define DMAMUX1_RGSR_OF(x)         (1 << (x)) /* Trigger overrun event flag */

/* DMAMUX1 request generator interrupt clear flag register */

#define DMAMUX1_RGCFR_COF(x)       (1 << (x)) /* Clear trigger overrun event flag */

/* DMA channel mapping
 *
 * XXXXX.DDD.CCCCCCCC
 * C - DMAMUX request
 * D - DMA controller
 * X - free bits
 */

#define DMAMAP_MAP(d,c)           ((d) << 8 | (c))
#define DMAMAP_CONTROLLER(m)      ((m) >> 8 & 0x07)
#define DMAMAP_REQUEST(m)         ((m) >> 0 & 0xff)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Import DMAMUX map */

#if defined(CONFIG_STM32F0L0G0_STM32G0)
#  include "hardware/stm32g0_dmamux.h"
#elif defined(CONFIG_STM32F0L0G0_STM32C0)
#  include "hardware/stm32c0_dmamux.h"
#else
#  error "Unsupported STM32 M0 sub family"
#endif

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DMAMUX_H */
