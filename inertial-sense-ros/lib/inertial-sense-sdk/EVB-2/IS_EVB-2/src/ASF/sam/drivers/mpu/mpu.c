/**
 * \file
 *
 * \brief SAMV70/SAMV71/SAME70/SAMS70-XULTRA board mpu config.
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "mpu.h"

/**
 * \defgroup sam_drivers_mpu_group MPU - Memory Protect Unit
 * @{
 */

/** \file */

/**
 * \addtogroup mmu MMU Initialization
 *
 * \section Usage
 *
 * Translation Look-aside Buffers (TLBs) are an implementation technique that
 * caches translations or translation table entries. TLBs avoid the requirement
 * for every memory access to perform a translation table lookup.
 * The ARM architecture does not specify the exact form of the TLB structures
 * for any design. In a similar way to the requirements for caches, the
 * architecture only defines certain principles for TLBs:
 *
 * The MMU supports memory accesses based on memory sections or pages:
 * Super-sections Consist of 16MB blocks of memory. Support for Super sections
 * is optional.
 * -# Sections Consist of 1MB blocks of memory.
 * -# Large pages Consist of 64KB blocks of memory.
 * -# Small pages Consist of 4KB blocks of memory.
 *
 * Access to a memory region is controlled by the access permission bits and
 * the domain field in the TLB entry.
 * Memory region attributes
 * Each TLB entry has an associated set of memory region attributes. These
 * control accesses to the caches,
 * how the write buffer is used, and if the memory region is Shareable and
 * therefore must be kept coherent.
 *
 * Related files:\n
 * \ref mmu.c\n
 * \ref mmu.h \n
 */

/*----------------------------------------------------------------------------
 *        Exported functions

 *----------------------------------------------------------------------------*/
/**
 * \brief Enables the MPU module.
 *
 * \param dwMPUEnable  Enable/Disable the memory region.
 */
void mpu_enable(uint32_t dw_mpu_enable)
{
	MPU->CTRL = dw_mpu_enable ;
}

/**
 * \brief Set active memory region.
 *
 * \param dwRegionNum  The memory region to be active.
 */
void mpu_set_region_num(uint32_t dw_region_num)
{
	MPU->RNR = dw_region_num;
}

/**
 * \brief Disable the current active region.
 */
void mpu_disable_region(void)
{
	MPU->RASR &= 0xfffffffe;
}

/**
 * \brief Setup a memory region.
 *
 * \param dwRegionBaseAddr  Memory region base address.
 * \param dwRegionAttr  Memory region attributes.
 */
void mpu_set_region(uint32_t dw_region_base_addr, uint32_t dw_region_attr)
{
	MPU->RBAR = dw_region_base_addr;
	MPU->RASR = dw_region_attr;
}


/**
 * \brief Calculate region size for the RASR.
 */
uint32_t mpu_cal_mpu_region_size(uint32_t dw_actual_size_in_bytes)
{
	uint32_t dwRegionSize = 32;
	uint32_t dwReturnValue = 4;

	while( dwReturnValue < 31 ) {
		if( dw_actual_size_in_bytes <= dwRegionSize ) {
			break;
		} else {
			dwReturnValue++;
		}
		dwRegionSize <<= 1;
	}

	return ( dwReturnValue << 1 );
}


/**
 *  \brief Update MPU regions.
 *
 *  \return Unused (ANSI-C compatibility).
 */
void mpu_update_regions(uint32_t dw_region_num, uint32_t dw_region_base_addr, uint32_t dw_region_attr)
{
    volatile irqflags_t flags;

	/* Get and clear the global interrupt flags */
	flags = cpu_irq_save();

	/* Clean up data and instruction buffer */
	__DSB();
	__ISB();

	/* Set active region */
	mpu_set_region_num(dw_region_num);

	/* Disable region */
	mpu_disable_region();

	/* Update region attribute */
	mpu_set_region( dw_region_base_addr, dw_region_attr);

	/* Clean up data and instruction buffer to make the new region taking
	   effect at once */
	__DSB();
	__ISB();

	/* Restore global interrupt flags */
	cpu_irq_restore(flags);
}

//@}
