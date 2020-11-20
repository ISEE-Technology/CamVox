#ifndef __CORE_CM7_4_30_H
#define __CORE_CM7_4_30_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board_opt.h"

/**
  \brief   D-Cache Invalidate by address
  \details Invalidates D-Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_INLINE void SCB_InvalidateDCache_by_Addr (uint32_t *addr, int32_t dsize)
{
  #if (__DCACHE_PRESENT == 1U) && (CONF_BOARD_ENABLE_DCACHE==1)
     int32_t op_size = dsize;
    uint32_t op_addr = (uint32_t)addr;
     int32_t linesize = 32U;                /* in Cortex-M7 size of cache line is fixed to 8 words (32 bytes) */

    __DSB();

    while (op_size > 0) {
#if (__CM7_CMSIS_VERSION_SUB == 0x00)
      SCB->DCIMVAU = op_addr; // 4.00 typo
#else
      SCB->DCIMVAC = op_addr;
#endif // (__CM7_CMSIS_VERSION_SUB == 0x00)
      op_addr += linesize;
      op_size -= linesize;
    }

    __DSB();
    __ISB();
  #endif
}


/**
  \brief   D-Cache Clean by address
  \details Cleans D-Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_INLINE void SCB_CleanDCache_by_Addr (uint32_t *addr, int32_t dsize)
{
  #if (__DCACHE_PRESENT == 1) && (CONF_BOARD_ENABLE_DCACHE==1)
     int32_t op_size = dsize;
    uint32_t op_addr = (uint32_t) addr;
     int32_t linesize = 32U;                /* in Cortex-M7 size of cache line is fixed to 8 words (32 bytes) */

    __DSB();

    while (op_size > 0) {
      SCB->DCCMVAC = op_addr;
      op_addr += linesize;
      op_size -= linesize;
    }

    __DSB();
    __ISB();
  #endif
}


/**
  \brief   D-Cache Clean and Invalidate by address
  \details Cleans and invalidates D_Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_INLINE void SCB_CleanInvalidateDCache_by_Addr (uint32_t *addr, int32_t dsize)
{
  #if (__DCACHE_PRESENT == 1U) && (CONF_BOARD_ENABLE_DCACHE==1)
     int32_t op_size = dsize;
    uint32_t op_addr = (uint32_t) addr;
     int32_t linesize = 32U;                /* in Cortex-M7 size of cache line is fixed to 8 words (32 bytes) */

    __DSB();

    while (op_size > 0) {
      SCB->DCCIMVAC = op_addr;
      op_addr += linesize;
      op_size -= linesize;
    }

    __DSB();
    __ISB();
  #endif
}

#if CONF_BOARD_ENABLE_DCACHE == 1
	// move address back until it is 32 byte aligned, add to size the amount that address was moved back
	#define SCB_CLEANINVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED(addr,size)	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)(((uint32_t)(addr))&0xFFFFFFE0), (size) + (((uint32_t)(addr))&0x0000001F))
	//This one can be unsafe
	//#define SCB_INVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED(addr,size)			SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)(addr))&0xFFFFFFE0), (size) + (((uint32_t)(addr))&0x0000001F))
	#define SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED(addr,size)				SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)(addr))&0xFFFFFFE0), (size) + (((uint32_t)(addr))&0x0000001F))
#else
	#define SCB_CLEANINVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED(addr,size)
	#define SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED(addr,size)
#endif

#ifdef __cplusplus
}
#endif
#endif // __CORE_CM7_4_30_H
