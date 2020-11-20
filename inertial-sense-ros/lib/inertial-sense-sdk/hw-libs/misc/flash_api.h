/**
 * \file
 *
 * \brief API for AVR UC3 internal FLASH software drivers.
 *
 * AVR UC3 Flash Controller software driver module.
 *
 * Copyright (c) 2006-2011 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */


#ifndef _FLASH_API_H_
#define _FLASH_API_H_

#include <avr32/io.h>
#include "compiler.h"


#if defined(AVR32_FLASHC)

//! Flash Base Address
#define FLASH_API_BASE_ADDRESS      AVR32_FLASH

#define FLASH_API_SIZE              AVR32_FLASH_SIZE

//! Flash size.
#define FLASH_API_FLASH_SIZE        AVR32_FLASHC_FLASH_SIZE

//! Flash page size.
#define FLASH_API_PAGE_SIZE         AVR32_FLASHC_PAGE_SIZE

//! Number of flash regions defined by the FLASHC.
#define FLASH_API_REGIONS           AVR32_FLASHC_REGIONS

//! User page size
#define FLASH_API_USER_PAGE_SIZE    AVR32_FLASHC_USER_PAGE_SIZE

//! User page
#define FLASH_API_USER_PAGE         AVR32_FLASHC_USER_PAGE

//! User page address
#define FLASH_API_USER_PAGE_ADDRESS AVR32_FLASHC_USER_PAGE_ADDRESS

//! Number of GP fuses
#define FLASH_API_GPF_NUM           AVR32_FLASHC_GPF_NUM

/*! \name Flash commands
 */
//! @{
// Note: all commands supported by the flashc driver should be abstracted here.
#define FLASH_API_FCMD_CMD_EP       AVR32_FLASHC_FCMD_CMD_EP
#define FLASH_API_FCMD_CMD_EUP      AVR32_FLASHC_FCMD_CMD_EUP

//! @}

#ifdef __AVR32_ABI_COMPILER__ // Automatically defined when compiling for AVR32, not when assembling.

#include "flashc.h"

/*! \name Flash Properties
 */
//! @{

/*! \brief Gets the size of the whole flash array.
 *
 * \return The size of the whole flash array in bytes.
 */
#define flash_api_get_flash_size flashc_get_flash_size

/*! \brief Gets the total number of pages in the flash array.
 *
 * \return The total number of pages in the flash array.
 */
#define flash_api_get_page_count flashc_get_page_count

/*! \brief Gets the number of pages in each flash region.
 *
 * \return The number of pages in each flash region.
 */
#define flash_api_get_page_count_per_region  flashc_get_page_count_per_region

/*! \brief Gets the region number of a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \return The region number of the specified page.
 */
#define flash_api_get_page_region  flashc_get_page_region

/*! \brief Gets the number of the first page of a region.
 *
 * \param region The region number: \c 0 to <tt>(AVR32_FLASHC_REGIONS - 1)</tt>.
 *
 * \return The number of the first page of the specified region.
 */
#define flash_api_get_region_first_page_number flashc_get_region_first_page_number

//! @}


/*! \name FLASHC Control
 */
//! @{

/*! \brief Gets the number of wait states of flash read accesses.
 *
 * \return The number of wait states of flash read accesses.
 */
#define flash_api_get_wait_state   flashc_get_wait_state

/*! \brief Sets the number of wait states of flash read accesses.
 *
 * \param wait_state The number of wait states of flash read accesses: \c 0 to
 *                   \c 1.
 */
#define flash_api_set_wait_state     flashc_set_wait_state

/*! \brief Tells whether the Flash Ready interrupt is enabled.
 *
 * \return Whether the Flash Ready interrupt is enabled.
 */
#define flash_api_is_ready_int_enabled flashc_is_ready_int_enabled

/*! \brief Enables or disables the Flash Ready interrupt.
 *
 * \param enable Whether to enable the Flash Ready interrupt: \c true or
 *               \c false.
 */
#define flash_api_enable_ready_int     flashc_enable_ready_int

/*! \brief Tells whether the Lock Error interrupt is enabled.
 *
 * \return Whether the Lock Error interrupt is enabled.
 */
#define flash_api_is_lock_error_int_enabled  flashc_is_lock_error_int_enabled

/*! \brief Enables or disables the Lock Error interrupt.
 *
 * \param enable Whether to enable the Lock Error interrupt: \c true or
 *               \c false.
 */
#define flash_api_enable_lock_error_int  flashc_enable_lock_error_int

/*! \brief Tells whether the Programming Error interrupt is enabled.
 *
 * \return Whether the Programming Error interrupt is enabled.
 */
#define flash_api_is_prog_error_int_enabled  flashc_is_prog_error_int_enabled

/*! \brief Enables or disables the Programming Error interrupt.
 *
 * \param enable Whether to enable the Programming Error interrupt: \c true or
 *               \c false.
 */
#define flash_api_enable_prog_error_int    flashc_enable_prog_error_int

//! @}


/*! \name FLASHC Status
 */
//! @{

/*! \brief Tells whether the FLASHC is ready to run a new command.
 *
 * \return Whether the FLASHC is ready to run a new command.
 */
#define flash_api_is_ready   flashc_is_ready

/*! \brief Waits actively until the FLASHC is ready to run a new command.
 *
 * This is the default function assigned to \ref flash_api_wait_until_ready.
 */
#define flash_api_default_wait_until_ready flashc_default_wait_until_ready

//! Pointer to the function used by the driver when it needs to wait until the
//! FLASHC is ready to run a new command.
//! The default function is \ref flash_api_default_wait_until_ready.
//! The user may change this pointer to use another implementation.
#define flash_api_wait_until_ready         flashc_wait_until_ready

/*! \brief Tells whether a Lock Error has occurred during the last function
 *         called that issued one or more FLASHC commands.
 *
 * \return Whether a Lock Error has occurred during the last function called
 *         that issued one or more FLASHC commands.
 */
#define flash_api_is_lock_error            flashc_is_lock_error

/*! \brief Tells whether a Programming Error has occurred during the last
 *         function called that issued one or more FLASHC commands.
 *
 * \return Whether a Programming Error has occurred during the last function
 *         called that issued one or more FLASHC commands.
*/
#define flash_api_is_programming_error     flashc_is_programming_error

//! @}


/*! \name FLASHC Command Control
 */
//! @{

/*! \brief Gets the last issued FLASHC command.
 *
 * \return The last issued FLASHC command.
 */
#define flash_api_get_command              flashc_get_command

/*! \brief Gets the current FLASHC page number.
 *
 * \return The current FLASHC page number.
 */
#define flash_api_get_page_number          flashc_get_page_number

/*! \brief Issues a FLASHC command.
 *
 * \param command The command: \c AVR32_FLASHC_FCMD_CMD_x.
 * \param page_number The page number to apply the command to:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: use this to apply the command to the current page number
 *        or if the command does not apply to any page number;
 *   \arg this argument may have other meanings according to the command. See
 *        the FLASHC chapter of the MCU datasheet.
 *
 * \warning A Lock Error is issued if the command violates the protection
 *          mechanism.
 *
 * \warning A Programming Error is issued if the command is invalid.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_issue_command      flashc_issue_command

//! @}


/*! \name FLASHC Global Commands
 */
//! @{

/*! \brief Issues a No Operation command to the FLASHC.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_no_operation         flashc_no_operation

/*! \brief Issues an Erase All command to the FLASHC.
 *
 * This command erases all bits in the flash array, the general-purpose fuse
 * bits and the Security bit. The User page is not erased.
 *
 * This command also ensures that all volatile memories, such as register file
 * and RAMs, are erased before the Security bit is erased, i.e. deactivated.
 *
 * \warning A Lock Error is issued if at least one region is locked or the
 *          bootloader protection is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_all            flashc_erase_all

//! @}


/*! \name FLASHC Protection Mechanisms
 */
//! @{

/*! \brief Tells whether the Security bit is active.
 *
 * \return Whether the Security bit is active.
 */
#define flash_api_is_security_bit_active   flashc_is_security_bit_active

/*! \brief Activates the Security bit.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_activate_security_bit    flashc_activate_security_bit

/*! \brief Gets the bootloader protected size.
 *
 * \return The bootloader protected size in bytes.
 */
#define flash_api_get_bootloader_protected_size  flashc_get_bootloader_protected_size

/*! \brief Sets the bootloader protected size.
 *
 * \param bootprot_size The wanted bootloader protected size in bytes. If this
 *                      size is not supported, the actual size will be the
 *                      nearest greater available size or the maximal possible
 *                      size if the requested size is too large.
 *
 * \return The actual bootloader protected size in bytes.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_set_bootloader_protected_size  flashc_set_bootloader_protected_size

/*! \brief Tells whether external privileged fetch is locked.
 *
 * \return Whether external privileged fetch is locked.
 */
#define flash_api_is_external_privileged_fetch_locked  flashc_is_external_privileged_fetch_locked

/*! \brief Locks or unlocks external privileged fetch.
 *
 * \param lock Whether to lock external privileged fetch: \c true or \c false.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_external_privileged_fetch   flashc_lock_external_privileged_fetch

/*! \brief Tells whether the region of a page is locked.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \return Whether the region of the specified page is locked.
 */
#define flash_api_is_page_region_locked  flashc_is_page_region_locked

/*! \brief Tells whether a region is locked.
 *
 * \param region The region number: \c 0 to <tt>(AVR32_FLASHC_REGIONS - 1)</tt>.
 *
 * \return Whether the specified region is locked.
 */
#define flash_api_is_region_locked       flashc_is_region_locked

/*! \brief Locks or unlocks the region of a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 * \param lock Whether to lock the region of the specified page: \c true or
 *             \c false.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_page_region       flashc_lock_page_region

/*! \brief Locks or unlocks a region.
 *
 * \param region The region number: \c 0 to <tt>(AVR32_FLASHC_REGIONS - 1)</tt>.
 * \param lock Whether to lock the specified region: \c true or \c false.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_region            flashc_lock_region

/*! \brief Locks or unlocks all regions.
 *
 * \param lock Whether to lock the regions: \c true or \c false.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_all_regions       flashc_lock_all_regions

//! @}


/*! \name Access to General-Purpose Fuses
 */
//! @{

/*! \brief Reads a general-purpose fuse bit.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 *
 * \return The value of the specified general-purpose fuse bit.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_gp_fuse_bit       flashc_read_gp_fuse_bit

/*! \brief Reads a general-purpose fuse bit-field.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 *
 * \return The value of the specified general-purpose fuse bit-field.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_gp_fuse_bitfield  flashc_read_gp_fuse_bitfield

/*! \brief Reads a general-purpose fuse byte.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 *
 * \return The value of the specified general-purpose fuse byte.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_gp_fuse_byte      flashc_read_gp_fuse_byte

/*! \brief Reads all general-purpose fuses.
 *
 * \return The value of all general-purpose fuses as a word.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_all_gp_fuses      flashc_read_all_gp_fuses

/*! \brief Erases a general-purpose fuse bit.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_gp_fuse_bit      flashc_erase_gp_fuse_bit

/*! \brief Erases a general-purpose fuse bit-field.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_gp_fuse_bitfield     flashc_erase_gp_fuse_bitfield

/*! \brief Erases a general-purpose fuse byte.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_gp_fuse_byte         flashc_erase_gp_fuse_byte

/*! \brief Erases all general-purpose fuses.
 *
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_all_gp_fuses       flashc_erase_all_gp_fuses

/*! \brief Writes a general-purpose fuse bit.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 * \param value The value of the specified general-purpose fuse bit.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_gp_fuse_bit        flashc_write_gp_fuse_bit

/*! \brief Writes a general-purpose fuse bit-field.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 * \param value The value of the specified general-purpose fuse bit-field.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_gp_fuse_bitfield     flashc_write_gp_fuse_bitfield

/*! \brief Writes a general-purpose fuse byte.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 * \param value The value of the specified general-purpose fuse byte.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_gp_fuse_byte         flashc_write_gp_fuse_byte

/*! \brief Writes all general-purpose fuses.
 *
 * \param value The value of all general-purpose fuses as a word.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_all_gp_fuses         flashc_write_all_gp_fuses

/*! \brief Sets a general-purpose fuse bit with the appropriate erase and write
 *         operations.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 * \param value The value of the specified general-purpose fuse bit.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_gp_fuse_bit          flashc_set_gp_fuse_bit

/*! \brief Sets a general-purpose fuse bit-field with the appropriate erase and
 *         write operations.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 * \param value The value of the specified general-purpose fuse bit-field.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_gp_fuse_bitfield     flashc_set_gp_fuse_bitfield

/*! \brief Sets a general-purpose fuse byte with the appropriate erase and write
 *         operations.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 * \param value The value of the specified general-purpose fuse byte.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_gp_fuse_byte           flashc_set_gp_fuse_byte

/*! \brief Sets all general-purpose fuses with the appropriate erase and write
 *         operations.
 *
 * \param value The value of all general-purpose fuses as a word.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_all_gp_fuses           flashc_set_all_gp_fuses

//! @}


/*! \name Access to Flash Pages
 */
//! @{

/*! \brief Clears the page buffer.
 *
 * This command resets all bits in the page buffer to one. Write accesses to the
 * page buffer can only change page buffer bits from one to zero.
 *
 * \warning The page buffer is not automatically reset after a page write.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_clear_page_buffer        flashc_clear_page_buffer

/*! \brief Tells whether the page to which the last Quick Page Read or Quick
 *         Page Read User Page command was applied was erased.
 *
 * \return Whether the page to which the last Quick Page Read or Quick Page Read
 *         User Page command was applied was erased.
 */
#define flash_api_is_page_erased           flashc_is_page_erased

/*! \brief Applies the Quick Page Read command to a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \return Whether the specified page is erased.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_quick_page_read          flashc_quick_page_read

/*! \brief Erases a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the command is applied to a page belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_page               flashc_erase_page

/*! \brief Erases all pages within the flash array.
 *
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if at least one region is locked or the
 *          bootloader protection is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_all_pages            flashc_erase_all_pages

/*! \brief Writes a page from the page buffer.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \warning A Lock Error is issued if the command is applied to a page belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \warning The page buffer is not automatically reset after a page write.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 */
#define flash_api_write_page               flashc_write_page

/*! \brief Issues a Quick Page Read User Page command to the FLASHC.
 *
 * \return Whether the User page is erased.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_quick_user_page_read     flashc_quick_user_page_read

/*! \brief Erases the User page.
 *
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_user_page        flashc_erase_user_page

/*! \brief Writes the User page from the page buffer.
 *
 * \warning The page buffer is not automatically reset after a page write.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 */
#define flash_api_write_user_page         flashc_write_user_page

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src source byte.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source byte.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset8                  flashc_memset8

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source half-word.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source half-word.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset16               flashc_memset16

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source word.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source word.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset32               flashc_memset32

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source double-word.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source double-word.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset64               flashc_memset64

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source pattern.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source double-word.
 * \param src_width \a src width in bits: 8, 16, 32 or 64.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset                   flashc_memset

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the source pointed to by \a src.
 *
 * The destination areas that are not within the flash
 * array or the User page are caught by an assert() operation.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Pointer to source data.
 * \param nbytes Number of bytes to copy.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning If copying takes place between areas that overlap, the behavior is
 *          undefined.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memcpy                     flashc_memcpy

//! @}
#endif // __AVR32_ABI_COMPILER__

#elif defined(AVR32_FLASHCDW)

//! Flash Base Address
#define FLASH_API_BASE_ADDRESS      AVR32_FLASHCDW_BASE_ADDRESS

#define FLASH_API_SIZE              AVR32_FLASHCDW_FLASH_SIZE

//! Flash size.
#define FLASH_API_FLASH_SIZE        AVR32_FLASHCDW_FLASH_SIZE

//! Flash page size.
#define FLASH_API_PAGE_SIZE         AVR32_FLASHCDW_PAGE_SIZE

//! Number of flash regions defined by the FLASHCDW.
#define FLASH_API_REGIONS           AVR32_FLASHCDW_REGIONS

//! User page size
#define FLASH_API_USER_PAGE_SIZE    AVR32_FLASHCDW_USER_PAGE_SIZE

//! User page
#define FLASH_API_USER_PAGE         AVR32_FLASHCDW_USER_PAGE

//! User page address
#define FLASH_API_USER_PAGE_ADDRESS AVR32_FLASHCDW_USER_PAGE_ADDRESS

//! Number of GP fuses
#define FLASH_API_GPF_NUM           AVR32_FLASHC_GPF_NUM

/*! \name Flash commands
 */
//! @{
// Note: all commands supported by the flashcdw driver should be abstracted here.
#define FLASH_API_FCMD_CMD_EP       AVR32_FLASHCDW_FCMD_CMD_EP
#define FLASH_API_FCMD_CMD_EUP      AVR32_FLASHCDW_FCMD_CMD_EUP

//! @}

#ifdef __AVR32_ABI_COMPILER__ // Automatically defined when compiling for AVR32, not when assembling.

#include "flashcdw.h"

/*! \name Flash Properties
 */
//! @{

/*! \brief Gets the size of the whole flash array.
 *
 * \return The size of the whole flash array in bytes.
 */
#define flash_api_get_flash_size flashcdw_get_flash_size

/*! \brief Gets the total number of pages in the flash array.
 *
 * \return The total number of pages in the flash array.
 */
#define flash_api_get_page_count flashcdw_get_page_count

/*! \brief Gets the number of pages in each flash region.
 *
 * \return The number of pages in each flash region.
 */
#define flash_api_get_page_count_per_region  flashcdw_get_page_count_per_region

/*! \brief Gets the region number of a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \return The region number of the specified page.
 */
#define flash_api_get_page_region  flashcdw_get_page_region

/*! \brief Gets the number of the first page of a region.
 *
 * \param region The region number: \c 0 to <tt>(AVR32_FLASHC_REGIONS - 1)</tt>.
 *
 * \return The number of the first page of the specified region.
 */
#define flash_api_get_region_first_page_number flashcdw_get_region_first_page_number

//! @}


/*! \name FLASHC Control
 */
//! @{

/*! \brief Gets the number of wait states of flash read accesses.
 *
 * \return The number of wait states of flash read accesses.
 */
#define flash_api_get_wait_state   flashcdw_get_wait_state

/*! \brief Sets the number of wait states of flash read accesses.
 *
 * \param wait_state The number of wait states of flash read accesses: \c 0 to
 *                   \c 1.
 */
#define flash_api_set_wait_state     flashcdw_set_wait_state

/*! \brief Tells whether the Flash Ready interrupt is enabled.
 *
 * \return Whether the Flash Ready interrupt is enabled.
 */
#define flash_api_is_ready_int_enabled flashcdw_is_ready_int_enabled

/*! \brief Enables or disables the Flash Ready interrupt.
 *
 * \param enable Whether to enable the Flash Ready interrupt: \c true or
 *               \c false.
 */
#define flash_api_enable_ready_int     flashcdw_enable_ready_int

/*! \brief Tells whether the Lock Error interrupt is enabled.
 *
 * \return Whether the Lock Error interrupt is enabled.
 */
#define flash_api_is_lock_error_int_enabled  flashcdw_is_lock_error_int_enabled

/*! \brief Enables or disables the Lock Error interrupt.
 *
 * \param enable Whether to enable the Lock Error interrupt: \c true or
 *               \c false.
 */
#define flash_api_enable_lock_error_int  flashcdw_enable_lock_error_int

/*! \brief Tells whether the Programming Error interrupt is enabled.
 *
 * \return Whether the Programming Error interrupt is enabled.
 */
#define flash_api_is_prog_error_int_enabled  flashcdw_is_prog_error_int_enabled

/*! \brief Enables or disables the Programming Error interrupt.
 *
 * \param enable Whether to enable the Programming Error interrupt: \c true or
 *               \c false.
 */
#define flash_api_enable_prog_error_int    flashcdw_enable_prog_error_int

//! @}


/*! \name FLASHC Status
 */
//! @{

/*! \brief Tells whether the FLASHC is ready to run a new command.
 *
 * \return Whether the FLASHC is ready to run a new command.
 */
#define flash_api_is_ready   flashcdw_is_ready

/*! \brief Waits actively until the FLASHC is ready to run a new command.
 *
 * This is the default function assigned to \ref flash_api_wait_until_ready.
 */
#define flash_api_default_wait_until_ready flashcdw_default_wait_until_ready

//! Pointer to the function used by the driver when it needs to wait until the
//! FLASHC is ready to run a new command.
//! The default function is \ref flash_api_default_wait_until_ready.
//! The user may change this pointer to use another implementation.
#define flash_api_wait_until_ready         flashcdw_wait_until_ready

/*! \brief Tells whether a Lock Error has occurred during the last function
 *         called that issued one or more FLASHC commands.
 *
 * \return Whether a Lock Error has occurred during the last function called
 *         that issued one or more FLASHC commands.
 */
#define flash_api_is_lock_error            flashcdw_is_lock_error

/*! \brief Tells whether a Programming Error has occurred during the last
 *         function called that issued one or more FLASHC commands.
 *
 * \return Whether a Programming Error has occurred during the last function
 *         called that issued one or more FLASHC commands.
*/
#define flash_api_is_programming_error     flashcdw_is_programming_error

//! @}


/*! \name FLASHC Command Control
 */
//! @{

/*! \brief Gets the last issued FLASHC command.
 *
 * \return The last issued FLASHC command.
 */
#define flash_api_get_command              flashcdw_get_command

/*! \brief Gets the current FLASHC page number.
 *
 * \return The current FLASHC page number.
 */
#define flash_api_get_page_number          flashcdw_get_page_number

/*! \brief Issues a FLASHC command.
 *
 * \param command The command: \c AVR32_FLASHC_FCMD_CMD_x.
 * \param page_number The page number to apply the command to:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: use this to apply the command to the current page number
 *        or if the command does not apply to any page number;
 *   \arg this argument may have other meanings according to the command. See
 *        the FLASHC chapter of the MCU datasheet.
 *
 * \warning A Lock Error is issued if the command violates the protection
 *          mechanism.
 *
 * \warning A Programming Error is issued if the command is invalid.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_issue_command      flashcdw_issue_command

//! @}


/*! \name FLASHC Global Commands
 */
//! @{

/*! \brief Issues a No Operation command to the FLASHC.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_no_operation         flashcdw_no_operation

/*! \brief Issues an Erase All command to the FLASHC.
 *
 * This command erases all bits in the flash array, the general-purpose fuse
 * bits and the Security bit. The User page is not erased.
 *
 * This command also ensures that all volatile memories, such as register file
 * and RAMs, are erased before the Security bit is erased, i.e. deactivated.
 *
 * \warning A Lock Error is issued if at least one region is locked or the
 *          bootloader protection is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_all            flashcdw_erase_all

//! @}


/*! \name FLASHC Protection Mechanisms
 */
//! @{

/*! \brief Tells whether the Security bit is active.
 *
 * \return Whether the Security bit is active.
 */
#define flash_api_is_security_bit_active   flashcdw_is_security_bit_active

/*! \brief Activates the Security bit.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_activate_security_bit    flashcdw_activate_security_bit

/*! \brief Gets the bootloader protected size.
 *
 * \return The bootloader protected size in bytes.
 */
#define flash_api_get_bootloader_protected_size  flashcdw_get_bootloader_protected_size

/*! \brief Sets the bootloader protected size.
 *
 * \param bootprot_size The wanted bootloader protected size in bytes. If this
 *                      size is not supported, the actual size will be the
 *                      nearest greater available size or the maximal possible
 *                      size if the requested size is too large.
 *
 * \return The actual bootloader protected size in bytes.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_set_bootloader_protected_size  flashcdw_set_bootloader_protected_size

/*! \brief Tells whether external privileged fetch is locked.
 *
 * \return Whether external privileged fetch is locked.
 */
#define flash_api_is_external_privileged_fetch_locked  flashcdw_is_external_privileged_fetch_locked

/*! \brief Locks or unlocks external privileged fetch.
 *
 * \param lock Whether to lock external privileged fetch: \c true or \c false.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_external_privileged_fetch   flashcdw_lock_external_privileged_fetch

/*! \brief Tells whether the region of a page is locked.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \return Whether the region of the specified page is locked.
 */
#define flash_api_is_page_region_locked  flashcdw_is_page_region_locked

/*! \brief Tells whether a region is locked.
 *
 * \param region The region number: \c 0 to <tt>(AVR32_FLASHC_REGIONS - 1)</tt>.
 *
 * \return Whether the specified region is locked.
 */
#define flash_api_is_region_locked       flashcdw_is_region_locked

/*! \brief Locks or unlocks the region of a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 * \param lock Whether to lock the region of the specified page: \c true or
 *             \c false.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_page_region       flashcdw_lock_page_region

/*! \brief Locks or unlocks a region.
 *
 * \param region The region number: \c 0 to <tt>(AVR32_FLASHC_REGIONS - 1)</tt>.
 * \param lock Whether to lock the specified region: \c true or \c false.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_region            flashcdw_lock_region

/*! \brief Locks or unlocks all regions.
 *
 * \param lock Whether to lock the regions: \c true or \c false.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_lock_all_regions       flashcdw_lock_all_regions

//! @}


/*! \name Access to General-Purpose Fuses
 */
//! @{

/*! \brief Reads a general-purpose fuse bit.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 *
 * \return The value of the specified general-purpose fuse bit.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_gp_fuse_bit       flashcdw_read_gp_fuse_bit

/*! \brief Reads a general-purpose fuse bit-field.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 *
 * \return The value of the specified general-purpose fuse bit-field.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_gp_fuse_bitfield  flashcdw_read_gp_fuse_bitfield

/*! \brief Reads a general-purpose fuse byte.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 *
 * \return The value of the specified general-purpose fuse byte.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_gp_fuse_byte      flashcdw_read_gp_fuse_byte

/*! \brief Reads all general-purpose fuses.
 *
 * \return The value of all general-purpose fuses as a word.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_read_all_gp_fuses      flashcdw_read_all_gp_fuses

/*! \brief Erases a general-purpose fuse bit.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_gp_fuse_bit      flashcdw_erase_gp_fuse_bit

/*! \brief Erases a general-purpose fuse bit-field.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_gp_fuse_bitfield     flashcdw_erase_gp_fuse_bitfield

/*! \brief Erases a general-purpose fuse byte.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_gp_fuse_byte         flashcdw_erase_gp_fuse_byte

/*! \brief Erases all general-purpose fuses.
 *
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_erase_all_gp_fuses       flashcdw_erase_all_gp_fuses

/*! \brief Writes a general-purpose fuse bit.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 * \param value The value of the specified general-purpose fuse bit.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_gp_fuse_bit        flashcdw_write_gp_fuse_bit

/*! \brief Writes a general-purpose fuse bit-field.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 * \param value The value of the specified general-purpose fuse bit-field.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_gp_fuse_bitfield     flashcdw_write_gp_fuse_bitfield

/*! \brief Writes a general-purpose fuse byte.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 * \param value The value of the specified general-purpose fuse byte.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_gp_fuse_byte         flashcdw_write_gp_fuse_byte

/*! \brief Writes all general-purpose fuses.
 *
 * \param value The value of all general-purpose fuses as a word.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_write_all_gp_fuses         flashcdw_write_all_gp_fuses

/*! \brief Sets a general-purpose fuse bit with the appropriate erase and write
 *         operations.
 *
 * \param gp_fuse_bit The general-purpose fuse bit: \c 0 to \c 63.
 * \param value The value of the specified general-purpose fuse bit.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_gp_fuse_bit          flashcdw_set_gp_fuse_bit

/*! \brief Sets a general-purpose fuse bit-field with the appropriate erase and
 *         write operations.
 *
 * \param pos The bit-position of the general-purpose fuse bit-field: \c 0 to
 *            \c 63.
 * \param width The bit-width of the general-purpose fuse bit-field: \c 0 to
 *              \c 64.
 * \param value The value of the specified general-purpose fuse bit-field.
 *
 * \warning A Lock Error is issued if the Security bit is active and the command
 *          is applied to BOOTPROT or EPFL fuses.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_gp_fuse_bitfield     flashcdw_set_gp_fuse_bitfield

/*! \brief Sets a general-purpose fuse byte with the appropriate erase and write
 *         operations.
 *
 * \param gp_fuse_byte The general-purpose fuse byte: \c 0 to \c 7.
 * \param value The value of the specified general-purpose fuse byte.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_gp_fuse_byte           flashcdw_set_gp_fuse_byte

/*! \brief Sets all general-purpose fuses with the appropriate erase and write
 *         operations.
 *
 * \param value The value of all general-purpose fuses as a word.
 *
 * \warning A Lock Error is issued if the Security bit is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note The actual number of general-purpose fuse bits implemented by hardware
 *       is given by \c AVR32_FLASHC_GPF_NUM. The other bits among the 64 are
 *       fixed at 1 by hardware.
 */
#define flash_api_set_all_gp_fuses           flashcdw_set_all_gp_fuses

//! @}


/*! \name Access to Flash Pages
 */
//! @{

/*! \brief Clears the page buffer.
 *
 * This command resets all bits in the page buffer to one. Write accesses to the
 * page buffer can only change page buffer bits from one to zero.
 *
 * \warning The page buffer is not automatically reset after a page write.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_clear_page_buffer        flashcdw_clear_page_buffer

/*! \brief Tells whether the page to which the last Quick Page Read or Quick
 *         Page Read User Page command was applied was erased.
 *
 * \return Whether the page to which the last Quick Page Read or Quick Page Read
 *         User Page command was applied was erased.
 */
#define flash_api_is_page_erased           flashcdw_is_page_erased

/*! \brief Applies the Quick Page Read command to a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \return Whether the specified page is erased.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_quick_page_read          flashcdw_quick_page_read

/*! \brief Erases a page.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if the command is applied to a page belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_page               flashcdw_erase_page

/*! \brief Erases all pages within the flash array.
 *
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \warning A Lock Error is issued if at least one region is locked or the
 *          bootloader protection is active.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_all_pages            flashcdw_erase_all_pages

/*! \brief Writes a page from the page buffer.
 *
 * \param page_number The page number:
 *   \arg \c 0 to <tt>(flash_api_get_page_count() - 1)</tt>: a page number within
 *        the flash array;
 *   \arg <tt>< 0</tt>: the current page number.
 *
 * \warning A Lock Error is issued if the command is applied to a page belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \warning The page buffer is not automatically reset after a page write.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 */
#define flash_api_write_page               flashcdw_write_page

/*! \brief Issues a Quick Page Read User Page command to the FLASHC.
 *
 * \return Whether the User page is erased.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_quick_user_page_read     flashcdw_quick_user_page_read

/*! \brief Erases the User page.
 *
 * \param check Whether to check erase: \c true or \c false.
 *
 * \return Whether the erase succeeded or always \c true if erase check was not
 *         requested.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note An erase operation can only set bits.
 */
#define flash_api_erase_user_page        flashcdw_erase_user_page

/*! \brief Writes the User page from the page buffer.
 *
 * \warning The page buffer is not automatically reset after a page write.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 *
 * \note A write operation can only clear bits.
 */
#define flash_api_write_user_page         flashcdw_write_user_page

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src source byte.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source byte.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset8                  flashcdw_memset8

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source half-word.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source half-word.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset16               flashcdw_memset16

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source word.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source word.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset32               flashcdw_memset32

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source double-word.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source double-word.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset64               flashcdw_memset64

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the repeated \a src big-endian source pattern.
 *
 * The destination areas that are not within the flash array or the User page
 * are ignored.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Source double-word.
 * \param src_width \a src width in bits: 8, 16, 32 or 64.
 * \param nbytes Number of bytes to set.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memset                   flashcdw_memset

/*! \brief Copies \a nbytes bytes to the flash destination pointed to by \a dst
 *         from the source pointed to by \a src.
 *
 * The destination areas that are not within the flash
 * array or the User page are caught by an assert() operation.
 *
 * All pointer and size alignments are supported.
 *
 * \param dst Pointer to flash destination.
 * \param src Pointer to source data.
 * \param nbytes Number of bytes to copy.
 * \param erase Whether to erase before writing: \c true or \c false.
 *
 * \return The value of \a dst.
 *
 * \warning If copying takes place between areas that overlap, the behavior is
 *          undefined.
 *
 * \warning This function may be called with \a erase set to \c false only if
 *          the destination consists only of erased words, i.e. this function
 *          can not be used to write only one bit of a previously written word.
 *          E.g., if \c 0x00000001 then \c 0xFFFFFFFE are written to a word, the
 *          resulting value in flash may be different from \c 0x00000000.
 *
 * \warning A Lock Error is issued if the command is applied to pages belonging
 *          to a locked region or to the bootloader protected area.
 *
 * \note The FLASHC error status returned by \ref flash_api_is_lock_error and
 *       \ref flash_api_is_programming_error is updated.
 */
#define flash_api_memcpy                     flashcdw_memcpy

//! @}
#endif // __AVR32_ABI_COMPILER__

#else
#error "Unsupported flash controller"
#endif

#endif  // _FLASH_API_H_
