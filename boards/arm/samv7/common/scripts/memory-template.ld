/****************************************************************************
 * boards/arm/samv7/common/scripts/memory-template.ld
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

/* The SAMV7 can have up to 2048Kb of FLASH beginning at address 0x0040:0000
 * and 384Kb of SRAM beginining at 0x2040:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0400:0000 address range.
 *
 * The user space partition will be spanned with a single region of size
 * 2**n bytes.  The alignment of the user-space region must be the same.
 * As a consequence, as the user-space increases in size, the alignment
 * requirement also increases.  The sizes below give the largest possible
 * user address spaces (but leave far too much for the OS).
 *
 * The solution to this wasted memory is to (1) use more than one region to
 * span the user spaces, or (2) poke holes in a larger region to trim it
 * to fit better.
 *
 * A detailed memory map for the 384KB SRAM region is as follows:
 *
 *   0x2040 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------ ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x2040 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                 determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                 adjustments for alignment.  Typical is 1KB.
 *   ------ ----   Padded to 4KB
 *   0x2042 0000:  User .data region.  Size is variable.
 *   ------- ----  User .bss region  Size is variable.
 *   0x2044 0000:  Beginning of kernel heap.  Size determined by
 *                 CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------ ----   Beginning of user heap.  Can vary with other settings.
 *   0x2046 0000:  End+1 of mappable internal SRAM
 */

#include <nuttx/config.h>

#define FLASH_START_ADDR        0x00400000
#define DTCM_START_ADDR         0x20000000
#define SRAM_START_ADDR         0x20400000

#ifdef CONFIG_ARMV7M_DTCM
#  define FSRAM_START_ADDR      DTCM_START_ADDR
#  define FSRAM_REGION_SIZE     128k
#else
#  define FSRAM_START_ADDR      SRAM_START_ADDR
#  define FSRAM_REGION_SIZE     CONFIG_ARCH_CHIP_SAMV7_MEM_RAM
#endif

/* Define Flat Memory Model */

#ifdef CONFIG_SAMV7_FORMAT_MCUBOOT
#  define MCUBOOT_HEADER_SIZE   CONFIG_SAMV7_MCUBOOT_HEADER_SIZE
#  ifdef CONFIG_BOARDCTL_BOOT_IMAGE
#    define FLASH_OFFSET        0
#    define FLAT_REGION_SIZE    CONFIG_SAMV7_OTA_PRIMARY_SLOT_OFFSET
#    define MCUBOOT_USER_ALIGN  0
#  else
#    define FLASH_OFFSET        (CONFIG_SAMV7_OTA_PRIMARY_SLOT_OFFSET + MCUBOOT_HEADER_SIZE)
#    define FLAT_REGION_SIZE    (CONFIG_SAMV7_OTA_SLOT_SIZE - MCUBOOT_HEADER_SIZE)
#    define MCUBOOT_USER_ALIGN  (MCUBOOT_HEADER_SIZE / 2)
#  endif
#else
#  define FLASH_OFFSET          0
#  define FLAT_REGION_SIZE      CONFIG_ARCH_CHIP_SAMV7_MEM_FLASH
#    define MCUBOOT_USER_ALIGN  0
#endif

#define FLAT_REGION_OFFSET      (FLASH_START_ADDR + FLASH_OFFSET)

/* Split Flat Memory into Kernel and User Spaces */

#define FLASH_REGION_SIZE       (FLAT_REGION_SIZE / 2)
#define KERNEL_REGION_OFFSET    (FLASH_START_ADDR + FLASH_OFFSET)
#define USER_REGION_OFFSET      (KERNEL_REGION_OFFSET + FLASH_REGION_SIZE + MCUBOOT_USER_ALIGN)

#ifdef CONFIG_SAMV7_MEM_RAM_384
#  define SRAM_REGION_SIZE      128k
#  define KSRAM_START_ADDR      SRAM_START_ADDR
#  define KSRAM_REGION_SIZE     SRAM_REGION_SIZE
#  define USRAM_START_ADDR      (KSRAM_START_ADDR + KSRAM_REGION_SIZE)
#  define USRAM_REGION_SIZE     SRAM_REGION_SIZE
#  define XSRAM_START_ADDR      (USRAM_START_ADDR + USRAM_REGION_SIZE)
#  define XSRAM_REGION_SIZE     SRAM_REGION_SIZE
#else
#  define SRAM_REGION_SIZE      96k
#  define KSRAM_START_ADDR      SRAM_START_ADDR
#  define KSRAM_REGION_SIZE     SRAM_REGION_SIZE
#  define USRAM_START_ADDR      (KSRAM_START_ADDR + KSRAM_REGION_SIZE)
#  define USRAM_REGION_SIZE     SRAM_REGION_SIZE
#  define XSRAM_START_ADDR      (USRAM_START_ADDR + USRAM_REGION_SIZE)
#  define XSRAM_REGION_SIZE     64k
#endif

MEMORY
{
  flash  (rx) : ORIGIN = FLAT_REGION_OFFSET,   LENGTH = FLAT_REGION_SIZE
  kflash (rx) : ORIGIN = KERNEL_REGION_OFFSET, LENGTH = FLASH_REGION_SIZE
  uflash (rx) : ORIGIN = USER_REGION_OFFSET,   LENGTH = (FLASH_REGION_SIZE - MCUBOOT_USER_ALIGN)

  sram  (rwx) : ORIGIN = FSRAM_START_ADDR,     LENGTH = FSRAM_REGION_SIZE
  ksram (rwx) : ORIGIN = KSRAM_START_ADDR,     LENGTH = KSRAM_REGION_SIZE
  usram (rwx) : ORIGIN = USRAM_START_ADDR,     LENGTH = USRAM_REGION_SIZE
  xsram (rwx) : ORIGIN = XSRAM_START_ADDR,     LENGTH = XSRAM_REGION_SIZE
}
