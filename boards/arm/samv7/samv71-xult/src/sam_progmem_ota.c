/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_progmem_ota.c
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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>
#ifdef CONFIG_BCH
#include <nuttx/drivers/drivers.h>
#endif

#include "sam_progmem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x)                (sizeof((x)) / sizeof((x)[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ota_partition_s
{
  uint32_t    offset;          /* Partition offset from the beginning of MTD */
  uint32_t    size;            /* Partition size in bytes */
  const char *devpath;         /* Partition device path */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ota_partition_s g_ota_partition_table[] =
{
  {
    .offset  = 0x0,
    .size    = 0x20000,
    .devpath = "/dev/mtdboot0"
  },
  {
    .offset  = 0x20000,
    .size    = 0xe0000,
    .devpath = "/dev/mtdfota0"
  },
  {
    .offset  = 0x100000,
    .size    = 0xe0000,
    .devpath = "/dev/mtdfota1"
  },
  {
    .offset  = 0x1e0000,
    .size    = 0x20000,
    .devpath = "/dev/mtdswap0"
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spiflash_alloc_mtdpart
 *
 * Description:
 *   Allocate an MTD partition from the ESP32 SPI Flash.
 *
 * Input Parameters:
 *   mtd_offset - MTD Partition offset from the base address in SPI Flash.
 *   mtd_size   - Size for the MTD partition.
 *
 * Returned Value:
 *   ESP32 SPI Flash MTD data pointer if success or NULL if fail
 *
 ****************************************************************************/

static FAR struct mtd_dev_s *sam_progmem_init_ota_alloc_partition(
        FAR struct mtd_dev_s *mtd, const struct ota_partition_s *part)
{
  FAR struct mtd_dev_s *mtd_part;
  uint32_t blocks;
  uint32_t startblock;
  uint32_t size;

  ASSERT((part->offset + part->size) <= up_progmem_size());
  ASSERT((part->offset % up_progmem_pagesize(0)) == 0);
  ASSERT((part->size % up_progmem_pagesize(0)) == 0);

  finfo("\tMTD offset = 0x%x\n", part->offset);
  finfo("\tMTD size = 0x%x\n", part->size);

  startblock = up_progmem_getpage(part->offset);
  if (startblock < 0)
    {
      return NULL;
    }

  blocks = part->size / up_progmem_pagesize(0);

  return mtd_partition(mtd, startblock, blocks);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sam_progmem_init_ota_partitions(FAR struct mtd_dev_s *mtd)
{
  FAR struct mtd_dev_s *mtd_part;
#ifdef CONFIG_BCH
  char blockdev[18];
#endif
  int ret = OK;

  for (int i = 0; i < ARRAYSIZE(g_ota_partition_table); ++i)
    {
      const struct ota_partition_s *part = &g_ota_partition_table[i];
      mtd_part = sam_progmem_init_ota_alloc_partition(mtd, part);

      if (!mtd_part)
        {
          ferr("ERROR: create MTD OTA partition %s", part->devpath);
          continue;
        }

      ret = ftl_initialize(i, mtd_part);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#ifdef CONFIG_BCH
      snprintf(blockdev, 18, "/dev/mtdblock%d", i);

      ret = bchdev_register(blockdev, part->devpath, false);
      if (ret < 0)
        {
          ferr("ERROR: bchdev_register %s failed: %d\n", part->devpath, ret);
          return ret;
        }
#endif
    }

  return ret;
}