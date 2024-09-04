/*
 * Copyright (c) 2016 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/flash.h>
#include <errno.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/sys/barrier.h>
#include "flash_priv.h"

#include "fsl_common.h"

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_mcux);

// #include "ftfc_flash_driver.h"

#if DT_NODE_HAS_STATUS(DT_INST(0, nxp_kinetis_ftfc), okay)
#define DT_DRV_COMPAT nxp_kinetis_ftfc
#include "ftfc_flash_driver.h"
// #elif !DT_DRV_COMPAT nxp_kinetis_ftfc
// #error No matching compatible for soc_flash_mcux.c
#endif


#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

struct flash_priv {
	flash_ssd_config_t config;
	/*
	 * HACK: flash write protection is managed in software.
	 */
	struct k_sem write_lock;
	uint32_t pflash_block_base;
};

static const struct flash_parameters flash_mcux_parameters = {
#if DT_NODE_HAS_PROP(SOC_NV_FLASH_NODE, write_block_size)
	.write_block_size = DT_PROP(SOC_NV_FLASH_NODE, write_block_size),
#else
	.write_block_size = FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE,
#endif
	.erase_value = 0xff,
};

/*
 * Interrupt vectors could be executed from flash hence the need for locking.
 * The underlying MCUX driver takes care of copying the functions to SRAM.
 *
 * For more information, see the application note below on Read-While-Write
 * http://cache.freescale.com/files/32bit/doc/app_note/AN4695.pdf
 *
 */

static int flash_mcux_erase(const struct device *dev, off_t offset,
			    size_t len)
{
	struct flash_priv *priv = dev->data;
	uint32_t addr;
	status_t rc;
	unsigned int key;

	if (k_sem_take(&priv->write_lock, K_FOREVER)) {
		return -EACCES;
	}

	addr = offset + priv->pflash_block_base;

	key = irq_lock();
	rc =FLASH_DRV_EraseSector(&priv->config, addr, len);
	irq_unlock(key);

	k_sem_give(&priv->write_lock);

	return (rc == kStatus_Success) ? 0 : -EINVAL;
}


/*
 * @brief Read a flash memory area.
 *
 * @param dev Device struct
 * @param offset The address's offset
 * @param data The buffer to store or read the value
 * @param length The size of the buffer
 * @return 	0 on success,
 * 			-EIO for erroneous area
 */
static int flash_mcux_read(const struct device *dev, off_t offset,
				void *data, size_t len)
{
	struct flash_priv *priv = dev->data;
	uint32_t addr = offset;
	status_t rc = 0;

	rc = FLASH_IsFlashAreaReadable(&priv->config, addr, len);

	if (!rc) {
		memcpy(data, (void *) addr, len);
	}
#ifdef CONFIG_CHECK_BEFORE_READING
	else if (rc == -ENODATA) {
		/* Erased area, return dummy data as an erased page. */
		memset(data, 0xFF, len);
		rc = 0;
	}
#endif
	return rc;
}

static int flash_mcux_write(const struct device *dev, off_t offset,
				const void *data, size_t len)
{
	struct flash_priv *priv = dev->data;
	uint32_t addr;
	status_t rc;
	unsigned int key;

	if (k_sem_take(&priv->write_lock, K_FOREVER)) {
		return -EACCES;
	}

	addr = offset;

	key = irq_lock();
	rc = FLASH_DRV_Program(&priv->config, addr, len, (uint8_t *) data);
	irq_unlock(key);

	k_sem_give(&priv->write_lock);

	return (rc == kStatus_Success) ? 0 : -EINVAL;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) /
				DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

static void flash_mcux_pages_layout(const struct device *dev,
				    const struct flash_pages_layout **layout,
				    size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_mcux_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_mcux_parameters;
}

static struct flash_priv flash_data;

static const struct flash_driver_api flash_mcux_api = {
	.erase = flash_mcux_erase,
	.write = flash_mcux_write,
	.read = flash_mcux_read,
	.get_parameters = flash_mcux_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mcux_pages_layout,
#endif
};

static int flash_mcux_init(const struct device *dev)
{
	struct flash_priv *priv = dev->data;
	status_t rc;

	const flash_user_config_t Flash_InitConfig0 =
	{
	    .PFlashBase = 0x0U,
	    .PFlashSize = 0x80000U,
	    .DFlashBase = 0x10000000U,
	    .EERAMBase = 0x14000000U,
	    .CallBack = NULL_CALLBACK
	};

	k_sem_init(&priv->write_lock, 1, 1);

	rc = FLASH_DRV_Init(&Flash_InitConfig0,&flash_data.config);

	return (rc == kStatus_Success) ? 0 : -EIO;
}

DEVICE_DT_INST_DEFINE(0, flash_mcux_init, NULL,
			&flash_data, NULL, POST_KERNEL,
			CONFIG_FLASH_INIT_PRIORITY, &flash_mcux_api);
