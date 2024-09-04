/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * Based on adc_mcux_adc16.c, which is:
 * Copyright (c) 2017-2018, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_s32k144_adc12
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(adc_mcux_adc12);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"
#include "adc_driver.h"
#include <stdlib.h>

struct s32k144_adc12_config {
	ADC_Type *base;
	uint8_t instance;
	adc_converter_config_t adc_conv_config;
	adc_compare_config_t adc_Hw_Comp_config;
	adc_average_config_t adc_Hw_Avg_config;
	adc_chan_config_t adc_Chn_config;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
};

struct mcux_adc12_config {
	ADC_Type *base;
	adc_input_clock_t clock_src;
	adc_clk_divide_t clock_div;
	adc_voltage_reference_t ref_src;
	uint32_t sample_clk_count;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
};

struct mcux_adc12_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	uint8_t channel_id;
};

static int mcux_adc12_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	/*Dummy function, Checks to be implemented later.*/
	return 0;
}

static int mcux_adc12_start_read(const struct device *dev,
				 const struct adc_sequence *sequence)
{
	const struct mcux_adc12_config *config = dev->config;
	struct mcux_adc12_data *data = dev->data;
	adc_average_t mode;
	adc_resolution_t resolution;
	ADC_Type *base = config->base;
	int error;
	uint32_t tmp32;

	switch (sequence->resolution) {
	case 8:
		resolution = ADC_RESOLUTION_8BIT;
		break;
	case 10:
		resolution = ADC_RESOLUTION_10BIT;
		break;
	case 12:
		resolution = ADC_RESOLUTION_12BIT;
		break;
	default:

		LOG_ERR("Unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	tmp32 = base->CFG1 & ~(ADC_CFG1_MODE_MASK);
	tmp32 |= ADC_CFG1_MODE(resolution);
	base->CFG1 = tmp32;

	switch (sequence->oversampling) {

	case 2:
		mode = ADC_AVERAGE_4;
		break;
	case 3:
		mode = ADC_AVERAGE_8;
		break;
	case 4:
		mode = ADC_AVERAGE_16;
		break;
	case 5:
		mode = ADC_AVERAGE_32;
		break;
	default:
		LOG_ERR("Unsupported oversampling value %d",
			sequence->oversampling);
		return -ENOTSUP;
	}
	ADC12_SetHardwareAverage(config->base, mode);

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);
	error = adc_context_wait_for_completion(&data->ctx);

	return error;
}

static int mcux_adc12_read_async(const struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct mcux_adc12_data *data = dev->data;
	int error;
	adc_context_lock(&data->ctx, async ? true : false, async);
	error = mcux_adc12_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#include "S32K144_PDB.h"
#define PDB0 IP_PDB0

static int mcux_adc12_read(const struct device *dev,
			   const struct adc_sequence *sequence)
{
	const struct s32k144_adc12_config *config = dev->config;

	ADC_DRV_ConfigChan(config->instance, 0U, &config->adc_Chn_config);
	return 0;
	//Move to Non interrupt based function
	uint16_t result;
	uint16_t *buf = sequence->buffer;

	ADC_DRV_GetChanResult(config->instance, 0U, &result);
	*buf = result;
}

static void mcux_adc12_start_channel(const struct device *dev)
{
	const struct s32k144_adc12_config *config = dev->config;

	ADC_DRV_ConfigChan(config->instance, 0U, &config->adc_Chn_config);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct mcux_adc12_data *data =
		CONTAINER_OF(ctx, struct mcux_adc12_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	mcux_adc12_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct mcux_adc12_data *data =
		CONTAINER_OF(ctx, struct mcux_adc12_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}


static void mcux_adc12_isr(const struct device *dev)
{
	const struct s32k144_adc12_config *config = dev->config;
	uint16_t result;

	ADC_DRV_GetChanResult(config->instance, 0U, &result);
	ADC_DRV_GetChanResult(config->instance, 0U, &result);

	return;
}


#define ADC_NXP_S32_INSTANCE_CHECK(indx, n)	\
	((DT_INST_REG_ADDR(n) == IP_ADC##indx##_BASE) ? indx : 0)
#define ADC_NXP_S32_GET_INSTANCE(n)		\
	LISTIFY(__DEBRACKET ADC_INSTANCE_COUNT, ADC_NXP_S32_INSTANCE_CHECK, (|), n)


static int mcux_adc12_init(const struct device *dev)
{
	const struct s32k144_adc12_config *config = dev->config;
	struct mcux_adc12_data *data = dev->data;
	ADC_Type *base = config->base;
	int err;
	struct adc_converter_config_t *adc_conv_config = &(config->adc_conv_config);

	ADC_DRV_ConfigConverter(config->instance, adc_conv_config);
	ADC_DRV_AutoCalibration(config->instance);

	config->irq_config_func(dev);
	return 0;
}

static const struct adc_driver_api mcux_adc12_driver_api = {
	.channel_setup = mcux_adc12_channel_setup,
	.read = mcux_adc12_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = mcux_adc12_read_async,
#endif
};

#define ASSERT_WITHIN_RANGE(val, min, max, str) \
	BUILD_ASSERT(val >= min && val <= max, str)
#define ASSERT_ADC12_CLK_DIV_VALID(val, str) \
	BUILD_ASSERT(val == 1 || val == 2 || val == 4 || val == 8, str)
#define TO_ADC12_CLOCK_SRC(val) _DO_CONCAT(ADC_CLK_ALT_, val)
#define TO_ADC12_CLOCK_DIV(val) _DO_CONCAT(ADC_CLK_DIVIDE_, val)

#define ADC12_REF_SRC(n)						\
	COND_CODE_1(DT_INST_PROP(0, alternate_voltage_reference),	\
				 (ADC_VOLTAGEREF_VALT),	\
				 (ADC_VOLTAGEREF_VREF))

#define XSTR(x) STR(x)
#define STR(x) #x



#define ACD12_MCUX_INIT(n)						\
	static void mcux_adc12_config_func_##n(const struct device *dev); \
									\
									\
	static const struct s32k144_adc12_config s32k144_adc12_config_##n = {	\
		.base = (ADC_Type *)DT_INST_REG_ADDR(n),		\
		.instance = ADC_NXP_S32_GET_INSTANCE(n),			\
		.adc_conv_config.inputClock = \
				ADC_CLK_ALT_1,\
		.adc_conv_config.clockDivide =							\
			ADC_CLK_DIVIDE_4,	\
		.adc_conv_config.voltageRef = ADC_VOLTAGEREF_VREF,			\
		.adc_conv_config.sampleTime = 255U,						\
		.adc_conv_config.resolution = ADC_RESOLUTION_12BIT,		\
		.adc_conv_config.trigger = ADC_TRIGGER_SOFTWARE,		\
		.adc_conv_config.pretriggerSel = ADC_PRETRIGGER_SEL_SW,\
		.adc_conv_config.triggerSel = ADC_TRIGGER_SEL_PDB,		\
		.adc_conv_config.dmaEnable = false,						\
		.adc_conv_config.continuousConvEnable = false,			\
		.adc_conv_config.supplyMonitoringEnable = false,			\
		.adc_Chn_config.interruptEnable = true,\
		.adc_Chn_config.channel = ADC_INPUTCHAN_EXT12,\
		.irq_config_func = mcux_adc12_config_func_##n,			\
	};								\
									\
	static struct mcux_adc12_data mcux_adc12_data_##n = {		\
		ADC_CONTEXT_INIT_TIMER(mcux_adc12_data_##n, ctx),	\
		ADC_CONTEXT_INIT_LOCK(mcux_adc12_data_##n, ctx),	\
		ADC_CONTEXT_INIT_SYNC(mcux_adc12_data_##n, ctx),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &mcux_adc12_init,			\
			    NULL, &mcux_adc12_data_##n,			\
			    &s32k144_adc12_config_##n, POST_KERNEL,	\
			    CONFIG_ADC_INIT_PRIORITY,			\
			    &mcux_adc12_driver_api);			\
									\
	static void mcux_adc12_config_func_##n(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    0, mcux_adc12_isr,	\
			    DEVICE_DT_INST_GET(n), 0);			\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}


DT_INST_FOREACH_STATUS_OKAY(ACD12_MCUX_INIT)


