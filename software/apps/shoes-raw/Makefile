PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
APPLICATION_SRCS += softdevice_handler.c
# APPLICATION_SRCS += ble_advdata.c
# APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += app_error.c
APPLICATION_SRCS += app_gpiote.c
# APPLICATION_SRCS += ble_srv_common.c
# APPLICATION_SRCS += ble_conn_state.c
# APPLICATION_SRCS += sdk_mapped_flags.c
# APPLICATION_SRCS += app_util_platform.c
APPLICATION_SRCS += nrf_drv_common.c
# APPLICATION_SRCS += nrf_delay.c
APPLICATION_SRCS += led.c

APPLICATION_SRCS += nrf_drv_spi.c
APPLICATION_SRCS += nrf_drv_gpiote.c

APPLICATION_SRCS += adxl362.c

SOFTDEVICE_MODEL = s130

RAM_KB = 32
SDK_VERSION = 11
CFLAGS += -DDEBUG=1

LIBRARY_PATHS += . ../../include btle_hci/scanner/include
SOURCE_PATHS += ../../src btle_hci/scanner

NRF_BASE_PATH ?= ../../nrf5x-base
include $(NRF_BASE_PATH)/make/Makefile
