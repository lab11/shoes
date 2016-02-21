/*
 * Send an advertisement periodically while scanning for advertisements.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "ble_debug_assert_handler.h"
#include "led.h"
#include "nrf_drv_spi.h"
#include "app_gpiote.h"

#include "board.h"
#include "adxl362.h"

#include "simple_ble.h"
#include "simple_adv.h"


#define LED0 13
#define LED1 25

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    .platform_id       = 0xda,              // used as 4th octect in device BLE address
    .device_id         = DEVICE_ID_DEFAULT,
    .adv_name          = "SHOES!",
    .adv_interval      = MSEC_TO_UNITS(500, UNIT_0_625_MS),
    .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS)
};


#define ACCELEROMETER_INTERRUPT_PIN 5

static nrf_drv_spi_t _spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

app_gpiote_user_id_t gpiote_user_acc;


void ble_error(uint32_t error_code) {
    led_on(LED0);
}

void ble_evt_adv_report (ble_evt_t* p_ble_evt) {

    ble_gap_evt_adv_report_t* adv = &p_ble_evt->evt.gap_evt.params.adv_report;


    if (adv->rssi > -60) {
        led_toggle(LED1);
    }
}

static void acc_interrupt_handler (uint32_t pins_l2h, uint32_t pins_h2l) {
    if (pins_h2l & (1 << ACCELEROMETER_INTERRUPT_PIN)) {
        // High to low transition
        led_toggle(LED0);
    }
}


void accelerometer_init () {
    // Configure the accel hardware
    adxl362_accelerometer_init(&_spi, adxl362_NOISE_NORMAL, true, false, false);

    uint16_t act_thresh = 0x0222;
    adxl362_set_activity_threshold(act_thresh);
    uint16_t inact_thresh = 0x0096;
    adxl362_set_inactivity_threshold(inact_thresh);

    uint8_t a_time = 4;
    adxl362_set_activity_time(a_time);
    uint8_t ia_time = 30;
    adxl362_set_inactivity_time(ia_time);

    adxl362_interrupt_map_t intmap_2;

    intmap_2.DATA_READY = 0;
    intmap_2.FIFO_READY = 0;
    intmap_2.FIFO_WATERMARK = 0;
    intmap_2.FIFO_OVERRUN = 0;
    intmap_2.ACT = 0;
    intmap_2.INACT = 0;
    intmap_2.AWAKE = 1;
    intmap_2.INT_LOW = 1;
    adxl362_config_INTMAP(&intmap_2, false);

    adxl362_config_interrupt_mode(adxl362_INTERRUPT_LOOP, true , true);
    adxl362_activity_inactivity_interrupt_enable();

    adxl362_read_status_reg();


    // Configure the accel interrupt

    // Need one user: accelerometer
    APP_GPIOTE_INIT(1);

    // Register the accelerometer
    app_gpiote_user_register(&gpiote_user_acc,
                             1<<ACCELEROMETER_INTERRUPT_PIN,   // Which pins we want the interrupt for low to high
                             1<<ACCELEROMETER_INTERRUPT_PIN,   // Which pins we want the interrupt for high to low
                             acc_interrupt_handler);

    // Enable the interrupt!
    app_gpiote_user_enable(gpiote_user_acc);
}



int main(void) {
    led_init(LED0);
    led_off(LED0);
    led_init(LED1);
    led_off(LED1);

    // Setup BLE
    simple_ble_init(&ble_config);

    // Advertise because why not
    simple_adv_only_name();

    // And scan at the same time
    simple_ble_scan_start();

    accelerometer_init();

    while (1) {
        power_manage();
    }
}
