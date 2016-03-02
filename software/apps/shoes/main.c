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
 #include "app_timer.h"

#include "board.h"
#include "adxl362.h"

#include "simple_ble.h"
#include "simple_adv.h"



#define DEVICE_NAME "SHOES!"

#define SHOES_SERVICE 0x14
#define UMICH_COMPANY_IDENTIFIER 0x02E0

#define LED0 13
#define LED1 8

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    .platform_id       = 0xda,              // used as 4th octect in device BLE address
    .device_id         = DEVICE_ID_DEFAULT,
    .adv_name          = DEVICE_NAME,
    .adv_interval      = MSEC_TO_UNITS(10000, UNIT_0_625_MS), // NOT ACTUALLY USED. WE OVERRIDE SIMPLEBLE FUNCTIONS
    .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS)
};

typedef struct {
    uint8_t*  p_data;
    uint16_t  data_len;
} data_t;

typedef struct {
    uint8_t seq;
    uint8_t action;
} __attribute__((packed)) shoe_pkt_t;


#define ACCELEROMETER_INTERRUPT_PIN 11

static nrf_drv_spi_t _spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

app_gpiote_user_id_t gpiote_user_acc;

APP_TIMER_DEF(app_timer);


static uint8_t mdata[1+sizeof(shoe_pkt_t)] = {SHOES_SERVICE};


static shoe_pkt_t me = {0, 0xb6};

// Last seq number we got from a shoe
static uint8_t last_seq_number = 255;


void adv_init (shoe_pkt_t* shoe);


// Get field from advertisement
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata) {
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


void ble_error(uint32_t error_code) {
    led_on(LED0);
}



void ble_evt_adv_report (ble_evt_t* p_ble_evt) {
    uint32_t err;
    // led_toggle(LED1);

    // Setup some pointers and data structures. Wow can this get verbose.
    ble_gap_evt_adv_report_t* adv = &p_ble_evt->evt.gap_evt.params.adv_report;
    data_t adv_data;
    adv_data.p_data = adv->data;
    adv_data.data_len = adv->dlen;


    // Look for name
    data_t device_name;
    err = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &device_name);
    if (err != NRF_SUCCESS) {
        // This is definitely not a packet we are looking for
        return;
    }
    // Make sure its the correct name
    if (strncmp((char*) device_name.p_data, DEVICE_NAME, device_name.data_len) != 0) return;

    // Look for manufacturer data
    data_t manuf_data;
    err = adv_report_parse(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, &adv_data, &manuf_data);
    if (err != NRF_SUCCESS) return;

    // Length of manufacturer data should be:
    //  company id (2) + service (1) + sizeof(shoe_pkt_t)
    if (manuf_data.data_len != 3+sizeof(shoe_pkt_t)) return;

    // This should be our manufacturer ID and service
    uint16_t manufacturer = manuf_data.p_data[0] | (((uint16_t) manuf_data.p_data[1]) << 8);
    uint8_t service = manuf_data.p_data[2];
    if (manufacturer != UMICH_COMPANY_IDENTIFIER || service != SHOES_SERVICE) return;

    // At this point we can be pretty sure we have the correct packet
    shoe_pkt_t* shoe = (shoe_pkt_t*) (manuf_data.p_data+3);

    if (shoe->seq != last_seq_number) {
        last_seq_number = shoe->seq;
        // led_toggle(LED1);
    }




    // adv->rssi

}


static void app_timer_handler (void* p_context) {
    advertising_stop();
}

static void acc_interrupt_handler (uint32_t pins_l2h, uint32_t pins_h2l) {
    if (pins_h2l & (1 << ACCELEROMETER_INTERRUPT_PIN)) {
        // High to low transition
        led_toggle(LED1);

        me.seq++;
        adv_init(&me);
        advertising_start();
        app_timer_start(app_timer, APP_TIMER_TICKS(4000, 0), NULL);
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

void adv_init (shoe_pkt_t* shoe) {
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    // Common
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    // Put name in main packet or in the scan response
    advdata.name_type = BLE_ADVDATA_FULL_NAME;

    // Handle manufacturer data
    ble_advdata_manuf_data_t mandata;
    mandata.company_identifier = UMICH_COMPANY_IDENTIFIER;
    mandata.data.p_data = mdata;
    mandata.data.size   = 1 + sizeof(shoe_pkt_t);
    memcpy(mdata+1, shoe, sizeof(shoe_pkt_t));
    advdata.p_manuf_specific_data = &mandata;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // // Start the advertisement
    // advertising_start();
}

// Copied to allow me to set the advertising channels used
ble_gap_adv_params_t m_adv_params;
void advertising_init(void) {
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = MSEC_TO_UNITS(10000, UNIT_0_625_MS);
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    // Only use channel 39
    m_adv_params.channel_mask.ch_37_off = 1;
    m_adv_params.channel_mask.ch_38_off = 1;
    m_adv_params.channel_mask.ch_39_off = 0;
}

void advertising_start () {
    uint32_t err_code = sd_ble_gap_adv_start(&m_adv_params);
    if (err_code != NRF_ERROR_INVALID_STATE) {
        // ignore Invalid State responses. Occurs when start is called twice
        APP_ERROR_CHECK(err_code);
    }
}


static const ble_gap_scan_params_t m_scan_param = {
    .active      = 0,              // Active scanning not set.
    .selective   = 0,              // Selective scanning not set.
    .p_whitelist = NULL,           // No whitelist provided.
    .interval    = 0x00A0,
    .window      = 0x00A0,
    .timeout     = 0x0000          // No timeout.
};

static void scan_start () {
    ret_code_t err_code;

    err_code = sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
    }
}



int main(void) {
    uint32_t err_code;

    led_init(LED0);
    led_off(LED0);
    led_init(LED1);
    led_off(LED1);

    // Setup BLE
    simple_ble_init(&ble_config);

    // nrf_power_dcdc_mode_t = NRF_POWER_DCDC_ENABLE;
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    // init timers in case we need to
    APP_TIMER_INIT(0, 4, false);
    err_code = app_timer_create(&app_timer, APP_TIMER_MODE_SINGLE_SHOT, app_timer_handler);
    APP_ERROR_CHECK(err_code);



    // Advertise because why not
    // simple_adv_only_name();


    // And scan at the same time
    scan_start();

    accelerometer_init();



    while (1) {
        power_manage();
    }
}
