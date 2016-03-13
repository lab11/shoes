#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf_drv_spi.h"
#include "app_gpiote.h"
#include "nrf_delay.h"

#include "radio.h"

#include "led.h"
#include "adxl362.h"
#include "board.h"


/******************************************************************************/
/* Configuration
 ******************************************************************************/

#define DEVICE_NAME "Shoes!"

// Setup how we want the timeslots to work (taking the radio from the
// softdevice).
#define TIMESLOT_LENGTH_US   100000
// #define TIMESLOT_LENGTH_US   50000
#define TIMESLOT_DISTANCE_US 100000
#define TIMESLOT_TIMEOUT_US 200000

#define FLOOD_DURATION APP_TIMER_TICKS(5000, 0)


// Types of packets in the Shoes! protocol
#define SHOES_PKT_TYPE_FLOOD 0x01  // Start or continue a flood


// Pin defines
#define LED0 8
#define LED1 26
#define LED2 29
#define LED3 9
#define LED4 10

#define ACCELEROMETER_INTERRUPT_PIN 11
#define BUTTON_INTERRUPT_PIN 16


/******************************************************************************/
/* Global state
 ******************************************************************************/

// SPI bus for accelerometer
static nrf_drv_spi_t _spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

// Struct for getting GPIO interrupts
static app_gpiote_user_id_t gpiote_user_acc;

// Structs for calling the timeslot APIs
static nrf_radio_signal_callback_return_param_t m_signal_callback_return_param;
static nrf_radio_request_t m_timeslot_req_earliest = {
    NRF_RADIO_REQ_TYPE_EARLIEST,
    .params.earliest = {
        NRF_RADIO_HFCLK_CFG_NO_GUARANTEE,
        NRF_RADIO_PRIORITY_NORMAL,
        TIMESLOT_LENGTH_US,
        TIMESLOT_TIMEOUT_US
    }
};

// Timer for delaying packet re-transmissions
APP_TIMER_DEF(timer_flood_retransmission);

// Timer for going back to idle after a flood has started
APP_TIMER_DEF(timer_flood_end);

// Keep track of unique flood id for this node
static uint8_t _my_flood_counter = 0;

// How many floods we are currently seeing
static uint8_t _current_flood_count = 0;

// Advertisement packet structure
#define RX_BUF_SIZE 128
static uint8_t m_rx_buf[RX_BUF_SIZE];

typedef struct {
    uint8_t length;
    uint8_t type;
    uint8_t man_id1;
    uint8_t man_id2;
    uint8_t service_id;
    uint8_t packet_type;
    uint8_t data[4];
} __attribute__((packed)) shoes_manuf_data_t;

typedef struct {
    uint8_t type_and_options;
    uint8_t length;
    uint8_t s1;
    uint8_t src_addr[6];
    uint8_t flags[3];
    shoes_manuf_data_t manuf;
    uint8_t name[8];
} __attribute__((packed)) advertisement_t;


advertisement_t advertisement = {
    .type_and_options = 0x02,  // ADV_NONCONN_IND
    // .type_and_options = 0x00,  // ADV_NONCONN_IND
    .length = 27,
    .s1 = 0,
    .src_addr = {0x00, 0x00, 0x00, 0xe5, 0x98, 0xc0},
    .flags = {0x02, 0x01, 0x06},
    .manuf = {
        .length = 9,
        .type = 0xff,
        .man_id1 = 0xe0,
        .man_id2 = 0x02,
        .service_id = 0x14,
        .packet_type = 0,
        .data = {0, 0, 0, 0}},
    .name = {7, 0x09, 0x53, 0x48, 0x4f, 0x45, 0x53, 0x21}
};


/*******************************************************************************
 * Function signatures
 ******************************************************************************/

void timeslot_sys_event_handler (uint32_t evt);


/*******************************************************************************
 * Error callbacks
 ******************************************************************************/

void ble_error (uint32_t error_code) {
    led_on(LED0);
}

void assert_nrf_callback (uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(0x5599, line_num, p_file_name);
}


/*******************************************************************************
 * System functions
 ******************************************************************************/

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
// static void sleep_mode_enter(void)
// {
//     uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//     APP_ERROR_CHECK(err_code);

//     // Prepare wakeup buttons.
//     err_code = bsp_btn_ble_sleep_mode_prepare();
//     APP_ERROR_CHECK(err_code);

//     // Go to system-off mode (this function will not return; wakeup will cause a reset).
//     err_code = sd_power_system_off();
//     APP_ERROR_CHECK(err_code);
// }



static void on_ble_evt(ble_evt_t * p_ble_evt) {
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
        case BLE_GAP_EVT_DISCONNECTED:
        case BLE_GAP_EVT_TIMEOUT:
        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            break;

        default:
            break;
    }
}

static void ble_evt_dispatch (ble_evt_t * p_ble_evt) {
    on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch (uint32_t sys_evt) {
    timeslot_sys_event_handler(sys_evt);
}

// Function for the Power manager.
static void power_manage (void) {
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void timeslot_sys_event_handler (uint32_t evt) {
    uint32_t err_code;

    switch (evt) {
      case NRF_EVT_RADIO_SESSION_IDLE:
      case NRF_EVT_RADIO_BLOCKED:
        // Request a new timeslot
        err_code = sd_radio_request(&m_timeslot_req_earliest);
        APP_ERROR_CHECK(err_code);
        break;

      case NRF_EVT_RADIO_SESSION_CLOSED:
        break;

      case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        // ASSERT(false);
        break;

      case NRF_EVT_RADIO_CANCELED:
        err_code = sd_radio_request(&m_timeslot_req_earliest);
        APP_ERROR_CHECK(err_code);
        break;

      default:
        break;
    }
}



/*******************************************************************************
 * Control the radio
 ******************************************************************************/

static void continue_scan () {
    radio_disable();

    memset((void*) m_rx_buf, '\0', RX_BUF_SIZE);
    radio_buffer_configure(&m_rx_buf[0]);
    radio_rx_prepare(true);
    radio_rssi_enable();
}


static void start_scan () {
    NVIC_EnableIRQ(TIMER0_IRQn);

    radio_init(39); // set channel to only use 39
    radio_rx_timeout_init();

    continue_scan();
}


static void send_advertisement () {
    radio_disable();
    radio_init(39);
    radio_buffer_configure((uint8_t*) &advertisement);
    radio_tx_prepare();
}





/*******************************************************************************
 * Keep track of a little bit of history
 ******************************************************************************/

// Circular buffers to keep track of recent floods.
// We keep more than one because it's conceivable we get a packet about
// a flood that we have already completed.
#define KNOWN_FLOOD_BUFFER_LEN 10
uint32_t known_flood_initiators[KNOWN_FLOOD_BUFFER_LEN] = {0};
uint8_t known_flood_initiators_ids[KNOWN_FLOOD_BUFFER_LEN] = {0};
uint8_t known_flood_head = 0;

static bool flood_id_is_new (uint32_t id, uint8_t flood_id) {
    uint8_t i;

    for (i=0; i<KNOWN_FLOOD_BUFFER_LEN; i++) {
        if (known_flood_initiators[i] == id &&
            known_flood_initiators_ids[i] == flood_id) {
            // If we find the given flood initiator and flood id in the
            // stored values then we have seen this before
            return false;
        }
    }
    return true;
}

static void flood_id_record (uint32_t id, uint8_t flood_id) {
    uint8_t i;
    bool updated = false;

    for (i=0; i<KNOWN_FLOOD_BUFFER_LEN; i++) {
        if (known_flood_initiators[i] == id) {
            // Hey! we are already in the buffer store. Just update
            // the flood id
            known_flood_initiators_ids[i] = flood_id;
            updated = true;
            break;
        }
    }

    if (!updated) {
        // We couldn't find this id in our buffer, so we must add it.
        known_flood_initiators[known_flood_head] = id;
        known_flood_initiators_ids[known_flood_head] = flood_id;
        known_flood_head = (known_flood_head + 1) % KNOWN_FLOOD_BUFFER_LEN;
    }
}


/*******************************************************************************
 * Flooding functions
 ******************************************************************************/

static void led_iterate () {
    static uint8_t state = 0;

    state += 1;
    if (state > 0x0F) {
        state = 0;
    }

    if (state & 0x1) led_off(LED1); else led_on(LED1);
    if (state & 0x2) led_off(LED2); else led_on(LED2);
    if (state & 0x4) led_off(LED4); else led_on(LED4);
    if (state & 0x8) led_off(LED3); else led_on(LED3);
}

static void flood_leds (uint8_t flood_count) {
    if (flood_count == 1) {
        led_off(LED2);
        led_off(LED4);
        led_on(LED1);
        led_on(LED3);
    } else if (flood_count == 2) {
        led_off(LED1);
        led_off(LED3);
        led_on(LED2);
        led_on(LED4);
    } else if (flood_count == 3) {
        led_off(LED1);
        led_off(LED3);
        led_off(LED2);
        led_off(LED4);
    }
}

// Convert RSSI to a delay in app timer ticks.
static uint32_t rssi_to_ticks (int8_t rssi) {
    if (rssi <= -80) return APP_TIMER_TICKS(5, 0); // 5 ms
    if (rssi >= 0) return APP_TIMER_TICKS(100, 0); // 100 ms
    return APP_TIMER_TICKS(((99*((int16_t)rssi)) / 80) + 100, 0);
}

static void start_flood () {
    // Check how many floods we are a part of.
    // If we are already at the limit, we can't start another
    if (_current_flood_count < 3) {

        // Mark that we are part of a new flood
        _current_flood_count++;

        // Set our LED to start the flood
        flood_leds(_current_flood_count);

        // Set a timer to turn off the LEDs after the flood is over
        app_timer_stop(timer_flood_end);
        app_timer_start(timer_flood_end, FLOOD_DURATION, NULL);

        // Mark this as a flood
        advertisement.manuf.packet_type = SHOES_PKT_TYPE_FLOOD;
        _my_flood_counter++;
        advertisement.manuf.data[0] = _my_flood_counter;
        // Also include that we are the ones who started it
        advertisement.manuf.data[1] = advertisement.src_addr[2];
        advertisement.manuf.data[2] = advertisement.src_addr[1];
        advertisement.manuf.data[3] = advertisement.src_addr[0];

        // Now spread the flood
        send_advertisement();
    }
}

static void join_flood (int8_t rssi, uint32_t initiator_id, uint8_t flood_id) {
    // Check how many floods we are a part of.
    // If we are already at the limit, we can't start another
    if (_current_flood_count < 3) {

        // Mark that we are part of a new flood
        _current_flood_count++;

        // Set our LED to start the flood
        flood_leds(_current_flood_count);

        // Set a timer to turn off the LEDs after the flood is over
        app_timer_stop(timer_flood_end);
        app_timer_start(timer_flood_end, FLOOD_DURATION, NULL);

        // Mark this as a flood
        advertisement.manuf.packet_type = SHOES_PKT_TYPE_FLOOD;
        _my_flood_counter++;
        advertisement.manuf.data[0] = flood_id;
        // Also include the node that started this particular flood
        advertisement.manuf.data[1] = (initiator_id >> 16) & 0xFF;
        advertisement.manuf.data[2] = (initiator_id >> 8)  & 0xFF;
        advertisement.manuf.data[3] = (initiator_id >> 0)  & 0xFF;

        // Now spread the flood based on RSSI. Lower RSSI means quicker
        // retransmit time.
        app_timer_start(timer_flood_retransmission, rssi_to_ticks(rssi), NULL);
    }
}


/*******************************************************************************
 * Callbacks
 ******************************************************************************/


void rx_callback (bool crc_valid) {
    if (crc_valid) {
        led_toggle(LED0);

        int8_t rssi = -1 * (int8_t) radio_rssi_get();

        // Parse incoming packet as one of ours. Now normally a lot
        // of the following wouldn't be cool because of buffer overruns,
        // but we know the length of m_rx_buf, so if we overrun the actual
        // packet things won't break.
        advertisement_t* inadv = (advertisement_t*) m_rx_buf;

        // Check to see if this packet is us
        if (inadv->manuf.type == 0xff &&
            inadv->manuf.man_id1 == 0xe0 &&
            inadv->manuf.man_id2 == 0x02 &&
            // Check name
            memcmp(inadv->name+2, "SHOES!", 6) == 0) {

            // Decide what type of packet this is
            switch (inadv->manuf.packet_type) {

                // Flood packet from another node
                case SHOES_PKT_TYPE_FLOOD: {

                    // Decide if this flood was started by us. If so,
                    // we definitely want to ignore it.
                    if (inadv->manuf.data[3] != advertisement.src_addr[0] ||
                        inadv->manuf.data[3] != advertisement.src_addr[0] ||
                        inadv->manuf.data[3] != advertisement.src_addr[0]) {

                        // Get the per-flood-initiator unique id for this flood
                        uint8_t flood_id = inadv->manuf.data[0];

                        // Extract the lower 3 bytes of the ID from the node
                        // that started the flood
                        uint32_t id = (((uint32_t) inadv->manuf.data[3]) << 0) |
                                      (((uint32_t) inadv->manuf.data[2]) << 8) |
                                      (((uint32_t) inadv->manuf.data[1]) << 16);

                        // Decide if we have seen this flood before.
                        // If we haven't we can respond properly. If we have,
                        // then we can just ignore this packet.
                        if (flood_id_is_new(id, flood_id)) {

                            // Mark that we got this packet
                            flood_id_record(id, flood_id);

                            // Now join the flood.
                            join_flood(rssi, id, flood_id);

                        }
                    }

                    break;
                }

                default:
                    break;
            }

        }

    }
    continue_scan();
}

// This is called after a packet is transmitted. We want to
// go right back to listening.
void tx_callback () {
    start_scan();
}

// Called as a part of the timeslot API from the softdevice.
nrf_radio_signal_callback_return_param_t* radio_cb (uint8_t sig) {
    switch (sig) {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            // Setup a timer so we know when our timeslot is up
            NRF_TIMER0->TASKS_CLEAR = 1;
            NRF_TIMER0->EVENTS_COMPARE[0] = 0;
            NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
            NRF_TIMER0->CC[0] = TIMESLOT_LENGTH_US - 500;

            // Scan baby scan!
            start_scan();

            m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            radio_event_cb();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            // Our timeslot is about to expire. Ask for a new one.
            // I tried using the extension mechanism, I just got a bunch of
            // repeated callback weirdness.

            m_signal_callback_return_param.params.request.p_next = &m_timeslot_req_earliest;
            m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;

        default:
            break;
    }
    return &m_signal_callback_return_param;
}



// This timer callback occurs when we should transmit a packet to
// keep propagating a flood a different node started.
static void timer_flood_retransmission_callback (void* context) {
    send_advertisement();
}

// This timer callback happens to effectively end a flood
// at this node by turning off all LEDs
static void timer_flood_end_callback (void* context) {
    // Turn off all LEDs if flood has ended
    led_on(LED1);
    led_on(LED2);
    led_on(LED3);
    led_on(LED4);

    // Reset our flood state
    _current_flood_count = 0;
}

// Interrupt handler for button and accelerometer
static void interrupt_handler (uint32_t pins_l2h, uint32_t pins_h2l) {
    // led_toggle(LED0);
    if (pins_h2l & (1 << ACCELEROMETER_INTERRUPT_PIN)) {
        // High to low transition
        // led_toggle(LED0);
        // led_iterate();

        // me.seq++;
        // adv_init(&me);
        // advertising_start();
        // app_timer_start(app_timer, APP_TIMER_TICKS(4000, 0), NULL);
        // send_advertisement();

        start_flood();
    }

    if (pins_l2h & (1 << BUTTON_INTERRUPT_PIN)) {
        // Button press
        // led_toggle(LED0);
        // led_iterate();
        start_flood();
    }
}

/*******************************************************************************
 * Init functions
 ******************************************************************************/

static void ble_stack_init () {
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = {
        .source        = NRF_CLOCK_LF_SRC_RC,
        .rc_ctiv       = 4, // bradjc: I mostly made these up based on docs. May be not great.
        .rc_temp_ctiv  = 0,
        .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_100_PPM};

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    // Need these #defines. C is the worst.
    #define CENTRAL_LINK_COUNT    1
    #define PERIPHERAL_LINK_COUNT 1
    err_code = softdevice_enable_get_default_config(1, // central link count
                                                    1, // peripheral link count
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static void accelerometer_init () {
    uint32_t err;


    // Configure the accelerometer interrupt

    // Need one user: accelerometer
    APP_GPIOTE_INIT(2);

    // Register the accelerometer
    err = app_gpiote_user_register(&gpiote_user_acc,
        (1<<ACCELEROMETER_INTERRUPT_PIN) | (1<<BUTTON_INTERRUPT_PIN),   // Which pins we want the interrupt for low to high
        (1<<ACCELEROMETER_INTERRUPT_PIN) | (1<<BUTTON_INTERRUPT_PIN),   // Which pins we want the interrupt for high to low
        interrupt_handler);

    if (err != NRF_SUCCESS) {
        led_on(LED0);
    }



    // Configure the accel hardware
    adxl362_accelerometer_init(&_spi, adxl362_NOISE_NORMAL, false, false, false);

    uint16_t act_thresh = 0x222;
    // uint16_t act_thresh = 0x0020;
    adxl362_set_activity_threshold(act_thresh);
    uint16_t inact_thresh = 0x00c6;
    adxl362_set_inactivity_threshold(inact_thresh);

    // uint8_t a_time = 4;
    uint8_t a_time = 1;
    adxl362_set_activity_time(a_time);
    uint8_t ia_time = 30;
    adxl362_set_inactivity_time(ia_time);

    adxl362_config_interrupt_mode(adxl362_INTERRUPT_LOOP, true , true);
    // adxl362_config_interrupt_mode(adxl362_INTERRUPT_LOOP, false , false);
    adxl362_activity_inactivity_interrupt_enable();

    adxl362_interrupt_map_t intmap_2;

    intmap_2.DATA_READY = 0;
    intmap_2.FIFO_READY = 0;
    intmap_2.FIFO_WATERMARK = 0;
    intmap_2.FIFO_OVERRUN = 0;
    intmap_2.ACT     = 0;
    intmap_2.INACT   = 0;
    intmap_2.AWAKE   = 1;
    intmap_2.INT_LOW = 1;
    adxl362_config_INTMAP(&intmap_2, true);



    adxl362_autosleep();

    adxl362_measurement_mode();

    adxl362_read_status_reg();



    // Enable the interrupt!
    err = app_gpiote_user_enable(gpiote_user_acc);
    if (err != NRF_SUCCESS) {
        led_on(LED0);
    }
}


/******************************************************************************/
/* Main
 ******************************************************************************/

int main () {
    uint32_t err_code;

    // Initialize.
    led_init(LED0);
    led_off(LED0);

    // These are active high...go figure
    led_init(LED1);
    led_on(LED1);
    led_init(LED2);
    led_on(LED2);
    led_init(LED3);
    led_on(LED3);
    led_init(LED4);
    led_on(LED4);

    // Essentially initialize soft device
    ble_stack_init();

    // Call this to get magically lower power
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    // Copy BLE address into our outgoing advertisement struct
    memcpy(advertisement.src_addr, (uint8_t*) BLEADDR_FLASH_LOCATION, 6);

    // Need to init timer system
    APP_TIMER_INIT(0, // prescaler
                   4, // op queue size
                   false);
    // Create the one-shot timers
    app_timer_create(&timer_flood_retransmission, APP_TIMER_MODE_SINGLE_SHOT, timer_flood_retransmission_callback);
    app_timer_create(&timer_flood_end, APP_TIMER_MODE_SINGLE_SHOT, timer_flood_end_callback);

    // Need to setup our accelerometer
    accelerometer_init();

    // Create a session for doing timeslots
    err_code = sd_radio_session_open(radio_cb);
    APP_ERROR_CHECK(err_code);

    // Request a timeslot
    err_code = sd_radio_request(&m_timeslot_req_earliest);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    while (1) {
        power_manage();
    }
}
