#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_adc.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_tps.h"
#include "ble_ias.h"
#include "ble_lls.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_ias_c.h"
#include "app_util.h"
#include "btle.h"
#include "nrf_assert.h"
#include "nrf_scan.h"

 #include "nrf_delay.h"

#include "led.h"


#define LED0 25

#define DEVICE_NAME "Shoes!"                                /**< Name of device. Will be included in the advertising data. */

#define TIMESLOT_LENGTH_US   100000
// #define TIMESLOT_LENGTH_US   50000


#define TIMESLOT_DISTANCE_US 100000
#define TIMESLOT_TIMEOUT_US 200000

// static btle_cmd_param_le_write_scan_parameters_t scan_param = {
//   BTLE_SCAN_TYPE_PASSIVE,         /* Active scanning. SCAN_REQ packets may be sent */
//   TIMESLOT_DISTANCE_US,           /* Time from controller starts its last scan until it begins the next scan */
//   TIMESLOT_LENGTH_US,             /* Duration of the scan */
//   BTLE_ADDR_TYPE_PUBLIC,          /* Use public address type */
//   BTLE_SCAN_FILTER_ACCEPT_ANY     /* Accept anyone (whitelist unsupported for now) */
// };

// static btle_cmd_param_le_write_scan_enable_t scan_enable = {
//   BTLE_SCAN_MODE_ENABLE,              /* Enable scanner */
//   BTLE_SCAN_DUPLICATE_FILTER_DISABLE  /* Do not filter duplicates */
// };


static nrf_radio_signal_callback_return_param_t m_signal_callback_return_param;
static nrf_radio_request_t m_timeslot_req_earliest = {
    NRF_RADIO_REQ_TYPE_EARLIEST,
    .params.earliest = {
        NRF_RADIO_HFCLK_CFG_DEFAULT,
        NRF_RADIO_PRIORITY_NORMAL,
        TIMESLOT_LENGTH_US,
        TIMESLOT_TIMEOUT_US
    }
};


void timeslot_sys_event_handler(uint32_t evt);

bool sw_interrupt = false;


static app_timer_id_t m_battery_timer_id; /**< Battery measurement timer. */


void ble_error(uint32_t error_code) {
    led_on(LED0);
    // while(1);

        for(int i=0; i<100; i++) {
    led_toggle(25);
    nrf_delay_us(100);
  }


}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{

    for(int i=0; i<25; i++) {
    led_toggle(25);
    nrf_delay_us(250);
  }


    app_error_handler(0x5599, line_num, p_file_name);
}


// *@brief Function for handling Service errors.
//  *
//  * @details A pointer to this function will be passed to each service which may need to inform the
//  *          application about an error.
//  *
//  * @param[in] nrf_error   Error code containing information about what went wrong.

// static void service_error_handler(uint32_t nrf_error)
// {

//     for(int i=0; i<100; i++) {
//     led_toggle(25);
//     nrf_delay_us(100);
//   }


//     APP_ERROR_HANDLER(nrf_error);
// }





/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {

}




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





/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt) {
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            break;

        case BLE_GAP_EVT_TIMEOUT:
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in] sys_evt  system event.
 */
static void on_sys_evt(uint32_t sys_evt) {
    switch(sys_evt) {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch (uint32_t sys_evt) {
    timeslot_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


static void ble_stack_init(void) {
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION, NULL);

    ble_enable_params_t ble_enable_params;
    // Need these #defines. C is the worst.
    #define CENTRAL_LINK_COUNT    2
    #define PERIPHERAL_LINK_COUNT 1
    err_code = softdevice_enable_get_default_config(2, // central link count
                                                    1, // peripheral link count
                                                    &ble_enable_params);
  ////  APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
  /////  APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
 ////   APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
  ////  APP_ERROR_CHECK(err_code);
}


// Function for the Power manager.
static void power_manage (void) {
    uint32_t err_code = sd_app_evt_wait();
   //// APP_ERROR_CHECK(err_code);
}


void timeslot_sys_event_handler(uint32_t evt) {
    uint32_t err_code;

    switch (evt) {
      case NRF_EVT_RADIO_SESSION_IDLE:
      case NRF_EVT_RADIO_BLOCKED:
        /* Request a new timeslot */
        // err_code= btle_scan_enable_set(scan_enable);
      // led_off(25);
        err_code = sd_radio_request(&m_timeslot_req_earliest);
        // APP_ERROR_CHECK(err_code);
        break;

      case NRF_EVT_RADIO_SESSION_CLOSED:
        break;

      case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        // ASSERT(false);
        break;

      case NRF_EVT_RADIO_CANCELED:
      // led_off(25);
        // err_code = btle_scan_enable_set(scan_enable);
        err_code = sd_radio_request(&m_timeslot_req_earliest);
        // APP_ERROR_CHECK(err_code);
        break;

      default:
        break;
    }
}


#define RX_BUF_SIZE 128
static uint8_t m_rx_buf[RX_BUF_SIZE];
static uint8_t m_tx_buf[] =
{
  0xC3,                               // BLE Header (PDU_TYPE: SCAN_REQ, TXadd: 1 (random address), RXadd: 1 (random address)
  0x0C,                               // Length of payload: 12
  0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
  0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, // InitAddr LSByte first
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // AdvAddr LSByte first
};



void continue_scan () {
    radio_disable();

    memset((void *) m_rx_buf, '\0', RX_BUF_SIZE);
    radio_buffer_configure(&m_rx_buf[0]);
    radio_rx_prepare(true);
    radio_rssi_enable();
}


void start_scan () {
    NVIC_EnableIRQ(TIMER0_IRQn);


    radio_init(39); // set channel to only use 39
    radio_rx_timeout_init();


    continue_scan();
}


void send_advertisement () {
    memset ((void *) m_rx_buf, '\0', RX_BUF_SIZE);
    radio_buffer_configure (&m_rx_buf[0]);
    radio_rx_prepare (false);
    radio_rssi_enable ();
    radio_rx_timeout_enable ();
}



void rx_callback (bool crc_valid) {
    led_toggle(25);

    continue_scan();
}


void tx_callback () {

}



nrf_radio_signal_callback_return_param_t* radio_cb (uint8_t sig) {
    switch (sig) {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
          /* TIMER0 setup */
          NRF_TIMER0->TASKS_CLEAR = 1;
          NRF_TIMER0->EVENTS_COMPARE[0] = 0;
          NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
          NRF_TIMER0->CC[0] = TIMESLOT_LENGTH_US - 500;

            // led_on(25);
            // ll_scan_start();


            start_scan();

            m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            radio_event_cb();
            break;

        // case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
        //   /* Check the timeslot cleanup counter */
        //   if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
        //   {
        //     ll_scan_stop ();
        //     NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        //     NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
        //     NVIC_DisableIRQ(TIMER0_IRQn);

        //     m_signal_callback_return_param.params.request.p_next = &m_timeslot_req_normal;
        //     m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        //   }

        //   /* Check the timeout counter */
        //   if (NRF_TIMER0->EVENTS_COMPARE[1] != 0)
        //   {
        //     NRF_TIMER0->EVENTS_COMPARE[1] = 0;
        //     NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;

        //     radio_timeout_cb ();
        //   }
        //   break;

        // case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
        //   break;

        // case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
        //   break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
        // led_toggle(25);
            //Timer interrupt - do graceful shutdown - attempt to increase timeslot length
            // m_signal_callback_return_param.params.extend.length_us = TIMESLOT_LENGTH_US;
            // m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;

            m_signal_callback_return_param.params.request.p_next = &m_timeslot_req_earliest;
            m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;

        // case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
        //     //Extension succeeded, reset timer(configurations still valid since slot length is the same)
        //     NRF_TIMER0->TASKS_CLEAR = 1;
        //     m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        //     led_toggle(25);
        //     break;

        // case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
        //     //Extension failed, attempt schedule new timeslot
        //     // configure_next_event_earliest();
        //     m_signal_callback_return_param.params.request.p_next = &m_timeslot_req_earliest;
        //     m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        //     // led_toggle(25);
        //     break;
        default:
            break;
    }
    return &m_signal_callback_return_param;
}





int main(void)
{
    bool erase_bonds;
    uint32_t err_code;




    // Initialize.
    led_init(LED0);
    led_off(LED0);


    timers_init();

    ble_stack_init();


  //   for(int i=0; i<100; i++) {
  //   led_toggle(25);
  //   nrf_delay_us(100);
  // }


    // gap_params_init();
    // advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
    // services_init();
    // conn_params_init();



    // Start execution.
    // err_code = sd_nvic_SetPriority(SWI1_IRQn, NRF_APP_PRIORITY_LOW);
    // ASSERT(err_code == NRF_SUCCESS);

    // err_code = sd_nvic_EnableIRQ(SWI1_IRQn);
    // ASSERT(err_code == NRF_SUCCESS);



    // Create a session for doing timeslots
    err_code = sd_radio_session_open(radio_cb);
 ////   APP_ERROR_CHECK(err_code);

    // Configure scanning
    // ll_scan_init();
    // ll_scan_config(BTLE_SCAN_TYPE_PASSIVE,
    //                BTLE_ADDR_TYPE_PUBLIC,
    //                BTLE_SCAN_FILTER_ACCEPT_ANY);







    // Request a timeslot
    err_code = sd_radio_request(&m_timeslot_req_earliest);
  ////  APP_ERROR_CHECK(err_code);



    // err_code = btle_scan_init (SWI1_IRQn);
    // ASSERT(err_code == BTLE_STATUS_CODE_SUCCESS);

    // err_code = btle_scan_param_set (scan_param);
    // ASSERT (err_code == BTLE_STATUS_CODE_SUCCESS);

    // err_code = btle_scan_enable_set (scan_enable);
    // ASSERT (err_code == BTLE_STATUS_CODE_SUCCESS);

    // Enter main loop.
    while (1) {
        // if (sw_interrupt) {
        //     nrf_report_t report;
        //     while (btle_scan_ev_get(&report) != BTLE_STATUS_CODE_COMMAND_DISALLOWED) {
        //         // __LOG("Type: %X, Addr: %X:%X:%X:%X:%X:%X, RSSI: %i",
        //         //     report.event.params.le_advertising_report_event.event_type,
        //         //     report.event.params.le_advertising_report_event.address[5],
        //         //     report.event.params.le_advertising_report_event.address[4],
        //         //     report.event.params.le_advertising_report_event.address[3],
        //         //     report.event.params.le_advertising_report_event.address[2],
        //         //     report.event.params.le_advertising_report_event.address[1],
        //         //     report.event.params.le_advertising_report_event.address[0],
        //         //     report.event.params.le_advertising_report_event.rssi);
        //     }
        //     sw_interrupt = false;
        // }
    }
}

// Timeslot event interrupt
// Triggered whenever an event is ready to be pulled
// void SWI1_IRQHandler(void) {
//   sw_interrupt = true;
// }

