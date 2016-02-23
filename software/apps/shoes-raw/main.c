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

#include "led.h"


#define LED0 25

#define DEVICE_NAME "Shoes!"                                /**< Name of device. Will be included in the advertising data. */

#define TIMESLOT_LENGTH_US 10000
#define TIMESLOT_DISTANCE_US 20000
static btle_cmd_param_le_write_scan_parameters_t scan_param = {
  BTLE_SCAN_TYPE_ACTIVE,          /* Active scanning. SCAN_REQ packets may be sent */
  TIMESLOT_DISTANCE_US,           /* Time from controller starts its last scan until it begins the next scan */
  TIMESLOT_LENGTH_US,             /* Duration of the scan */
  BTLE_ADDR_TYPE_PUBLIC,          /* Use public address type */
  BTLE_SCAN_FILTER_ACCEPT_ANY     /* Accept anyone (whitelist unsupported for now) */
};

static btle_cmd_param_le_write_scan_enable_t scan_enable = {
  BTLE_SCAN_MODE_ENABLE,              /* Enable scanner */
  BTLE_SCAN_DUPLICATE_FILTER_DISABLE  /* Do not filter duplicates */
};
void timeslot_sys_event_handler(uint32_t evt);
bool sw_interrupt = false;


static app_timer_id_t                   m_battery_timer_id;                          /**< Battery measurement timer. */


void ble_error(uint32_t error_code) {
    led_on(LED0);
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
    app_error_handler(0x5599, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}





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
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

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


// Function for the Power manager.
static void power_manage (void) {
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


void timeslot_sys_event_handler(uint32_t evt) {
    uint32_t err_code;

    switch (evt) {
      case NRF_EVT_RADIO_SESSION_IDLE:
      case NRF_EVT_RADIO_BLOCKED:
        /* Request a new timeslot */
        err_code= btle_scan_enable_set(scan_enable);
        APP_ERROR_CHECK(err_code);
        break;

      case NRF_EVT_RADIO_SESSION_CLOSED:
        break;

      case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        ASSERT(false);
        break;

      case NRF_EVT_RADIO_CANCELED:
        err_code = btle_scan_enable_set(scan_enable);
        APP_ERROR_CHECK(err_code);
        break;

      default:
        break;
    }
}


int main(void)
{
    bool erase_bonds;
    uint32_t err_code;




    // Initialize.
    led_init(LED0);
    timers_init();

    ble_stack_init();
    // gap_params_init();
    // advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
    // services_init();
    // conn_params_init();



    // Start execution.
    err_code = sd_nvic_SetPriority(SWI1_IRQn, NRF_APP_PRIORITY_LOW);
    ASSERT(err_code == NRF_SUCCESS);

    err_code = sd_nvic_EnableIRQ(SWI1_IRQn);
    ASSERT(err_code == NRF_SUCCESS);


    err_code = btle_scan_init (SWI1_IRQn);
    ASSERT(err_code == BTLE_STATUS_CODE_SUCCESS);

    err_code = btle_scan_param_set (scan_param);
    ASSERT (err_code == BTLE_STATUS_CODE_SUCCESS);

    err_code = btle_scan_enable_set (scan_enable);
    ASSERT (err_code == BTLE_STATUS_CODE_SUCCESS);

    // Enter main loop.
    while (1) {
        if (sw_interrupt) {
            nrf_report_t report;
            while (btle_scan_ev_get(&report) != BTLE_STATUS_CODE_COMMAND_DISALLOWED) {
                // __LOG("Type: %X, Addr: %X:%X:%X:%X:%X:%X, RSSI: %i",
                //     report.event.params.le_advertising_report_event.event_type,
                //     report.event.params.le_advertising_report_event.address[5],
                //     report.event.params.le_advertising_report_event.address[4],
                //     report.event.params.le_advertising_report_event.address[3],
                //     report.event.params.le_advertising_report_event.address[2],
                //     report.event.params.le_advertising_report_event.address[1],
                //     report.event.params.le_advertising_report_event.address[0],
                //     report.event.params.le_advertising_report_event.rssi);
            }
            sw_interrupt = false;
        }
    }
}

// Timeslot event interrupt
// Triggered whenever an event is ready to be pulled
void SWI1_IRQHandler(void) {
  sw_interrupt = true;
}

