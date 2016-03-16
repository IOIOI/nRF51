/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_gls_main main.c
 * @{
 * @ingroup ble_sdk_app_gls
 * @brief Glucose Meter service Sample Application
 *
 * This file contains the source code for a sample application using the Glucose Meter service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <cstdint>
#include <cstring>
#include <string>

extern "C" {
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "app_button.h"
#include "pstorage.h"
#include "app_uart.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
}

#include "advertising.h"

#if BUTTONS_NUMBER < 2
#error "Not enough resources on board to run example"
#endif


#define CENTRAL_LINK_COUNT              0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define APP_LOG NRF_LOG

#define UART_TX_BUF_SIZE 1024                                                      /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                                                         /**< UART RX buffer size. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_TIMER_PRESCALER            0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE        4                                           /**< Size of timer operation queues. */

#define SECURITY_REQUEST_DELAY         APP_TIMER_TICKS(400, APP_TIMER_PRESCALER)  /**< Delay after connection until Security Request is sent, if necessary (ticks). */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (10 ms). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (100 ms) */
#define SLAVE_LATENCY                  0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                 1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                 1                                           /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_LESC                 0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS             0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< Display I/O capabilities. */
#define SEC_PARAM_OOB                  0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE         7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE         16                                          /**< Maximum encryption key size. */

#define PASSKEY_TXT                    "Passkey:"                                  /**< Message to be displayed together with the pass-key. */
#define PASSKEY_TXT_LENGTH             8                                           /**< Length of message to be displayed together with the pass-key. */
#define PASSKEY_LENGTH                 6                                           /**< Length of pass-key received by the stack for display. */

#define DEAD_BEEF                      0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED      BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

static uint16_t                        m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t m_nus;                                      						/**< Structure to identify the Nordic UART Service. */

APP_TIMER_DEF(m_sec_req_timer_id);                                                /**< Security Request timer. */

pm_peer_id_t peer_to_be_deleted = PM_PEER_ID_INVALID;


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
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch(p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_PRINTF_DEBUG("Connected to previously bonded device\r\n");
            // Start Security Request timer.
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                    APP_ERROR_CHECK(err_code);
            }
        }break;//PM_EVT_BONDED_PEER_CONNECTED

        case PM_EVT_CONN_SEC_START:
            break;//PM_EVT_CONN_SEC_START

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            /*Check if the link is authenticated (meaning at least MITM)*/
            pm_conn_sec_status_t conn_sec_status;
            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            APP_ERROR_CHECK(err_code);
            if (!conn_sec_status.mitm_protected)
            {
                APP_LOG("Collector did not use MITM, disconnecting\r\n");
                /*The peer did not use MITM, disconnect*/
                err_code = pm_peer_id_get(m_conn_handle, &peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }    
            else
            {
                NRF_LOG_PRINTF_DEBUG("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                               ble_conn_state_role(p_evt->conn_handle),
                               p_evt->conn_handle,
                               p_evt->params.conn_sec_succeeded.procedure);
                err_code = pm_peer_rank_highest(p_evt->peer_id);
                if (err_code != NRF_ERROR_BUSY)
                {
                        APP_ERROR_CHECK(err_code);
                }
            }
        }break;//PM_EVT_CONN_SEC_SUCCEEDED

        case PM_EVT_CONN_SEC_FAILED:
        {
            /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
             *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
             *  be restarted until the link is disconnected and reconnected. Sometimes it is
             *  impossible, to secure the link, or the peer device does not support it. How to
             *  handle this error is highly application dependent. */
            APP_LOG("link secure failed! ");        
            switch (p_evt->params.conn_sec_failed.error)
            {
                case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING");
                    break;//PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING

                case PM_CONN_SEC_ERROR_MIC_FAILURE:
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_MIC_FAILURE");
                    break;//PM_CONN_SEC_ERROR_MIC_FAILURE

                case PM_CONN_SEC_ERROR_DISCONNECT :
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_DISCONNECT ");
                    break;//PM_CONN_SEC_ERROR_DISCONNECT

                case PM_CONN_SEC_ERROR_SMP_TIMEOUT:
                    NRF_LOG_DEBUG("error: PM_CONN_SEC_ERROR_SMP_TIMEOUT");
                    break;//PM_CONN_SEC_ERROR_SMP_TIMEOUT

                default:
                    NRF_LOG_DEBUG("unknown error");
                    break;
            }
            APP_LOG("\r\nDisconnecting\r\n");
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
        }break;//PM_EVT_CONN_SEC_FAILED

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        }break;//PM_EVT_CONN_SEC_CONFIG_REQ

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        }break;//PM_EVT_STORAGE_FULL

        case PM_EVT_ERROR_UNEXPECTED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
            break;//PM_EVT_ERROR_UNEXPECTED

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break;//PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            // Assert.
            APP_ERROR_CHECK_BOOL(false);
            break;//PM_EVT_PEER_DATA_UPDATE_FAILED

        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break;//PM_EVT_PEER_DELETE_SUCCEEDED

        case PM_EVT_PEER_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
            break;//PM_EVT_PEER_DELETE_FAILED

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();
            break;

        case PM_EVT_PEERS_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
            break;//PM_EVT_PEERS_DELETE_FAILED

        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break;//PM_EVT_LOCAL_DB_CACHE_APPLIED

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break;//PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break;//PM_EVT_SERVICE_CHANGED_IND_SENT

        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break;//PM_EVT_SERVICE_CHANGED_IND_CONFIRMED

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t             err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Initiate bonding.
        NRF_LOG_DEBUG("Start encryption\r\n");
        err_code = pm_conn_secure(m_conn_handle, false);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    // Create Security Request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t* p_nus, uint8_t* p_data, uint16_t length)
{
    p_data[length] = 0;
    APP_LOG("[APP] Recv: %s ", (char*)p_data);
    APP_LOG("\r\n");

//    const std::string msg((char*)p_data, length);
//
//    if(msg.find("LED2") != std::string::npos){
//    	LEDS_INVERT(BSP_LED_1_MASK);
//    	APP_LOG("[APP] 2 \r\n");
//    }
//    if(msg.find("LED3") != std::string::npos){
//        	LEDS_INVERT(BSP_LED_2_MASK);
//        	APP_LOG("[APP] 3 \r\n");
//
//        }
//    if(msg.find("LED4") != std::string::npos){
//        	//LEDS_INVERT(BSP_LED_3_MASK);
//        	APP_LOG("[APP] 4 \r\n");
//
//        }
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
static void services_init(void)
{
	uint32_t err_code;

	    ble_nus_init_t nus_init;

	    memset(&nus_init, 0, sizeof(nus_init));

	    nus_init.data_handler = nus_data_handler;

	    err_code = ble_nus_init(&m_nus, &nus_init);
	    APP_ERROR_CHECK(err_code);}

/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;                    //lint -save -e438 // Last value assigned to variable 'err_code' not used

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
        {
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            /*check if the last connected peer had not used MITM, if so, delete its bond information*/
            if (peer_to_be_deleted != PM_PEER_ID_INVALID)
            {
                ret_code_t ret_val = pm_peer_delete(peer_to_be_deleted);
                APP_ERROR_CHECK(ret_val);
                APP_LOG("Collector's bond deleted\r\n");
                peer_to_be_deleted = PM_PEER_ID_INVALID;
            }
        }break;//BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
        {
            peer_to_be_deleted = PM_PEER_ID_INVALID;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            // Start Security Request timer.
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
        }break;//BLE_GAP_EVT_CONNECTED

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;//BLE_GATTS_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
        break;//BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            char passkey[PASSKEY_LENGTH+1];
            memcpy(passkey,p_ble_evt->evt.gap_evt.params.passkey_display.passkey,PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;
            // Don't send delayed Security Request if security procedure is already in progress.
            err_code = app_timer_stop(m_sec_req_timer_id);
            APP_ERROR_CHECK(err_code);

            APP_LOG("Passkey: %s\r\n",passkey);
        }break;//BLE_GAP_EVT_PASSKEY_DISPLAY

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;//BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            if(p_ble_evt->evt.gatts_evt.params.authorize_request.type
               != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_PREP_WRITE_REQ)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type
                        == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle,&auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        }break;//BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

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
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    fs_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
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


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;//BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;//BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
        	advertising_restart();
            break;//BSP_EVENT_WHITELIST_OFF

        case BSP_EVENT_KEY_1:
        	ble_nus_string_send(&m_nus, (uint8_t*)"KEY1\r\n", 5);
            break;//BSP_EVENT_KEY_1

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond              = SEC_PARAM_BOND;
    sec_param.mitm              = SEC_PARAM_MITM;
    sec_param.lesc              = SEC_PARAM_LESC;
    sec_param.keypress          = SEC_PARAM_KEYPRESS;
    sec_param.io_caps           = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob               = SEC_PARAM_OOB;
    sec_param.min_key_size      = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size      = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc     = 1;
    sec_param.kdist_own.id      = 1;
    sec_param.kdist_peer.enc    = 1;
    sec_param.kdist_peer.id     = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
    err_code = NRF_LOG_INIT();
    APP_ERROR_CHECK(err_code);

    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        APP_LOG("Bonds erased!\r\n");
    }
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.

    advertising_start();

    APP_LOG("\r\nPMD Start!\r\n");

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

/**
 * @}
 */
