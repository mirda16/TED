#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
//#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_saadc.h"
//#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"
//#include "nrf_ppi.h"
#include "nrf_csense.h"
#include "nrf_drv_csense.h"
#include "myservice.h"


/*#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
*/

#define DEVICE_NAME                     "TED_Matrace1"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                0                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
/*CSENCE SECTION*/
#define APP_TIMER_TICKS_TIMEOUT APP_TIMER_TICKS(2000)
APP_TIMER_DEF(m_app_timer_id);

uint8_t char2_data[CHARACTERISTIC_SIZE];
uint16_t csense_data[CHARACTERISTIC_SIZE];
volatile uint8_t state = 1;
static uint8_t custom_value = 0;
static int timeCounter = 0;
static int maxCounter = 100;


/* Pin used to measure capacitor charging time. */
#if USE_COMP == 0
#ifdef ADC_PRESENT
#define OUTPUT_PIN 30
#elif defined(SAADC_PRESENT)
#define OUTPUT_PIN 26
#endif
#endif

/* Analog inputs. */
#define AIN_1                   1  // analog input 1 (P0
#define AIN_7                   7  // analog input 1 (P0
#define AIN_4                   4  // analog input 1 (P0

#define SAMPLES_IN_BUFFER 1  // saadc sample buffer

/* Masks of analog channels to be used by library. */
#define PAD_1_MASK              (1UL << AIN_1)
//#endif
#define PAD_2_MASK              (1UL << AIN_7)

#define PAD_3_MASK              (1UL << AIN_4)
//#define LED                     14
#define IN                      5
#define GND_h1                  8  // GND is not used
#define GND_h2                  9  // GND is not used

uint8_t channel;
uint16_t cap;
static nrf_saadc_value_t m_buffer[SAMPLES_IN_BUFFER];
uint16_t adc_result = 0;

uint16_t val = 100;
uint16_t prewData[3] = {0,0,0};
static uint8_t dataToSend[CHARACTERISTIC_SIZE];

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

/*void update_data()
{
    ret_code_t err_code;
    //Send back some data
    for(int i=1;i<2;i++){
        char2_data[i] = 0;
    }
    char2_data[0] = maxCounter;
    char2_data[1] = timeCounter;
    err_code = my_service_update_data(m_conn_handle, char2_data);
    if (err_code !=0)
    {
    APP_ERROR_CHECK(err_code);
    }
}*/

/**
 * @brief Tell if the pad is occupated or free or if humidity sensor is wet or not.
 *
 * @param [in] p_presence_data       Pointer na vstupni data.
 * @param [in] humidity              Hodnota daná AD převodníkem snímajícím vlhkost podlozky. S vlhkosti hodnota klesa.
 * 
 * @return None
 */

 void presence (uint16_t *p_presence_data, uint16_t humidity, uint8_t *p_out_presence){
  uint16_t hum_levels[3] = {700, 800, 900};  // levely jsou dany pro tri urovne mokre matrace, cim je matrace mokrejsi tim je hodnota nizzi
  uint16_t presence_levels_12[3] = {100, 38, 100}; // sucha; vlhka
  uint16_t presence_levels_3[3] = {1490, 40, 100};  // sucha; vlhka

  if (humidity > hum_levels[2]){
    // schua matrace;
    p_out_presence[6] = 0x30; //HEX ASCII 0 
    if (p_presence_data[2] < presence_levels_3[0]){
      // prostredni segment obsazeny
      p_out_presence[4] = 0x31;  // HEX ASCII 1, matrace obsazena 
      if (p_presence_data[0] > presence_levels_12[0]){
        // krajni segment obsazeny
        p_out_presence[0] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[0] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
      if (p_presence_data[1] > presence_levels_12[0]){
        // krajni segment obsazeny
        p_out_presence[2] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[2] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
    }else{
      // prostredni segment prazdny
      p_out_presence[4] = 0x31; //HEX ASCII 0, matrace prazna
      if (p_presence_data[0] > presence_levels_12[1]){
        // krajni segment obsazeny
        p_out_presence[0] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[0] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
      if (p_presence_data[1] > presence_levels_12[1]){
        // krajni segment obsazeny
        p_out_presence[2] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[2] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
      }
  }else if (humidity < hum_levels[0]){
    // mokra matrace
    p_out_presence[6] = 0x31; //HEX ASCII 1
    if (p_presence_data[2] < presence_levels_3[0]){
      // prostredni segment obsazeny
      p_out_presence[4] = 0x31;  // HEX ASCII 1, matrace obsazena 
      if (p_presence_data[0] > presence_levels_12[0]){
        // krajni segment obsazeny
        p_out_presence[0] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[0] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
      if (p_presence_data[1] > presence_levels_12[0]){
        // krajni segment obsazeny
        p_out_presence[2] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[2] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
    }else{
      // prostredni segment prazdny
      p_out_presence[4] = 0x30; //HEX ASCII 0, matrace prazna
      if (p_presence_data[0] > presence_levels_12[0]){
        // krajni segment obsazeny
        p_out_presence[0] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[0] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
      if (p_presence_data[1] > presence_levels_12[0]){
        // krajni segment obsazeny
        p_out_presence[2] = 0x31;  // HEX ASCII 1, matrace obsazena
      }else{
        // krajni segment prazdny
        p_out_presence[2] = 0x30;  // HEX ASCII 0, matrace prazdna
      }
    }
  }
}   

/**
 * @brief Low pass filtr.
 *
 * @param [in] rawData       Pointer na vstupni data.
 * @param [in] SmoothDataFP          Mnozstvi dat ze kterych se ma pocitat prumer.
 * 
 * @return Prumer z casti vstupnich dat, ktera jedefinovana v len.
 */
uint16_t low_pass(int16_t rawData, uint16_t SmoothDataFP){
  uint16_t SmoothDataINT;
  uint8_t Beta = 1;  // Length of the filter < 16
  // uint8_t FP_Shift= 3;
  // Function that brings Fresh Data into RawData
  // rawData <<= FP_Shift; // Shift to fixed point
  SmoothDataFP = (SmoothDataFP << Beta)-SmoothDataFP; 
  SmoothDataFP += rawData;
  SmoothDataFP >>= Beta;
  // Don't do the following shift if you want to do further
  // calculations in fixed-point using SmoothData
  // SmoothDataINT = SmoothDataFP>> FP_Shift;
  return SmoothDataFP;
}

// SAADC

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
  if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
  {
    ret_code_t err_code;
    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    adc_result = p_event->data.done.p_buffer[0];
  }
}

void init_saadc()
{
  ret_code_t err_code;
  /*Config the channel*/
  //nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6); // NRF52840DK
  nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2); // Board v1 ADC channel config
  //channel_config.acq_time = NRF_SAADC_ACQTIME_20US;
  
  err_code = nrf_drv_saadc_init(NULL, NULL);
  APP_ERROR_CHECK(err_code);
  
  err_code = nrf_drv_saadc_channel_init(6, &channel_config);  // chanel 6 of ADC inicialize
  APP_ERROR_CHECK(err_code);
  
 // err_code = nrf_drv_saadc_buffer_convert(m_buffer,SAMPLES_IN_BUFFER);
 // APP_ERROR_CHECK(err_code);
}

void take_adc_sample()
{
  ret_code_t err_code;
  err_code = nrfx_saadc_sample_convert(6, &adc_result);
  APP_ERROR_CHECK(err_code);
}

void csense_handler(nrf_drv_csense_evt_t * p_event_struct)
{ 
  if (p_event_struct->analog_channel == 7){
    csense_data[0] = p_event_struct->read_value;
    }else if (p_event_struct->analog_channel == 4){
      csense_data[1] = p_event_struct->read_value;
    }else{
      csense_data[2] = p_event_struct->read_value;
    }
  channel = p_event_struct->analog_channel;
} 

/**
 * @brief Function for initializing the capacitive sensor.
 */
void csense_initialize(void)
{
    ret_code_t err_code;
    nrf_drv_csense_config_t csense_config;

#if USE_COMP == 0
    csense_config.output_pin = OUTPUT_PIN;
#endif
    // !!! pro detekci na vsech snimacich jednotlive je nutne zachovat toto nastaveni !!!
    nrf_gpio_pin_clear(IN); // set IN state to 0
    nrf_gpio_cfg_input(IN, NRF_GPIO_PIN_PULLUP);      // set IN as input nopull, pokud ma cist spravne i spolecny musi tu byt nopull
    //nrf_gpio_cfg_input(GND_h1, NRF_GPIO_PIN_NOPULL);  // set GND_h1 as input nopull
    //nrf_gpio_cfg_input(GND_h2, NRF_GPIO_PIN_NOPULL);  // set GND_h2 as input nopull
    nrf_gpio_cfg_input(4, NRF_GPIO_PIN_NOPULL);       // set hum (AIN2/P0.04) as input nopull
    nrf_delay_ms(10);
    err_code = nrf_drv_csense_init(&csense_config, csense_handler);  // csense inicialization
    APP_ERROR_CHECK(err_code);
    nrf_drv_csense_channels_enable(PAD_1_MASK | PAD_2_MASK | PAD_3_MASK);  // three chanes are enabled by their bitmasks
    err_code = nrf_drv_csense_sample(); // take one sample for all enabled chanels (analog inputs)
    APP_ERROR_CHECK(err_code);
}

void timer_signal_handler(void* p_context)
{ 
    ret_code_t err_code;
    timeCounter++;
    if (timeCounter % 2 == 1)
    {
      csense_initialize();  //inicialize csense, take one sample after initialization
      //nrf_delay_ms(10);
      //nrf_drv_csense_uninit();  // csense unitialize
    }
    // tohle dela stridani csence a adc, pokud jsem to delal v jednom behu programu tak to pada
    if (timeCounter % 2 == 0)
    {
      prewData[0] = low_pass(csense_data[0], prewData[0]);  // low pass filtering
      prewData[1] = low_pass(csense_data[1], prewData[1]);  // low pass filtering
      prewData[2] = low_pass(csense_data[2], prewData[2]);  // low pass filtering

      /*
      NRF_LOG_INFO("Channel: 7, \t value: %d  filtered %d" ,csense_data[0], prewData);
      NRF_LOG_INFO("Channel: 7, \t value: %d" ,csense_data[0]);
      NRF_LOG_INFO("Channel: 4, \t value: %d" ,csense_data[1]);
      NRF_LOG_INFO("Channel: 1, \t value: %d" ,csense_data[2]);
      */
      // prepare RAW dat afrom csense to send
      dataToSend[0] = (uint8_t)(prewData[0] >> 8);
      dataToSend[1] = (uint8_t)(prewData[0]);
      dataToSend[2] = (uint8_t)(prewData[1] >> 8);
      dataToSend[3] = (uint8_t)(prewData[1]);
      dataToSend[4] = (uint8_t)(prewData[2] >> 8);
      dataToSend[5] = (uint8_t)(prewData[2]);

      /*err_code = my_service_update_data(m_conn_handle, dataToSend);
      APP_ERROR_CHECK(err_code);
      err_code = BLE_NUS_ENABLED;*/

      nrf_drv_csense_uninit();  // csense unitialize
      //NRF_LOG_INFO("Csense unitialized!!!");
      nrf_delay_ms(10);
      init_saadc();  // saadc initialize
      // pin settiongs for ADC sampling
      nrf_gpio_cfg_output(IN);  // set IN(P0.05) as output
      nrf_gpio_pin_set(IN);  // set IN state to 1
      nrf_gpio_cfg_output(GND_h1);  // set GND as output
      nrf_gpio_pin_clear(GND_h1);  // set GND state to 0
      nrf_gpio_cfg_output(GND_h2);  // set GND as output
      nrf_gpio_pin_clear(GND_h2);  // set GND state to 0
      nrf_delay_ms(10);
      take_adc_sample();  // take one ADC sample
      //NRF_LOG_INFO("Saadc sample is: %d", adc_result);
      //nrf_drv_saadc_abort();  // tohle nefunguje
      nrf_drv_saadc_uninit();  // uninitialization SAADC
      /*nrf_gpio_pin_clear(IN);  // set IN state to 0
      nrf_gpio_cfg_input(IN, NRF_GPIO_PIN_NOPULL);
      nrf_gpio_cfg_input(GND_h1, NRF_GPIO_PIN_NOPULL);
      nrf_gpio_cfg_input(GND_h2, NRF_GPIO_PIN_NOPULL);*/
      // prepare RAW data to sending
      dataToSend[6] = (uint8_t)(adc_result >> 8);
      dataToSend[7] = (uint8_t)(adc_result);
      // update data to send

      //presence part
      //presence (prewData, adc_result, dataToSend);  // presence and humidity check
      err_code = my_service_update_data(m_conn_handle, dataToSend);  // my service update
      APP_ERROR_CHECK(err_code);
      err_code = BLE_NUS_ENABLED;
    }
}


static void advertising_start(bool erase_bonds);


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
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_signal_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
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


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    err_code = my_service_init();  
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

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
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, APP_TIMER_TICKS_TIMEOUT, NULL);
       APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            //advertising_start();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            //NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            //NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            //NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            //NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            //NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    //NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
/*static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}*/


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    //if (NRF_LOG_PROCESS() == false)
    //{
        nrf_pwr_mgmt_run();
    //}
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    //log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();
    
    //nrf_gpio_cfg_output(LED);
    
    //ret_code_t err_code;
    //err_code = NRF_LOG_INIT(NULL);
    //APP_ERROR_CHECK(err_code);
    
    // Start execution.
    //NRF_LOG_INFO("Template example started.");
    //NRF_LOG_FLUSH();
    //csense_initialize();  //inicialize csense
    application_timers_start();
    advertising_start(erase_bonds);
    //NRF_LOG_INFO("Capacitive sensing driver example started. \n");
    //NRF_LOG_FLUSH();
    
    
    
    // Enter main loop.
    for (;;)
    {
        __WFI();
        nrf_pwr_mgmt_run();
        idle_state_handle();
        //NRF_LOG_FLUSH();
    }
}


/**
 * @}
 */
