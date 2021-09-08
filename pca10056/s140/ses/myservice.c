#include "myservice.h"
#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static uint16_t                 service_handle;
static ble_gatts_char_handles_t char1_handles;
static ble_gatts_char_handles_t char2_handles;

uint32_t my_service_init(void)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    ble_srv_security_mode_t        dis_attr_md;
    uint8_t i=0;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CUSTOM_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    ble_gatts_char_md_t  char_md;
    ble_gatts_attr_t     attr_char_value; 
    ble_gatts_attr_md_t  attr_md;

    uint8_t initial_char_values[CHARACTERISTIC_SIZE];
    for(i=0;i<CHARACTERISTIC_SIZE;i++) initial_char_values[i] = 0x11;
 
    /*char1 will be for writing*//*
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write_wo_resp    = 1;
    char_md.char_props.write    = 1;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CUSTOM_CHAR1);

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.write_perm);
    attr_md.read_perm  = dis_attr_md.read_perm;
    attr_md.write_perm = dis_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHARACTERISTIC_SIZE;
    attr_char_value.max_len   = CHARACTERISTIC_SIZE;
    attr_char_value.p_value   = initial_char_values;

    err_code =  sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, &char1_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }*/

    /*Char2 will be for reading only*/
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read  = 1;
    char_md.char_props.notify = 1;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CUSTOM_CHAR2);

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.write_perm);
    attr_md.read_perm  = dis_attr_md.read_perm;
    attr_md.write_perm = dis_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHARACTERISTIC_SIZE;
    attr_char_value.max_len   = CHARACTERISTIC_SIZE;
    attr_char_value.p_value   = initial_char_values;

    err_code =  sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, &char2_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t my_service_update_data(uint16_t conn_handle, uint8_t *new_data)
{   

    //uint8_t len = 6;
    uint32_t err_code = NRF_SUCCESS;

    ble_gatts_value_t gatts_value;
    memset(&gatts_value, 0, sizeof(gatts_value));
    //gatts_value.len     = sizeof(&new_data);
    gatts_value.len = 10;
    gatts_value.offset  = 0;
    gatts_value.p_value = new_data;
    //NRF_LOG_INFO("Data to send: %d \n", new_data);
    //gatts_value.len     = 4;
    //uint8_t values[4] = {0x12, 0x35, 0x56, 0x78};
    //gatts_value.p_value = values;

    if(conn_handle!=BLE_CONN_HANDLE_INVALID)
    {
        //NRF_LOG_INFO("To send:");
        err_code = sd_ble_gatts_value_set(conn_handle,
                                          char2_handles.value_handle,
                                          &gatts_value);
       NRF_LOG_INFO("Data updated");
    }
    return err_code;
}

/*uint32_t my_service_update_data(uint16_t conn_handle, uint8_t *new_data)
{   

uint16_t len = 10;
uint32_t err_code = NRF_SUCCESS;
ble_gatts_hvx_params_t hvx_params;
memset(&hvx_params, 0, sizeof(hvx_params));

hvx_params.handle = char2_handles.value_handle;
hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
hvx_params.offset = 0;
hvx_params.p_len  = &len;
hvx_params.p_data = new_data; 


    if(conn_handle!=BLE_CONN_HANDLE_INVALID)
    {
        //NRF_LOG_INFO("To send:");
       err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
       //APP_ERROR_CHECK(err_code);
       NRF_LOG_INFO("Data updated");
    }
    return err_code;
}*/