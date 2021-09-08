#ifndef MYSERVICE_H__
#define MYSERVICE_H__

#include <stdint.h>
#include "ble_srv_common.h"

#define BLE_UUID_CUSTOM_SERVICE          0x343d1980ee0011eb9a030242ac130003
#define BLE_UUID_CUSTOM_CHAR1            0x0001
#define BLE_UUID_CUSTOM_CHAR2            0x0002
#define CHARACTERISTIC_SIZE              10


uint32_t myservice_init(void);
uint32_t my_service_update_data(uint16_t conn_handle, uint8_t *new_data);

#endif