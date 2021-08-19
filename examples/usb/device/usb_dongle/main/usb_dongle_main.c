/**
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 * 
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include <stdint.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

#include "tinyusb.h"

#include "cmd_wifi.h"

#if CFG_TUD_NET
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#endif /* CFG_TUD_NET */

#if CFG_TUD_BTH
#include "esp_bt.h"
#include "bt_hci_common.h"
#endif /* CFG_TUD_BTH */

#if CFG_TUD_CDC
#include "tusb_cdc_acm.h"
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
void Command_Parse(char* Cmd);
#elif CONFIG_UART_ENABLE
void initialise_uart(void);
#endif

#ifdef CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM
#endif /* CONFIG_HEAP_TRACING */

static const char *TAG = "USB_Dongle";

/*
 * Register commands that can be used with FreeRTOS+CLI through the UDP socket.
 * The commands are defined in CLI-commands.c.
 */
void vRegisterCLICommands(void);

#if CFG_TUD_BTH
static uint8_t hci_cmd_buf[128];
static uint8_t hci_evt_buf[128];
static uint8_t hci_acl_buf_host[128]; // recv data from host
static uint8_t hci_acl_long_buf_host[1024]; 
static uint8_t hci_acl_buf_contr[128]; 
SemaphoreHandle_t BT_Semaphore = NULL;

void ble_controller_init(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret;
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGI(TAG, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
}

/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
    //TODO
}

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    memset(hci_evt_buf, 0x0 , 128);
    memset(hci_acl_buf_contr, 0x0 , 128);
    uint16_t act_len = len - 1;
    //printf("host_rcv_pkt: data[0] = %02x\n", data[0]);

    if(data[0] == 0x04) { // event data from controller
      memcpy(hci_evt_buf, data +1 , act_len);
      printf("evt_data from controller, evt_data_length: %d :\n", act_len);
      for (uint16_t i = 0; i < act_len; i++) {
        printf("%02x ", hci_evt_buf[i]);
      }
      printf("\n");
      xSemaphoreGive(BT_Semaphore);
      tud_bt_event_send(hci_evt_buf, act_len);
    } else if(data[0] == 0x02) { // acl data from controller
      memcpy(hci_acl_buf_contr, data +1 , act_len);
      printf("acl_data from controller, acl_data_length: %d :\n", act_len);
      for (uint16_t i = 0; i < act_len; i++) {
        printf("%02x ", hci_acl_buf_contr[i]);
      }
      printf("\n");
      //xSemaphoreGive(BT_Semaphore);
      tud_bt_acl_data_send(hci_acl_buf_contr, act_len);
    }
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

// Invoked when HCI command was received over USB from Bluetooth host.
// Detailed format is described in Bluetooth core specification Vol 2,
// Part E, 5.4.1.
// Length of the command is from 3 bytes (2 bytes for OpCode,
// 1 byte for parameter total length) to 258.
void tud_bt_hci_cmd_cb(void *hci_cmd, size_t cmd_len)
{
    memset(hci_cmd_buf, 0x0 , 128);
    hci_cmd_buf[0] = 0x01;
    memcpy(hci_cmd_buf+1, hci_cmd, cmd_len);

    printf("cmd_data from host, length: %d\n",cmd_len +1);
    for(int i = 0 ; i< cmd_len + 1; i++) {
        printf("%02x ",hci_cmd_buf[i]);
    }
    printf("\n");

    esp_vhci_host_send_packet(hci_cmd_buf, cmd_len +1);

    xSemaphoreTake(BT_Semaphore, portMAX_DELAY);
}

// Invoked when ACL data was received over USB from Bluetooth host.
// Detailed format is described in Bluetooth core specification Vol 2,
// Part E, 5.4.2.
// Length is from 4 bytes, (12 bits for Handle, 4 bits for flags
// and 16 bits for data total length) to endpoint size.
static bool prepare_write = false;
static uint16_t write_offset = 0;
static uint16_t acl_data_length = 0;
void tud_bt_acl_data_received_cb(void *acl_data, uint16_t data_len)
{ // if acl_data is long data
    if(!prepare_write) {
        // first get acl_data_length
        acl_data_length = *(((uint16_t * )acl_data) + 1);
        if(acl_data_length > data_len) {
          prepare_write = true;
          memset(hci_acl_buf_host, 0x0, 128);
          hci_acl_buf_host[0] = 0x02;
          memcpy(hci_acl_buf_host + 1, acl_data, data_len);
          write_offset = data_len + 1;
        } else {
          memset(hci_acl_buf_host, 0x0 , 128);
          hci_acl_buf_host[0] = 0x02;
          memcpy(hci_acl_buf_host + 1, acl_data, data_len);
          printf("short acl_data from host, will send to controller, length: %d\n",data_len + 1);
          for(int i = 0 ; i< data_len + 1; i++) {
            printf("%02x ",hci_acl_buf_host[i]);
          }
          printf("\n");
          esp_vhci_host_send_packet(hci_acl_buf_host, data_len + 1);
        }
    } else {
        memcpy(hci_acl_buf_host + write_offset, acl_data, data_len);
        write_offset += data_len;
        if(acl_data_length > write_offset) {
          printf(" Remaining bytes: %d\n", acl_data_length - write_offset);
          return;
        }
        printf("long acl_data from host, will send to controller, length: %d\n",data_len + 1);
        for(int i = 0 ; i< write_offset; i++) {
          printf("%02x ",hci_acl_buf_host[i]);
        }
        printf("\n");
        prepare_write = false;
        esp_vhci_host_send_packet(hci_acl_buf_host, data_len + write_offset);
    }
}

// Called when event sent with tud_bt_event_send() was delivered to BT stack.
// Controller can release/reuse buffer with Event packet at this point.
void tud_bt_event_sent_cb(uint16_t sent_bytes)
{
    //TODO
}

// Called when ACL data that was sent with tud_bt_acl_data_send()
// was delivered to BT stack.
// Controller can release/reuse buffer with ACL packet at this point.
void tud_bt_acl_data_sent_cb(uint16_t sent_bytes)
{
    //TODO
}
#endif /* CFG_TUD_BTH */

#if CFG_TUD_NET
extern bool s_wifi_is_connected;
SemaphoreHandle_t Net_Semphore;
bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
    if (s_wifi_is_connected) {
      esp_wifi_internal_tx(ESP_IF_WIFI_STA, src, size);
    }
    tud_network_recv_renew();
    return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
    uint16_t len = arg;

    /* traverse the "pbuf chain"; see ./lwip/src/core/pbuf.c for more info */
    memcpy(dst, ref, len);
    return len;
}

void tud_network_init_cb(void)
{
    /* TODO */
}

bool tud_network_wait_xmit(uint32_t ms)
{
    if (xSemaphoreTake(Net_Semphore, ms/portTICK_PERIOD_MS) == pdTRUE) {
      xSemaphoreGive(Net_Semphore);
      return true;
    }
    return false;
}

void tud_network_idle_status_change_cb(bool enable)
{
    if (enable == true) {
      xSemaphoreGive(Net_Semphore);
    } else {
      xSemaphoreTake(Net_Semphore, 0);
    }
}

esp_err_t pkt_wifi2usb(void *buffer, uint16_t len, void *eb)
{
    if (!tud_ready()) {
      esp_wifi_internal_free_rx_buffer(eb);
      return ERR_USE;
    }
    
    if (tud_network_wait_xmit(100)) {
      /* if the network driver can accept another packet, we make it happen */
      if (tud_network_can_xmit()) {
          tud_network_xmit(buffer, len);
      }
    }

    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}
#endif /* CFG_TUD_NET */

#if CFG_TUD_CDC
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(0, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        buf[rx_size] = '\0';
        Command_Parse((char*)buf);
    } else {
        ESP_LOGE(TAG, "itf %d: itf Read error", itf);
    }
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! itf:%d dtr:%d, rst:%d", itf, dtr, rst);
}
#endif /* CFG_TUD_CDC */

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if CFG_TUD_NET
    vSemaphoreCreateBinary(Net_Semphore);
    initialise_wifi();
#endif /* CFG_TUD_NET */

#if CFG_TUD_BTH
    // init ble controller
    ble_controller_init();
    esp_vhci_host_register_callback(&vhci_host_cb);
    vSemaphoreCreateBinary(BT_Semaphore);
#endif /* CFG_TUD_BTH */

    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t tusb_cfg = {
        .external_phy = false // In the most cases you need to use a `false` value
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

#if CFG_TUD_CDC
    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 128,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
#elif CONFIG_UART_ENABLE
    initialise_uart();
#endif /* CFG_TUD_CDC */

    ESP_LOGI(TAG, "USB initialization DONE");

#ifdef CONFIG_HEAP_TRACING
    heap_trace_init_standalone(trace_record, NUM_RECORDS);
    heap_trace_start(HEAP_TRACE_LEAKS);
#endif
    
    /* Register commands with the FreeRTOS+CLI command interpreter. */
    vRegisterCLICommands();
}
