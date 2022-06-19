/*
 * Copyright 2022 Jakub Oleksiak

 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

extern "C" {
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "driver/uart.h"

#include "time.h"
#include "sys/time.h"

#include "esp_vfs.h"
#include "sys/unistd.h"

extern void app_main(void);

}

#include "generic_task.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

typedef struct {
    uint16_t             event;
    void                 *param;
} msg_spp_t;

static void esp_spp_cb(generic_task<msg_spp_t>& task, msg_spp_t *pMsg);

generic_task<msg_spp_t> _spp_task("SPPAppT", esp_spp_cb, 10 /* was priority 12 */, 2048, 10);


    //Set UART pins (using UART0 default pins ie no changes.)
    //GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE
    //uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num);
    //uart_set_pin(UART_NUM_0, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //UART0 -   TX - 35 txd0 gpio1
    //          RX - 34 rxd0 gpio3
    //          RTS - 36 io22 gpio22
    //          CTS - 31 io19 gpio19
    //UART1 -   TX - 18 swp/sd3 (D3) gpio10
    //          RX - 17 shd/sd2 (D2) gpio9
    //          RTS - 19 scs/cmd (cmd) gpio11
    //          CTS - 20 sck/clk (clk) gpio6
    //UART2 -   TX - 28 IO17 (17) gpio17
    //          RX - 27 io16 (16) gpio16
    //          RTS - 21 sd0/sd0 (d0) gpio7
    //          CTS - 22 sd1/sd1 (d1) gpio8
#define UART0_PIN_TX    1
#define UART0_PIN_RX    3
#define UART0_PIN_RTS   22
#define UART0_PIN_CTS   19
//------------------------------
#define UART1_PIN_TX    10
#define UART1_PIN_RX    9
#define UART1_PIN_RTS   11
#define UART1_PIN_CTS   6
//------------------------------
#define UART2_PIN_TX    17
#define UART2_PIN_RX    16
#define UART2_PIN_RTS   7
#define UART2_PIN_CTS   8

static const char *UART_TAG = "uart_events";

#define UART_EX_UART_NUM UART_NUM_1

#define UART_PIN_TX    UART1_PIN_TX
#define UART_PIN_RX    UART1_PIN_RX
#define UART_PIN_RTS   UART_PIN_NO_CHANGE
#define UART_PIN_CTS   UART_PIN_NO_CHANGE


#define UART_BUF_SIZE (1024)
#define UART_RD_BUF_SIZE (UART_BUF_SIZE)

static void uart_cb(generic_task<uart_event_t>& task, uart_event_t* pMsg);

generic_task<uart_event_t> _uart_task("UARTAppT", uart_cb, 10 /* was priority 12 */, 2048, 10);

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}


//bt serial port profile
//this one is exec in separate spp_task
//static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
static void esp_spp_cb(generic_task<msg_spp_t>& task, msg_spp_t *pMsg)
{
    esp_spp_cb_event_t event = static_cast<esp_spp_cb_event_t>(pMsg->event);
    esp_spp_cb_param_t *param = static_cast<esp_spp_cb_param_t*>(pMsg->param);

    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        *reinterpret_cast<int*>(task._pContext) = 0;
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d", param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len > 0)
        {
            size_t len = param->data_ind.len;
            void *pBuff = param->data_ind.data;

            //esp_log_buffer_hex("", pBuff, len);

            //esp_spp_write(param->data_ind.handle, param->data_ind.len, param->data_ind.data);
            int t_len = uart_write_bytes(UART_EX_UART_NUM, (const char*) pBuff, len);
            if (len != t_len)
            {
                ESP_LOGE(SPP_TAG, "ESP_SPP_DATA_IND_EVT Input len:%d not match transmit len:%d", len, t_len);
            }
            else
            {
                ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT transmission end");
            }
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        //gettimeofday(&time_old, NULL);
        //handle =param->open.handle;
        *reinterpret_cast<int*>(task._pContext) = param->srv_open.handle;
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }

    if (pMsg->param != nullptr)
    {
        delete[] static_cast<uint8_t*>(pMsg->param);
        pMsg->param = nullptr;
    }
}



//generic access profile - controls connections and advertising. visibility and interact
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}



static void esp_spp_stack_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    msg_spp_t msg;
    std::memset(&msg, 0, sizeof(msg_spp_t));

    msg.event = event;

    if ((msg.param = new uint8_t[sizeof(esp_spp_cb_param_t)] /* malloc(param_len) */) != nullptr) {
        std::memcpy(msg.param, param, sizeof(esp_spp_cb_param_t));
        _spp_task.send_msg(&msg);
    }
    else {
        ESP_LOGE(SPP_TAG, "ESP_SPP_STACK alloc error");
    }
}



static void uart_cb(generic_task<uart_event_t>& task, uart_event_t* pMsg)
{
    switch(pMsg->type) {
        case UART_DATA:
            ESP_LOGI(UART_TAG, "[UART DATA]: to be read %d", pMsg->size);
            //uart_write_bytes(UART_EX_UART_NUM, (const char*) task._pContext, pMsg->size);
            if (pMsg->size > 0)
            {
                int i_len = UART_RD_BUF_SIZE;
                if (i_len > pMsg->size)
                {
                    i_len = pMsg->size;
                }
                int rx_len = uart_read_bytes(UART_EX_UART_NUM, task._pContext, i_len, portMAX_DELAY);
                ESP_LOGI(UART_TAG, "[UART DATA]: read %d", rx_len);
                if (rx_len > 0)
                {
                    int h = *reinterpret_cast<int*>(_spp_task._pContext);
                    if (h!=0)
                    {
                        if (esp_spp_write(h, rx_len, task._pContext) != ESP_OK)
                        {
                            ESP_LOGE(UART_TAG, "Writing UART data to spp failed. Data lost.");
                        }
                        else
                        {
                            ESP_LOGI(UART_TAG, "Uart transmission end");
                        }
                    }
                    else
                    {
                        ESP_LOGE(UART_TAG, "Uart data received, but spp closed. Data lost.");
                    }
                }
                else
                {
                    ESP_LOGE(UART_TAG, "Uart data not read. Data lost.");
                }
            }
            break;

        case UART_FIFO_OVF:
            ESP_LOGI(UART_TAG, "hw fifo overflow");
            uart_flush_input(UART_EX_UART_NUM);
            task.reset_queue();
            break;

        case UART_BUFFER_FULL:
            ESP_LOGI(UART_TAG, "ring buffer full");
            uart_flush_input(UART_EX_UART_NUM);
            task.reset_queue();
            break;

        case UART_BREAK:
            ESP_LOGI(UART_TAG, "uart rx break");
            break;

        case UART_PARITY_ERR:
            ESP_LOGI(UART_TAG, "uart parity error");
            break;

        case UART_FRAME_ERR:
            ESP_LOGI(UART_TAG, "uart frame error");
            break;
			
        //UART_PATTERN_DET

        default:
            ESP_LOGI(UART_TAG, "uart event type: %d", pMsg->type);
            break;
    }

}




void app_main(void)
{
    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    esp_log_level_set(SPP_TAG, ESP_LOG_INFO);

    ESP_LOGI(SPP_TAG, "Initializing BT\n");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if (esp_spp_register_callback(esp_spp_stack_cb) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed", __func__);
        return;
    }

    _spp_task._pContext = new uint8_t[8];

    if ((ret = _spp_task.start_up()) != 0)
    {
        ESP_LOGE(SPP_TAG, "%s spp task startup: %d\n", __func__, (int)ret);
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));




    esp_log_level_set(UART_TAG, ESP_LOG_INFO);

    ESP_LOGI(UART_TAG, "Initializing UART\n");

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB
    };

    QueueHandle_t uart_uart0_queue = nullptr;
    int rx_buffer_size = UART_BUF_SIZE * 2;
    int tx_buffer_size = UART_BUF_SIZE * 2;
    int uart_queue_size = 20;
    int intr_alloc_flags = 0;
	
    if ((ret = uart_driver_install(UART_EX_UART_NUM, rx_buffer_size, tx_buffer_size, 
                                    uart_queue_size, &uart_uart0_queue, intr_alloc_flags)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s uart driver install failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    if ((ret = uart_param_config(UART_EX_UART_NUM, &uart_config)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s uart param config failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //Set UART log level
    esp_log_level_set(UART_TAG, ESP_LOG_INFO);

    if ((ret = uart_set_pin(UART_EX_UART_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_RTS, UART_PIN_CTS)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s uart set pin failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }


    _uart_task._pContext = new uint8_t[UART_RD_BUF_SIZE];

    if ((ret = _uart_task.start_up(&uart_uart0_queue)) != 0)
    {
        ESP_LOGE(SPP_TAG, "%s uart task startup: %d\n", __func__, (int)ret);
        return;
    }

    ESP_LOGI(UART_TAG, "Initializing done\n");
}
