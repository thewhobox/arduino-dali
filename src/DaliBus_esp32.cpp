/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301  USA
*/

#include "DaliBus.h"
//#ifdef DALI_USE_ESP32

#include "DaliBus_esp32.h"

static rmt_transmit_config_t transmit_config;

static bool dali_rmt_rx_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    printf("dali_rmt_rx_callback\r\n");
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void dali_rmt_rx_task(void *arg)
{
    printf("dali_rmt_rx_task\r\n");
    DaliBusClass *daliClass = (DaliBusClass *)arg;

    printf("daliClass: %p\r\n", daliClass);
    rmt_rx_done_event_data_t rx_data;
    rmt_symbol_word_t raw_symbols[64];
    rmt_receive_config_t dali_rxReceiveConfig = (rmt_receive_config_t) {
        .signal_range_min_ns = DALI_USTONS(2),
        .signal_range_max_ns = DALI_USTONS(DALI_THRESHOLD_2TE_HIGH),
    };

    printf("dali_rxChannel: %p\r\n", daliClass->getRxHandle());
    while(1)
    {
        esp_err_t err = rmt_receive(daliClass->getRxHandle(), raw_symbols, sizeof(raw_symbols), &dali_rxReceiveConfig);
        if(err != ESP_OK)
        {
            printf("rmt_receive failed:              %d (%s)\n", err, esp_err_to_name(err));
            continue;
        }

        if (xQueueReceive(daliClass->getQueueHandle(), &rx_data, pdMS_TO_TICKS(DALI_BACKWARD_FRAME_TIMEOUT_MS)) == pdPASS)
        {
            printf("Received %d symbols\n", rx_data.num_symbols);
        }
    }
}

static size_t dali_rmt_tx_encoder_cb(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    // We need a minimum of 18 symbol spaces to encode a command.
    // Symbol spaces = 1 start bit + 16 bit command + 2 stop bits
    // Commands with more than 2 Bytes cannot be handled.
    if (symbols_free < 18) {
        return 0;
    }

    // Send a start bit first.
    if (symbols_written == 0) {
        symbols[0] = DALI_SYMBOL_ONE;
        return 1;
    }

    // We can calculate where in the data we are from the symbol pos.
    // Divide symbols_written by 8 as each bit translates to one symbol.
    size_t data_pos = (symbols_written - 1) / 8;
    uint8_t *data_bytes = (uint8_t*)data;
    if (data_pos < data_size) {
        // Encode a byte
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            if (data_bytes[data_pos]&bitmask) {
                symbols[symbol_pos++] = DALI_SYMBOL_ONE;
            } else {
                symbols[symbol_pos++] = DALI_SYMBOL_ZERO;
            }
        }
        // We're done; we should have written 16 symbols.
        return symbol_pos;
    } else {
        // Command has been encoded.
        // Add stop bits, and we're done.
        symbols[0] = DALI_SYMBOL_STOP;
        *done = 1; // Indicate end of the transaction.
        return 1;  // We only wrote one symbol
    }
}

int DaliBusClass::begin(byte tx_pin, byte rx_pin, bool active_low)
{
    esp_err_t resp = 0;

    dali_txChannel = NULL;
    dali_txChannelConfig.clk_src = RMT_CLK_SRC_REF_TICK; // select source clock
    dali_txChannelConfig.gpio_num = (gpio_num_t)tx_pin;
    dali_txChannelConfig.mem_block_symbols = 64;
    dali_txChannelConfig.resolution_hz = DALI_RMT_RESOLUTION_HZ;
    dali_txChannelConfig.trans_queue_depth = 3; // set the number of transactions that can be pending in the background
    dali_txChannelConfig.flags.invert_out = false;
    resp = rmt_new_tx_channel(&dali_txChannelConfig, &dali_txChannel);
    printf("rmt_new_tx_channel:              %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_CREATE_TX;

    // TODO wont work
    // dali_rxChannelConfig = (rmt_receive_config_t) {
    //     .signal_range_min_ns = DALI_USTONS(2),
    //     .signal_range_max_ns = DALI_USTONS(DALI_THRESHOLD_2TE_HIGH),
    // };

    dali_txChannelEncoder = NULL;
    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = dali_rmt_tx_encoder_cb
        //Note we don't set min_chunk_size here as the default of 64 is good enough.
    };
    resp = rmt_new_simple_encoder(&simple_encoder_cfg, &dali_txChannelEncoder);
    printf("rmt_new_simple_encoder:          %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_CREATE_ENCODER;

    resp = rmt_enable(dali_txChannel);
    printf("rmt_enable:                      %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_ENABLE_TX;

    transmit_config = (rmt_transmit_config_t) {
        .loop_count = 0
    };

    dali_rxChannel = NULL;
    dali_rxChannelConfig.clk_src = RMT_CLK_SRC_REF_TICK;
    dali_rxChannelConfig.resolution_hz = DALI_RMT_RESOLUTION_HZ;
    dali_rxChannelConfig.mem_block_symbols = 64; // amount of RMT symbols that the channel can store at a time
    dali_rxChannelConfig.gpio_num = (gpio_num_t)rx_pin;
    dali_rxChannelConfig.flags.invert_in = true;
    resp = rmt_new_rx_channel(&dali_rxChannelConfig, &dali_rxChannel);
    printf("rmt_new_rx_channel:              %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_CREATE_RX;
    
    dali_rxChannelQueue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
     rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = dali_rmt_rx_callback,
    };
    resp = rmt_rx_register_event_callbacks(dali_rxChannel, &cbs, dali_rxChannelQueue);
    printf("rmt_rx_register_event_callbacks: %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK);
        return DALI_ERR_CREATE_RX;
        
    resp = rmt_enable(dali_rxChannel);
    printf("rmt_enable:                      %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_ENABLE_RX;

    TaskHandle_t rxTaskHandle;
    BaseType_t resp2 = xTaskCreate(dali_rmt_rx_task, "daliRX", 3048, this, 0, &rxTaskHandle);
    printf("xTaskCreate:                     %d (%s)\n", resp2, resp2 == pdPASS ? "pdPASS" : "pdFAILED");
    printf("rxTaskHandle:                    %p\n", rxTaskHandle);

    printf("DaliBus initialized\n");
    return 0;
}

rmt_channel_handle_t DaliBusClass::getRxHandle()
{
    return dali_rxChannel;
}

QueueHandle_t DaliBusClass::getQueueHandle()
{
    return dali_rxChannelQueue;
}

daliReturnValue DaliBusClass::sendRaw(const byte * message, uint8_t bits)
{
    isSending = true;

    printf("Sending %d bits\n", bits);
    esp_err_t error = rmt_transmit(dali_txChannel, dali_txChannelEncoder, message, bits / 8, &transmit_config);
    printf("rmt_transmit:           %d (%s)\n", error, esp_err_to_name(error));
    if(error != ESP_OK)
    {
        isSending = false;
        // TODO add more handling of error
        return DALI_TX_ERROR;
    }
    error = rmt_tx_wait_all_done(dali_txChannel, 100);
    printf("rmt_tx_wait_all_done:   %d (%s)\n", error, esp_err_to_name(error));
    if(error != ESP_OK)
    {
        isSending = false;
        if(error == ESP_ERR_INVALID_ARG)
            return DALI_INVALID_PARAMETER;
        else if(error == ESP_ERR_TIMEOUT)
            return DALI_SEND_TIMEOUT;
        return DALI_SEND_TIMEOUT;
    }

    isSending = false;
    return DALI_NO_ERROR;
}

int DaliBusClass::getLastResponse()
{
    return 0;
}

bool DaliBusClass::busIsIdle()
{
    return !isSending;
}
//#endif