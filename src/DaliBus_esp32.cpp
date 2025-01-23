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
    //printf("dali_rmt_rx_callback\r\n");
    BaseType_t high_task_wakeup = pdFALSE;
    // QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // // send the received RMT symbols to the parser task
    // xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);

    // printf("edata:                           %p\r\n", edata);
    // printf("user_data:                       %p\r\n", user_data);
    DaliBusClass *daliClass = static_cast<DaliBusClass*>(user_data);
    // printf("daliClass:                       %p\r\n", daliClass);

    xQueueSendFromISR(daliClass->getQueueHandle(), edata, &high_task_wakeup);

    gpio_intr_enable(daliClass->getRxPin());
    return high_task_wakeup == pdTRUE;
}

static void dali_rmt_rx_task(void *arg)
{
    printf("dali_rmt_rx_task\r\n");
    DaliBusClass *daliClass = static_cast<DaliBusClass*>(arg);

    printf("daliClass: %p\r\n", daliClass);
    rmt_receive_config_t dali_rxReceiveConfig = (rmt_receive_config_t) {
        .signal_range_min_ns = DALI_USTONS(2),
        .signal_range_max_ns = DALI_USTONS(DALI_THRESHOLD_2TE_HIGH),
    };
    rmt_rx_done_event_data_t rx_data;

    while(1)
    {
        if (xQueueReceive(daliClass->getQueueHandle(), &rx_data, pdMS_TO_TICKS(DALI_BACKWARD_FRAME_TIMEOUT_MS)) == pdPASS)
        {
            printf("Received %d symbols\n", rx_data.num_symbols);
            //uint8_t data[3];
            uint32_t data = 0;
            size_t sizeInBits = 0;
            esp_err_t resp = daliClass->decode_symbols(&rx_data, &data, &sizeInBits);
            printf("decode_symbols:                  %d (%s) - %.6X %i bits\n", resp, esp_err_to_name(resp), data, sizeInBits);
            if(sizeInBits == 8)
            {
                daliClass->lastResponse = data & 0xFF;
            }
            daliClass->setReceiving(false);

            if(daliClass->receivedCallback != 0)
            {
                daliClass->receivedCallback(&data + 1, sizeInBits);
            }
        }
    }
}

static esp_err_t dali_rmt_rx_decoder(dali_receivePrevBit_t* receive_prev_bit, uint32_t* frame, uint8_t* frame_index, uint16_t duration, uint16_t level) {
    level = (level == 0) ? 1 : 0;
    if ((duration > DALI_USTORMT(DALI_THRESHOLD_1TE_LOW))
            && (duration < DALI_USTORMT(DALI_THRESHOLD_1TE_HIGH))) {
        // short break (1 Te)
        if (((*receive_prev_bit) == DALI_RECEIVE_PREV_BIT_ONE)
                && (level == 0)) {
            // _/-\_/-
            //    ^
            //    ^ ignore this edge
        } else if (((*receive_prev_bit) == DALI_RECEIVE_PREV_BIT_ONE)
                && (level == 1)) {
            // this is a repeated one
            (*frame) <<= 1;
            (*frame) |= 1;
            (*frame_index)++;
            (*receive_prev_bit) = DALI_RECEIVE_PREV_BIT_ONE;
        } else if (((*receive_prev_bit) == DALI_RECEIVE_PREV_BIT_ZERO)
                && (level == 1)) {
            // -\_/-\_
            //    ^
            //    ^ ignore this edge
        } else if (((*receive_prev_bit) == DALI_RECEIVE_PREV_BIT_ZERO)
                && (level == 0)) {
            // this is a repeated zero
            (*frame) <<= 1;
            (*frame_index)++;
            (*receive_prev_bit) = DALI_RECEIVE_PREV_BIT_ZERO;
        }
    } else if ((duration > DALI_USTORMT(DALI_THRESHOLD_2TE_LOW))
            && (duration < DALI_USTORMT(DALI_THRESHOLD_2TE_HIGH))) {
        if (((*receive_prev_bit) == DALI_RECEIVE_PREV_BIT_ONE)
                && (level == 0)) {
            // this is a zero following a one
            (*frame) <<= 1;
            (*frame_index)++;
            (*receive_prev_bit) = DALI_RECEIVE_PREV_BIT_ZERO;
        } else if (((*receive_prev_bit) == DALI_RECEIVE_PREV_BIT_ZERO)
                && (level == 1)) {
            // this is a one following a zero
            (*frame) <<= 1;
            (*frame) |= 1;
            (*frame_index)++;
            (*receive_prev_bit) = DALI_RECEIVE_PREV_BIT_ONE;
        } else {
            // illegal state
            // again, this is somewhere in the middle of a transmission
            // -> do not reset right away, but wait in error state
            return ESP_ERR_INVALID_STATE;
        }
    }

    return ESP_OK;
}

esp_err_t DaliBusClass::decode_symbols(rmt_rx_done_event_data_t *rx_data, uint32_t *data, size_t *size)
{
    dali_receivePrevBit_t received_prev_bit = DALI_RECEIVE_PREV_BIT_ONE;
    uint32_t frame = 0;
    uint8_t frame_index = 0;

    esp_err_t resp = dali_rmt_rx_decoder(&received_prev_bit, &frame, &frame_index, rx_data->received_symbols[0].duration1, rx_data->received_symbols[0].level1);
    for (size_t i = 1; i < rx_data->num_symbols; i++) {
        resp = dali_rmt_rx_decoder(&received_prev_bit, &frame, &frame_index, rx_data->received_symbols[i].duration0, rx_data->received_symbols[i].level0);
        if (frame_index == 8) {
            break;
        }
        resp = dali_rmt_rx_decoder(&received_prev_bit, &frame, &frame_index, rx_data->received_symbols[i].duration1, rx_data->received_symbols[i].level1);
        if (frame_index == 8) {
            break;
        }
    }

    *data = frame;
    *size = frame_index;

    return ESP_OK;
}

gpio_num_t DaliBusClass::getRxPin()
{
    return dali_rxChannelConfig.gpio_num;
}

static void IRAM_ATTR dali_rmt_start_rx_receive(void* arg)
{
    DaliBusClass *daliClass = static_cast<DaliBusClass*>(arg);
    daliClass->setReceiving(true);
    xTaskAbortDelay(daliClass->rxTaskHandle);
    gpio_intr_disable(daliClass->getRxPin());
    rmt_receive(daliClass->getRxHandle(), daliClass->rawSymbols, sizeof(daliClass->rawSymbols), &(daliClass->dali_rxReceiveConfig));
}

static size_t dali_rmt_tx_encoder_cb(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg)
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
    printf("rmt_enable (tx):                 %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_ENABLE_TX;

    transmit_config = (rmt_transmit_config_t) {
        .loop_count = 0
    };

    
    dali_rxReceiveConfig = (rmt_receive_config_t) {
        .signal_range_min_ns = DALI_USTONS(2),
        .signal_range_max_ns = DALI_USTONS(DALI_THRESHOLD_2TE_HIGH),
    };

    dali_rxChannel = NULL;
    dali_rxChannelConfig = (rmt_rx_channel_config_t) {
        .gpio_num = (gpio_num_t)rx_pin,
        .clk_src = RMT_CLK_SRC_REF_TICK,
        .resolution_hz = DALI_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .flags = {
            .invert_in = true
        }
    };
    resp = rmt_new_rx_channel(&dali_rxChannelConfig, &dali_rxChannel);
    printf("rmt_new_rx_channel:              %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_CREATE_RX;
    
    dali_rxChannelQueue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
    printf("dali_rxChannelQueue:             %p\n", dali_rxChannelQueue);
     rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = dali_rmt_rx_callback,
    };
    resp = rmt_rx_register_event_callbacks(dali_rxChannel, &cbs, this);
    printf("rmt_rx_register_event_callbacks: %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_CREATE_RX;
        
    resp = rmt_enable(dali_rxChannel);
    printf("rmt_enable (rx):                 %d (%s)\n", resp, esp_err_to_name(resp));
    if(resp != ESP_OK)
        return DALI_ERR_ENABLE_RX;

    BaseType_t resp2 = xTaskCreate(dali_rmt_rx_task, "daliRX", 2048, this, 0, &rxTaskHandle);
    printf("xTaskCreate:                     %d (%s)\n", resp2, resp2 == pdPASS ? "pdPASS" : "pdFAILED");

    gpio_config_t io_conf = {};
    // Interrupt happens
    io_conf.intr_type = dali_rxChannelConfig.flags.invert_in ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1UL << rx_pin);
    //io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    // Configure the pin
    resp = gpio_config(&io_conf);
    printf("gpio_config:                     %d (%s)\n", resp, esp_err_to_name(resp));
    // Configure the interrupt
    resp = gpio_install_isr_service(0 /* No flags */); // Call this only once !!
    printf("gpio_install_isr_service:        %d (%s)\n", resp, esp_err_to_name(resp));
    resp = gpio_isr_handler_add(dali_rxChannelConfig.gpio_num, dali_rmt_start_rx_receive, this);
    printf("gpio_isr_handler_add:            %d (%s) - %d\n", resp, esp_err_to_name(resp), dali_rxChannelConfig.gpio_num);

    printf("daliClass:                       %p\n", this);

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
    gpio_intr_disable(getRxPin());
    lastResponse = DALI_RX_EMPTY;
    esp_err_t error = rmt_transmit(dali_txChannel, dali_txChannelEncoder, message, bits / 8, &transmit_config);
    printf("rmt_transmit:           %d (%s)\n", error, esp_err_to_name(error));
    if(error != ESP_OK)
    {
        isSending = false;
        // TODO add more handling of error
        return DALI_TX_ERROR;
    }
    error = rmt_tx_wait_all_done(dali_txChannel, 100);
    gpio_intr_enable(getRxPin());
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

    //wait at up to 22TE
    // - we receive a response first (if any)
    // - or wait time to the next forware frame
    vTaskDelay(pdMS_TO_TICKS(DALI_TE_TO_MS(22)));
    printf("transmit done\n");

    isSending = false;
    return DALI_NO_ERROR;
}

void DaliBusClass::setReceiving(bool value)
{
    isReceiving = value;
}

int DaliBusClass::getLastResponse()
{
    return lastResponse;
}

bool DaliBusClass::busIsIdle()
{
    return !isSending && !isReceiving;
}
//#endif