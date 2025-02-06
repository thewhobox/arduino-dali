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
#include <stdio.h>

#ifdef DALI_USE_ESP32
#undef DALI_DEBUG

#include "DaliBus_esp32.h"

static rmt_transmit_config_t transmit_config;

static bool dali_rmt_rx_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    // no printfs in this function, as it is called from an ISR
    BaseType_t high_task_wakeup = pdFALSE;
    DaliBusClass *daliClass = static_cast<DaliBusClass *>(user_data);
    xQueueSendFromISR(daliClass->getQueueHandle(), edata, &high_task_wakeup);
    gpio_intr_enable(daliClass->getRxPin());
    return high_task_wakeup == pdTRUE;
}

static void dali_rmt_rx_task(void *arg)
{
    DaliBusClass *daliClass = static_cast<DaliBusClass *>(arg);

    rmt_receive_config_t dali_rxReceiveConfig = (rmt_receive_config_t){
        .signal_range_min_ns = DALI_USTONS(2),
        .signal_range_max_ns = DALI_USTONS(DALI_THRESHOLD_2TE_HIGH),
    };
    rmt_rx_done_event_data_t rx_data;

    while (1)
    {
        if (xQueueReceive(daliClass->getQueueHandle(), &rx_data, pdMS_TO_TICKS(DALI_BACKWARD_FRAME_TIMEOUT_MS)) == pdPASS)
        {
            // printf("Received %d symbols\n", rx_data.num_symbols);
            uint32_t rxCommand = 0;
            size_t sizeInBits = 0;
            esp_err_t resp = daliClass->decode_symbols(&rx_data, &rxCommand, &sizeInBits);
            if (resp != ESP_OK)
            {
                // we could not decode it, so we ignore it
                daliClass->setReceiving(false);
                continue;
            }

            // printf("decode_symbols:                  %d (%s) - %.6X %i bits\n", resp, esp_err_to_name(resp), rxCommand, sizeInBits);
            if (sizeInBits == 8)
            {
                daliClass->lastResponse = rxCommand & 0xFF;
                // printf("Received response:               %.2X (%.8X\n", daliClass->lastResponse, daliClass->lastResponse);
            }

            if (daliClass->receivedCallback != 0)
            {
                uint8_t *data = new uint8_t[4];
                // handle support for 25 bit commands
                if (sizeInBits == 25)
                {
                    uint8_t temp = rxCommand & 0xFF;
                    rxCommand = (rxCommand >> 1) & 0xFFFF;
                    rxCommand |= temp;
                }

                uint8_t offset = sizeInBits - 8; // Start with bitlen - 8 for the first byte
                // Extract the first byte (always available if bitlen >= 16)
                data[0] = (rxCommand >> offset) & 0xFF;
                // Decrease offset and extract the second byte if bitlen >= 16
                offset -= 8;
                if (sizeInBits >= 16)
                {
                    data[1] = (rxCommand >> offset) & 0xFF;
                }
                else
                {
                    data[1] = 0; // Clear the second byte if it's not present
                }
                // Decrease offset and extract the third byte if bitlen >= 24
                offset -= 8;
                if (sizeInBits >= 24)
                {
                    data[2] = (rxCommand >> offset) & 0xFF;
                }
                else
                {
                    data[2] = 0; // Clear the third byte if it's not present
                }
                // printf("calling receiveCallback\n");
                daliClass->receivedCallback(data, sizeInBits);
                delete[] data;
            }

            // printf("receiving done\n");
            daliClass->setReceiving(false);
        }
    }
}



// Helper function to print a number in binary format
void print_binary(uint32_t num, uint8_t bits)
{
    for (int i = bits - 1; i >= 0; i--)
    {
        printf("%d", (num >> i) & 1);
    }
}

static esp_err_t dali_rmt_rx_decoder(uint32_t *frame, uint8_t *frame_index, uint16_t duration, bool level, bool *prev_level)
{
#ifdef DALI_DEBUG
    printf("Duration: %d, Level: %d, Prev Bit: %d\n", duration, level, *prev_level);
#endif

    if (duration == 0)
    {
        // this is the stop bit
        if (*prev_level == false && *frame_index % 2 != 0)
        {
            (*frame_index)++;
            // here we are missing the last 1,
            // since it is in the stop bit
            (*frame) <<= 1;
            (*frame) |= 1;
/*#ifdef DALI_DEBUG
            printf("1 -> Frame: ");
            print_binary(*frame, *frame_index / 2);
            printf(", Index: %d\n\n", *frame_index);
#endif*/
        }
        return ESP_FAIL; // means stop it
    }
    else if ((duration > DALI_USTORMT(DALI_THRESHOLD_1TE_LOW)) && (duration < DALI_USTORMT(DALI_THRESHOLD_1TE_HIGH)))
    {
        // short break (1 Te)
        (*frame_index)++;

        if (*frame_index % 2 == 0)
        {
            if (level == 1 && *prev_level == 0)
            {
                // this is a one
                // _/-\_/-
                //  ^
                (*frame) <<= 1;
                (*frame) |= 1;
#ifdef DALI_DEBUG
                printf("1 -> Frame: ");
                print_binary(*frame, *frame_index / 2);
                printf(", Index: %d\n\n", *frame_index);
#endif
            }
            if (level == 0 && *prev_level == 1)
            {
                // this is a zero
                // _/-\_/-
                //    ^
                (*frame) <<= 1;
#ifdef DALI_DEBUG
                printf("0 -> Frame: ");
                print_binary(*frame, *frame_index / 2);
                printf(", Index: %d\n\n", *frame_index);
#endif
            }
        }
        *prev_level = level;
        return ESP_OK;
    }
    else if ((duration > DALI_USTORMT(DALI_THRESHOLD_2TE_LOW)) && (duration < DALI_USTORMT(DALI_THRESHOLD_2TE_HIGH)))
    {
        // long break (2 Te)
        (*frame_index)++;

        if (*frame_index % 2 == 0)
        {
            if (level == 1 && *prev_level == 0)
            {
                // this is a one
                // _/--\_/-
                //   ^
                (*frame) <<= 1;
                (*frame) |= 1;
#ifdef DALI_DEBUG
                printf("1 -> Frame: ");
                print_binary(*frame, *frame_index / 2);
                printf(", Index: %d\n\n", *frame_index);
#endif
            }
            if (level == 0 && *prev_level == 1)
            {
                // this is a zero
                // _/-\__/-
                //    ^
                (*frame) <<= 1;
#ifdef DALI_DEBUG
                printf("0 -> Frame: ");
                print_binary(*frame, *frame_index / 2);
                printf(", Index: %d\n\n", *frame_index);
#endif
            }
        }
        // imaginary fill with 1 symbol
        (*frame_index)++;
        *prev_level = level;
    }

    return ESP_OK;
}

static void IRAM_ATTR dali_rmt_start_rx_receive(void *arg)
{
    // no printfs in this function, as it is called from an ISR
    DaliBusClass *daliClass = static_cast<DaliBusClass *>(arg);
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
    if (symbols_free < 18)
    {
        return 0;
    }

    // Send a start bit first.
    if (symbols_written == 0)
    {
        symbols[0] = DALI_SYMBOL_ONE;
        return 1;
    }

    // We can calculate where in the data we are from the symbol pos.
    // Divide symbols_written by 8 as each bit translates to one symbol.
    size_t data_pos = (symbols_written - 1) / 8;
    uint8_t *data_bytes = (uint8_t *)data;
    if (data_pos < data_size)
    {
        // Encode a byte
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1)
        {
            if (data_bytes[data_pos] & bitmask)
            {
                symbols[symbol_pos++] = DALI_SYMBOL_ONE;
            }
            else
            {
                symbols[symbol_pos++] = DALI_SYMBOL_ZERO;
            }
        }
        // We're done; we should have written 16 symbols.
        return symbol_pos;
    }
    else
    {
        // Command has been encoded.
        // Add stop bits, and we're done.
        symbols[0] = DALI_SYMBOL_STOP;
        *done = 1; // Indicate end of the transaction.
        return 1;  // We only wrote one symbol
    }
}

esp_err_t DaliBusClass::decode_symbols(rmt_rx_done_event_data_t *rx_data, uint32_t *data, size_t *size)
{
    uint32_t frame = 0;
    uint8_t frame_index = 0;
    bool prev_level = true;

#ifdef DALI_DEBUG
    for (size_t i = 0; i < rx_data->num_symbols; i++)
    {
        printf("Duration0: %d, Level0: %d\n", rx_data->received_symbols[i].duration0, rx_data->received_symbols[i].level0);
        printf("Duration1: %d, Level1: %d\n", rx_data->received_symbols[i].duration1, rx_data->received_symbols[i].level1);
    }
#endif

    if (rx_data->received_symbols[0].duration0 > 650)
    {
        // this is the start bit and the first bit!
        // so we give it a 1
        dali_rmt_rx_decoder(&frame, &frame_index, 417, 1, &prev_level);
    }

    // TODO dont depend on receiving only 8bits
    esp_err_t resp = dali_rmt_rx_decoder(&frame, &frame_index, rx_data->received_symbols[0].duration1, rx_data->received_symbols[0].level1, &prev_level);
    for (size_t i = 1; i < rx_data->num_symbols; i++)
    {

        resp = dali_rmt_rx_decoder(&frame, &frame_index, rx_data->received_symbols[i].duration0, rx_data->received_symbols[i].level0, &prev_level);
        if (!resp == ESP_OK)
        {
            break;
        }

        resp = dali_rmt_rx_decoder(&frame, &frame_index, rx_data->received_symbols[i].duration1, rx_data->received_symbols[i].level1, &prev_level);
        if (!resp == ESP_OK)
        {
            break;
        }
    }

    *data = frame;
    *size = frame_index / 2;

    return ESP_OK;
}

gpio_num_t DaliBusClass::getRxPin()
{
    return dali_rxChannelConfig.gpio_num;
}

int DaliBusClass::begin(byte tx_pin, byte rx_pin, bool active_low)
{
    esp_err_t resp = 0;

    dali_txChannel = NULL;

    dali_txChannelConfig = (rmt_tx_channel_config_t){
        .gpio_num = (gpio_num_t)tx_pin,
        .clk_src = RMT_CLK_SRC_REF_TICK,
        .resolution_hz = DALI_RMT_RESOLUTION_HZ, // 1us resolution
        .mem_block_symbols = 64,                 // Memory block size, 64 * 4 = 256 Bytes
        .trans_queue_depth = 3,                  // Number of transactions that can pend in the background
        .flags = {
            .invert_out = false,
            .with_dma = false, // ESP32 does not support DMA
        }};
    resp = rmt_new_tx_channel(&dali_txChannelConfig, &dali_txChannel);

    // printf("rmt_new_tx_channel:              %d (%s)\n", resp, esp_err_to_name(resp));
    if (resp != ESP_OK)
        return DALI_ERR_CREATE_TX;

    dali_txChannelEncoder = NULL;
    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = dali_rmt_tx_encoder_cb
        // Note we don't set min_chunk_size here as the default of 64 is good enough.
    };
    resp = rmt_new_simple_encoder(&simple_encoder_cfg, &dali_txChannelEncoder);
    // printf("rmt_new_simple_encoder:          %d (%s)\n", resp, esp_err_to_name(resp));
    if (resp != ESP_OK)
        return DALI_ERR_CREATE_ENCODER;

    resp = rmt_enable(dali_txChannel);
    // printf("rmt_enable (tx):                 %d (%s)\n", resp, esp_err_to_name(resp));
    if (resp != ESP_OK)
        return DALI_ERR_ENABLE_TX;

    transmit_config = (rmt_transmit_config_t){
        .loop_count = 0};

    dali_rxReceiveConfig = (rmt_receive_config_t){
        .signal_range_min_ns = DALI_USTONS(2),
        .signal_range_max_ns = DALI_USTONS(DALI_THRESHOLD_2TE_HIGH),
    };

    dali_rxChannel = NULL;
    dali_rxChannelConfig = (rmt_rx_channel_config_t){
        .gpio_num = (gpio_num_t)rx_pin,
        .clk_src = RMT_CLK_SRC_REF_TICK,
        .resolution_hz = DALI_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .flags = {
            .invert_in = true}};
    resp = rmt_new_rx_channel(&dali_rxChannelConfig, &dali_rxChannel);
    // printf("rmt_new_rx_channel:              %d (%s)\n", resp, esp_err_to_name(resp));
    if (resp != ESP_OK)
        return DALI_ERR_CREATE_RX;

    dali_rxChannelQueue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
    // printf("dali_rxChannelQueue:             %p\n", dali_rxChannelQueue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = dali_rmt_rx_callback,
    };
    resp = rmt_rx_register_event_callbacks(dali_rxChannel, &cbs, this);
    // printf("rmt_rx_register_event_callbacks: %d (%s)\n", resp, esp_err_to_name(resp));
    if (resp != ESP_OK)
        return DALI_ERR_CREATE_RX;

    resp = rmt_enable(dali_rxChannel);
    // printf("rmt_enable (rx):                 %d (%s)\n", resp, esp_err_to_name(resp));
    if (resp != ESP_OK)
        return DALI_ERR_ENABLE_RX;

    BaseType_t resp2 = xTaskCreate(dali_rmt_rx_task, "daliRX", 4096, this, 0, &rxTaskHandle);
    // printf("xTaskCreate:                     %d (%s)\n", resp2, resp2 == pdPASS ? "pdPASS" : "pdFAILED");

    gpio_config_t io_conf = {};
    // Interrupt happens
    io_conf.intr_type = dali_rxChannelConfig.flags.invert_in ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pin_bit_mask = (1UL << rx_pin);
    io_conf.pin_bit_mask = BIT64(dali_rxChannelConfig.gpio_num);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    // Configure the pin
    resp = gpio_config(&io_conf);
    // printf("gpio_config:                     %d (%s)\n", resp, esp_err_to_name(resp));
    //  Configure the interrupt
    resp = gpio_install_isr_service(0 /* No flags */); // Call this only once !!
    // printf("gpio_install_isr_service:        %d (%s)\n", resp, esp_err_to_name(resp));
    resp = gpio_isr_handler_add(dali_rxChannelConfig.gpio_num, dali_rmt_start_rx_receive, this);
    // printf("gpio_isr_handler_add:            %d (%s) - %d\n", resp, esp_err_to_name(resp), dali_rxChannelConfig.gpio_num);

    // printf("daliClass:                       %p\n", this);

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

daliReturnValue DaliBusClass::sendRaw(const byte *message, uint8_t bits)
{
    isSending = true;

    //printf("Sending %d bits: %.2X%.2X%.2X\n", bits, message[0], message[1], message[2]);
    gpio_intr_disable(getRxPin());
    lastResponse = DALI_RX_EMPTY;

    // handle support for sending 25bit commands
    uint8_t txmessage[4];
    memcpy(txmessage, message, 3);
    if (bits == 25)
    {
        txmessage[3] = (txmessage[2] & 1) << 7;
        txmessage[2] = (txmessage[2] >> 1) | 0b10000000;
    }

    esp_err_t error = rmt_transmit(dali_txChannel, dali_txChannelEncoder, txmessage, bits / 8, &transmit_config);
    // printf("rmt_transmit:                    %d (%s)\n", error, esp_err_to_name(error));
    if (error != ESP_OK)
    {
        isSending = false;
        // TODO add more handling of error
        return DALI_TX_ERROR;
    }
    error = rmt_tx_wait_all_done(dali_txChannel, 100);
    gpio_intr_enable(getRxPin());
    // printf("rmt_tx_wait_all_done:            %d (%s)\n", error, esp_err_to_name(error));
    if (error != ESP_OK)
    {
        isSending = false;
        if (error == ESP_ERR_INVALID_ARG)
            return DALI_INVALID_PARAMETER;
        else if (error == ESP_ERR_TIMEOUT)
            return DALI_SEND_TIMEOUT;
        return DALI_SEND_TIMEOUT;
    }

    // wait at up to 22TE
    //  - we receive a response first (if any)
    //  - or wait time to the next forware frame
    vTaskDelay(pdMS_TO_TICKS(DALI_TE_TO_MS(22)));
    //printf("transmit done\n");

    isSending = false;
    return DALI_SENT;
}

void DaliBusClass::setReceiving(bool value)
{
    isReceiving = value;
}

int DaliBusClass::getLastResponse()
{
    // printf("getLastResponse:                 %.2X (%i)\n", lastResponse, lastResponse);
    return lastResponse;
}

bool DaliBusClass::busIsIdle()
{
    return !isSending && !isReceiving;
}
#endif