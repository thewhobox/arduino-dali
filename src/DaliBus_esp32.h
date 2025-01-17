#pragma once

#include "DaliBus.h"
#ifdef DALI_USE_ESP32
#include "arduino.h"
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <esp_err.h>

#define DALI_RMT_RESOLUTION_HZ 1000000
#define DALI_USTORMT(x) ((x) * (DALI_RMT_RESOLUTION_HZ / 1000000))
#define DALI_USTONS(x)  (x * 1000)
#define DALI_ONE_TE 416

static const rmt_symbol_word_t DALI_SYMBOL_ONE = {
	.duration0 = DALI_USTORMT(DALI_ONE_TE),
	.level0 = 1,
	.duration1 = DALI_USTORMT(DALI_ONE_TE),
	.level1 = 0,
};

static const rmt_symbol_word_t DALI_SYMBOL_ZERO = {
	.duration0 = DALI_USTORMT(DALI_ONE_TE),
	.level0 = 0,
	.duration1 = DALI_USTORMT(DALI_ONE_TE),
	.level1 = 1,
};

static const rmt_symbol_word_t DALI_SYMBOL_STOP = {
	.duration0 = DALI_USTORMT(DALI_ONE_TE) * 2,
	.level0 = 0,
	.duration1 = DALI_USTORMT(DALI_ONE_TE) * 2,
	.level1 = 0,
};

class DaliBusClass
{
public:
	int begin(byte tx_pin, byte rx_pin, bool active_low = true);
	daliReturnValue sendRaw(const byte *message, uint8_t bits);

	int getLastResponse();

	bool busIsIdle();
	volatile byte busIdleCount;

	EventHandlerReceivedDataFuncPtr receivedCallback;
	EventHandlerActivityFuncPtr activityCallback;
	EventHandlerErrorFuncPtr errorCallback;

	// TODO remove temp
	bool tempBusLevel = false;
	uint16_t tempDelta = 0;

private:
	rmt_channel_handle_t dali_rxChannel;
	rmt_rx_channel_config_t dali_rxChannelConfig;
	QueueHandle_t dali_rxChannelQueue;
	rmt_channel_handle_t dali_txChannel;
	rmt_tx_channel_config_t dali_txChannelConfig;
	rmt_encoder_handle_t dali_txChannelEncoder;

	bool isSending = false;
};

#endif