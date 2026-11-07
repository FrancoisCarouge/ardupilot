#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

void RmtSigReader::init()
{
    rmt_rx_channel_config_t config {};
    config.gpio_num = HAL_ESP32_RCIN;
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = frequency;
    config.mem_block_symbols = max_pulses; //each block could store 64 pulses

    // 8 ticks of the 80MHz APB clock is a 100ns glitch filter, and the frame
    // ends after idle_threshold ticks of the 1MHz resolution
    receive_config.signal_range_min_ns = 100;
    receive_config.signal_range_max_ns = idle_threshold * (1000000000 / frequency);
    receive_config.flags.en_partial_rx = 0;

    // the receive done callback copies each frame into this ring buffer
    handle = xRingbufferCreate(max_pulses * 8, RINGBUF_TYPE_NOSPLIT);

    rmt_rx_event_callbacks_t callbacks {};
    callbacks.on_recv_done = on_recv_done;

    // on failure no pulses are read, as with no receiver connected
    if (handle == nullptr ||
        rmt_new_rx_channel(&config, &channel) != ESP_OK ||
        rmt_rx_register_event_callbacks(channel, &callbacks, this) != ESP_OK ||
        rmt_enable(channel) != ESP_OK ||
        rmt_receive(channel, rx_symbols, sizeof(rx_symbols), &receive_config) != ESP_OK) {
        return;
    }
    started = true;
}

// runs in ISR context: queue the received frame and start the next reception
bool RmtSigReader::on_recv_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    RmtSigReader *reader = (RmtSigReader *)user_ctx;
    BaseType_t task_woken = pdFALSE;
    xRingbufferSendFromISR(reader->handle, edata->received_symbols,
                           edata->num_symbols * sizeof(rmt_symbol_word_t), &task_woken);
    rmt_receive(channel, reader->rx_symbols, sizeof(reader->rx_symbols), &reader->receive_config);
    return task_woken == pdTRUE;
}

bool RmtSigReader::add_item(uint32_t duration, bool level)
{
    bool has_more = true;
    if (duration == 0) {
        has_more = false;
        duration = idle_threshold;
    }
    if (level) {
        if (last_high == 0) {
            last_high = duration;
        }
    } else {
        if (last_high != 0) {
            ready_high = last_high;
            ready_low = duration;
            pulse_ready = true;
            last_high = 0;
        }
    }
    return has_more;
}

bool RmtSigReader::read(uint32_t &width_high, uint32_t &width_low)
{
    if (!started) {
        return false;
    }
    if (item == nullptr) {
        item = (rmt_symbol_word_t*) xRingbufferReceive(handle, &item_size, 0);
        item_size /= sizeof(rmt_symbol_word_t);
        current_item = 0;
    }
    if (item == nullptr) {
        return false;
    }
    bool buffer_empty = (current_item == item_size);
    buffer_empty = buffer_empty ||
                   !add_item(item[current_item].duration0, item[current_item].level0);
    buffer_empty = buffer_empty ||
                   !add_item(item[current_item].duration1, item[current_item].level1);
    current_item++;
    if (buffer_empty) {
        vRingbufferReturnItem(handle, (void*) item);
        item = nullptr;
    }
    if (pulse_ready) {
        width_high = ready_high;
        width_low = ready_low;
        pulse_ready = false;
        return true;
    }
    return false;
}
#endif
