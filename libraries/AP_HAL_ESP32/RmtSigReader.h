/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <AP_HAL/utility/RingBuffer.h>
#include "AP_HAL_ESP32.h"
#include "driver/rmt_rx.h"
#include "freertos/ringbuf.h"

class ESP32::RmtSigReader
{
public:
    static const int frequency = 1000000;  //1MHZ
    static const int max_pulses = 128;
    static const int idle_threshold = 3000;  //we require at least 3ms gap between frames
    void init();
    bool read(uint32_t &width_high, uint32_t &width_low);
private:
    bool add_item(uint32_t duration, bool level);
    static bool on_recv_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);

    rmt_channel_handle_t channel;
    rmt_receive_config_t receive_config;
    rmt_symbol_word_t rx_symbols[max_pulses];
    RingbufHandle_t handle;
    rmt_symbol_word_t* item;
    size_t item_size;
    size_t current_item;

    uint32_t last_high;
    uint32_t ready_high;
    uint32_t ready_low;
    bool pulse_ready;
    bool started;
};
