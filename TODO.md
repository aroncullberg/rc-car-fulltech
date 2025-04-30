# Write our own NMEA parser to be able to ditch TinyGps++-espidf

# Write a component to time the frequency of a task, with some smoothing of the values not just raw values
a start:

```c++
uint64_t prev_us = 0;

// stuff 

uint64_t now_us = esp_timer_get_time();
uint64_t now_us = esp_timer_get_time();
if (prev_us) {
    uint64_t delta_us = now_us - prev_us;
    float freq_hz = 1e6f / delta_us;
    ESP_LOGI(TAG, "(%04lld ms) %-4.1f Hz ", delta_us / 1000, freq_hz);
}
prev_us = now_us;
```