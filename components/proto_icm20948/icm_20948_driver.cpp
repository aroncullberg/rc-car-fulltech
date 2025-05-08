//
// Created by aron on 2025-04-30.
//


#include <esp_check.h>
#include <esp_timer.h>

#include "icm_20948_driver.h"
#include "imu_types.h"
#include "imu.hpp"


using motion::Imu;
using motion::Accel;
using motion::Gyro;
using namespace proto;

static constexpr auto tag = "Icm20948Driver";

constexpr static DRAM_ATTR int16_t kAccelBiasY =  -300;
constexpr static DRAM_ATTR int16_t kAccelBiasZ =  +16600;
constexpr static DRAM_ATTR int16_t kGyroBiasX =  -25;
constexpr static DRAM_ATTR int16_t kGyroBiasY =  -50;

Icm20948Driver::Icm20948Driver(const Config &config) : cfg_(config) {
}

esp_err_t Icm20948Driver::init() {
    ESP_LOGI(tag, "Initializing IMU on SPI bus (MISO:%d, MOSI:%d, SCK:%d, CS:%d)",
             cfg_.spi_miso, cfg_.spi_mosi,
             cfg_.spi_clock_hz, cfg_.spi_cs);

    ESP_RETURN_ON_ERROR(configure_spi(), tag, "Failed to configure SPI");

    ESP_RETURN_ON_ERROR(configure_sensor(), tag, "Failed to configure sensor");

    ESP_RETURN_ON_ERROR(configure_interrupt_pin(), tag, "Failed to configure interrupt pin");

    Imu::instance().set_accel_fsr(cfg_.accel_fsr);
    Imu::instance().set_gyro_fsr(cfg_.gyro_fsr);

    return ESP_OK;
}

esp_err_t Icm20948Driver::start() {
    if (task_) {
        return ESP_OK;
    }

    gpio_intr_enable(cfg_.int_gpio);

    BaseType_t task_created = xTaskCreatePinnedToCore(
        task_entry,
        "icm20948_task",
        cfg_.task_stack,
        this,
        cfg_.task_prio,
        &task_,
        1
    );

    if (task_created != pdPASS) {
        ESP_LOGE(tag, "Failed to create IMU task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(tag, "IMU Started");
    return ESP_OK;
}

void Icm20948Driver::stop() {
    if (!task_) return;

    vTaskDelete(task_);
    task_ = nullptr;
    gpio_intr_disable(cfg_.int_gpio);
}

esp_err_t Icm20948Driver::configure_spi() {
    spi_bus_config_t spi_bus_config{};
    spi_bus_config.miso_io_num = cfg_.spi_miso;
    spi_bus_config.mosi_io_num = cfg_.spi_mosi;
    spi_bus_config.sclk_io_num = cfg_.spi_sck;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    // spi_bus_config.max_transfer_sz = 64;

    ESP_RETURN_ON_ERROR(spi_bus_initialize(cfg_.spi_host, &spi_bus_config, SPI_DMA_CH_AUTO), tag,
                        "Failed to initialize SPI bus");

    spi_device_interface_config_t device_interface_config{};
        device_interface_config.command_bits = 0;
        device_interface_config.address_bits = 0;
        device_interface_config.dummy_bits = 0;
        device_interface_config.mode = 0;
        device_interface_config.duty_cycle_pos = 128;
        device_interface_config.cs_ena_posttrans = 0;
        device_interface_config.cs_ena_pretrans = 0;
        device_interface_config.clock_speed_hz = cfg_.spi_clock_hz;
        device_interface_config.spics_io_num = cfg_.spi_cs;
        device_interface_config.flags = 0;
        device_interface_config.queue_size = cfg_.queue_len;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(cfg_.spi_host, &device_interface_config, &spi_), tag,
                        "Failed to add device");

    icm20948_init_spi(&device_, &spi_);
    return ESP_OK;
}

esp_err_t Icm20948Driver::configure_interrupt_pin() {
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << cfg_.int_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, // INT is active-low, edge trig
    };

    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), tag, "gpio config");

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(cfg_.int_gpio, isr_thunk, this), tag, "isr add");

    gpio_intr_disable(cfg_.int_gpio);

    icm20948_enable_fifo(&device_, true);

    icm20948_int_enable_t int_en{};
    int_en.RAW_DATA_0_RDY_EN = 1;
    ESP_RETURN_ON_FALSE(icm20948_int_enable(&device_, &int_en, nullptr) == ICM_20948_STAT_OK, ESP_FAIL,
                        tag, "Failed to enable interrupt");

    return ESP_OK;
}

esp_err_t Icm20948Driver::configure_sensor() {
    icm20948_sw_reset(&device_);
    vTaskDelay(pdMS_TO_TICKS(100));

    icm20948_sleep(&device_, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    icm20948_low_power(&device_, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    constexpr auto sensors = static_cast<icm20948_internal_sensor_id_bm>(
        ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

    icm20948_set_sample_mode(&device_, sensors, SAMPLE_MODE_CONTINUOUS);

    // fODR = 1125Hz / (DIV + 1) = 1125 / (3 + 1) = 281.25Hz
    icm20948_smplrt_t smplrt{};
    smplrt.a = 2;
    smplrt.g = 2;
    icm20948_set_sample_rate(&device_, static_cast<icm20948_internal_sensor_id_bm>(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR), smplrt);

    icm20948_fss_t myfss{};
    myfss.a = cfg_.accel_fsr;
    myfss.g = cfg_.gyro_fsr;
    icm20948_set_full_scale(&device_, sensors, myfss);

    icm20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = ACC_D23BW9_N34BW4;
    myDLPcfg.g = GYR_D23BW9_N35BW9;
    icm20948_set_dlpf_cfg(&device_, sensors, myDLPcfg);

    icm20948_enable_dlpf(&device_, ICM_20948_INTERNAL_ACC, true);
    icm20948_enable_dlpf(&device_, ICM_20948_INTERNAL_GYR, true);

    return ESP_OK;
}


void Icm20948Driver::isr_thunk(void *arg) {
    static_cast<Icm20948Driver *>(arg)->isr();
}

void Icm20948Driver::isr() {
    const uint32_t token = esp_timer_get_time();
    BaseType_t x_higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(task_, token, eSetValueWithoutOverwrite, &x_higher_priority_task_woken);
    if (x_higher_priority_task_woken) portYIELD_FROM_ISR();
}

void Icm20948Driver::task_entry(void *arg) {
    static_cast<Icm20948Driver *>(arg)->run();
}

[[noreturn]] void Icm20948Driver::run() {
    uint32_t token;

    static uint64_t last_accel_us = 0;
    static uint64_t last_gyro_us  = 0;

    for (;;) {
        xTaskNotifyWait(0, 0, &token, portMAX_DELAY);

        icm20948_status_e st;
        do {
            icm20948_agmt_t agmt;
            st = icm20948_get_agmt(&device_, &agmt);
            if (st == ICM_20948_STAT_OK || st == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
                Accel accel{};
                accel.x = agmt.acc.axes.x * get_accel_scale() ;
                accel.y = (agmt.acc.axes.y + kAccelBiasY) * get_accel_scale();
                accel.z =( agmt.acc.axes.z + kAccelBiasZ) * get_accel_scale();

                accel.frequency = last_accel_us ? 1e6f / (token - last_accel_us) : 0.0f;
                last_accel_us = token;

                Imu::instance().update(accel);

                Gyro gyro{};
                gyro.x = (agmt.gyr.axes.x + kGyroBiasX)* get_gyro_scale();
                gyro.y = (agmt.gyr.axes.y + kGyroBiasY) * get_gyro_scale();
                gyro.z = agmt.gyr.axes.z* get_gyro_scale();
                gyro.frequency = last_gyro_us ? 1e6f / (token - last_gyro_us) : 0.0f;
                last_gyro_us = token;
                Imu::instance().update(gyro);
            }
        } while (st == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL);
    }
}
