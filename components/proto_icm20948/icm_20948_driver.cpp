//
// Created by aron on 2025-04-30.
//


#include <esp_check.h>
#include <esp_timer.h>

#include "icm_20948_driver.h"
#include "imu_types.h"
#include "imu.h"
#include "measure.hpp"

using motion::Imu;
using motion::Accel;
using motion::Gyro;
using motion::Quat6;
using motion::Quat9;
using motion::Stats;
using namespace proto;

inline icm20948_status_e writeBank0(icm20948_device_t* dev, uint8_t reg, uint8_t value) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(icm20948_set_bank(dev, 0));
    return icm20948_execute_w(dev, reg, &value, 1);
}

Icm20948Driver::Icm20948Driver(const Config &config) : cfg_(config) {
}

esp_err_t Icm20948Driver::init() {
    ESP_LOGI(TAG, "Initializing IMU on SPI bus (MISO:%d, MOSI:%d, SCK:%d, CS:%d)",
             cfg_.spi_miso, cfg_.spi_mosi,
             cfg_.spi_clock_hz, cfg_.spi_cs);

    ESP_RETURN_ON_ERROR(configureSpi(), TAG, "Failed to configure SPI");

    ESP_RETURN_ON_ERROR(configureSensor(), TAG, "Failed to configure sensor");

    ESP_RETURN_ON_ERROR(configureDmp(), TAG, "Failed to configure DMP");

    ESP_RETURN_ON_ERROR(configureInterruptPin(), TAG, "Failed to configure interrupt pin");

    int_queue_ = xQueueCreate(cfg_.queue_len, sizeof(uint32_t));
    if (!int_queue_) return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t Icm20948Driver::start() {
    if (task_) {
        return ESP_OK;
    }

    // Enable GPIO interrupt before task to not miss first edge (might not be needed)
    gpio_intr_enable(cfg_.int_gpio);

    BaseType_t task_created = xTaskCreatePinnedToCore(
        taskEntry,
        "icm20948_task",
        cfg_.task_stack,
        this,
        cfg_.task_prio,
        &task_,
        1
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "IMU Started");
    return ESP_OK;
}

void Icm20948Driver::stop() {
    if (!task_) return;

    vTaskDelete(task_);
    task_ = nullptr;
    gpio_intr_disable(cfg_.int_gpio);
    if (int_queue_) {
        vQueueDelete(int_queue_);
        int_queue_ = nullptr;
    }
}

esp_err_t Icm20948Driver::configureSpi() {
    spi_bus_config_t spi_bus_config{};
    spi_bus_config.miso_io_num = cfg_.spi_miso;
    spi_bus_config.mosi_io_num = cfg_.spi_mosi;
    spi_bus_config.sclk_io_num = cfg_.spi_sck;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 64;

    ESP_RETURN_ON_ERROR(spi_bus_initialize(cfg_.spi_host, &spi_bus_config, SPI_DMA_CH_AUTO), TAG,
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
    ESP_RETURN_ON_ERROR(spi_bus_add_device(cfg_.spi_host, &device_interface_config, &spi_), TAG,
                        "Failed to add device");

    icm20948_init_spi(&device_, &spi_);
    return ESP_OK;
}

esp_err_t Icm20948Driver::configureInterruptPin() {
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << cfg_.int_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, // INT is active-low, edge trig
    };

    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), TAG, "gpio config");

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(cfg_.int_gpio, isrThunk, this), TAG, "isr add");

    gpio_intr_disable(cfg_.int_gpio);

    return ESP_OK;
}

esp_err_t Icm20948Driver::configureSensor() {
    // constexpr int MAX_RETRIES = 5;
    // int retry_count = 0;
    //
    // // ID
    // while (retry_count <= MAX_RETRIES && icm20948_check_id(&device_) != ICM_20948_STAT_OK) {
    //     ESP_LOGW(TAG, "ID check failed, attempt %d of %d", retry_count + 1, MAX_RETRIES);
    //     retry_count++;
    //     if (retry_count < MAX_RETRIES) {
    //         vTaskDelay(pdMS_TO_TICKS(10));
    //     }
    // }
    //
    // if (retry_count >= MAX_RETRIES) {
    //     ESP_LOGE(TAG, "ID check failed after %d attempts", MAX_RETRIES);
    //     return ESP_ERR_NOT_FOUND;
    // }
    // ESP_LOGI(TAG, "ICM20948 check id passed");
    //
    // // WHOAMI
    // icm20948_status_e status = ICM_20948_STAT_ERR;
    // uint8_t whoami = 0x00;
    // retry_count = 0;
    // while (retry_count <= MAX_RETRIES && ((status != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI))) {
    //     whoami = 0x00;
    //     status = icm20948_get_who_am_i(&device_, &whoami);
    //     if (status != ICM_20948_STAT_OK || whoami != ICM_20948_WHOAMI) {
    //         ESP_LOGW(TAG, "whoami does not match (0x%02x). Retrying...", whoami);
    //         retry_count++;
    //
    //         vTaskDelay(pdMS_TO_TICKS(10));
    //     } else {
    //         break;
    //     }
    // }
    //
    // if (retry_count >= MAX_RETRIES) {
    //     ESP_LOGE(TAG, "whoami check failed after %d attempts, last value: 0x%02x", MAX_RETRIES, whoami);
    //     return ESP_ERR_NOT_FOUND;
    // } else {
    //     ESP_LOGI(TAG, "ICM20948 whoami passed: 0x%02x", whoami);
    // }

    icm20948_sw_reset(&device_);
    vTaskDelay(pdMS_TO_TICKS(100));

    // NOTE: this is kinda important, i mean what moron would try to configure it without waking up the device FIRST.
    icm20948_sleep(&device_, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    icm20948_low_power(&device_, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    // constexpr auto sensors = static_cast<icm20948_internal_sensor_id_bm>(
    //     ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
    //
    // icm20948_fss_t myfss{};
    // myfss.a = cfg_.accel_fsr;
    // myfss.g = cfg_.gyro_fsr;
    // icm20948_set_full_scale(&device_, sensors, myfss);

    // Set full scale ranges using direct register access, wont work otherwise, i dont like it either.
    esp_err_t err = setFullScaleRanges();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set full scale ranges");
        return err;
    }

    return ESP_OK;
}

// Curtesy of claude 3.5
esp_err_t Icm20948Driver::setFullScaleRanges() {
    icm20948_set_bank(&device_, 2);

    icm20948_accel_config_t accel_config;
    icm20948_status_e status = icm20948_execute_r(&device_, AGB2_REG_ACCEL_CONFIG,
                                                  reinterpret_cast<uint8_t *>(&accel_config),
                                                  sizeof(accel_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer config");
        return ESP_FAIL;
    }

    accel_config.ACCEL_FS_SEL = cfg_.accel_fsr;
    status = icm20948_execute_w(&device_, AGB2_REG_ACCEL_CONFIG,
                                reinterpret_cast<uint8_t *>(&accel_config),
                                sizeof(accel_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to write accelerometer FSR");
        return ESP_FAIL;
    }

    // NOTE: mgith need to have dlayed here
    vTaskDelay(pdMS_TO_TICKS(100)); // Give it time to apply

    icm20948_gyro_config_1_t gyro_config;
    status = icm20948_execute_r(&device_, AGB2_REG_GYRO_CONFIG_1,
                                reinterpret_cast<uint8_t *>(&gyro_config),
                                sizeof(gyro_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope config");
        return ESP_FAIL;
    }

    gyro_config.GYRO_FS_SEL = cfg_.gyro_fsr;
    status = icm20948_execute_w(&device_, AGB2_REG_GYRO_CONFIG_1,
                                reinterpret_cast<uint8_t *>(&gyro_config),
                                sizeof(gyro_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to write gyroscope FSR");
        return ESP_FAIL;
    }

    // NOTE: might need to have delay here
    vTaskDelay(pdMS_TO_TICKS(100)); // Give it time to apply

    return ESP_OK;
}

esp_err_t Icm20948Driver::configureDmp() {
    bool success = true;

    success &= icm20948_init_dmp_sensor_with_defaults(&device_) == ICM_20948_STAT_OK;

    success &= inv_icm20948_enable_dmp_sensor(&device_, INV_ICM20948_SENSOR_ACCELEROMETER, 1) ==
            ICM_20948_STAT_OK;
    success &= inv_icm20948_enable_dmp_sensor(&device_, INV_ICM20948_SENSOR_GYROSCOPE, 1) ==
            ICM_20948_STAT_OK;
    success &= inv_icm20948_enable_dmp_sensor(&device_, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1) ==
            ICM_20948_STAT_OK;
    success &= inv_icm20948_enable_dmp_sensor(&device_, INV_ICM20948_SENSOR_ROTATION_VECTOR, 1) ==
            ICM_20948_STAT_OK;

    // Set the ODR registers and clear the ODR counter

    // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate (225Hz) / ODR ) - 1
    // E.g. For a 25Hz ODR rate, value= (225/25) -1 = 8.

    success &= inv_icm20948_set_dmp_sensor_period(&device_, DMP_ODR_Reg_Accel, 224) == ICM_20948_STAT_OK;
    success &= inv_icm20948_set_dmp_sensor_period(&device_, DMP_ODR_Reg_Gyro, 0) == ICM_20948_STAT_OK;
    success &= inv_icm20948_set_dmp_sensor_period(&device_, DMP_ODR_Reg_Quat6, 224) == ICM_20948_STAT_OK;
    success &= inv_icm20948_set_dmp_sensor_period(&device_, DMP_ODR_Reg_Quat9, 224) == ICM_20948_STAT_OK;

    uint16_t wm = 56;
    uint8_t wm_le[2] = {static_cast<uint8_t>(wm & 0xFF),
                        static_cast<uint8_t>(wm >> 8)};
    success &= inv_icm20948_write_mems(&device_, FIFO_WATERMARK, 2, wm_le) == ICM_20948_STAT_OK;

    success &= icm20948_enable_fifo(&device_, true) == ICM_20948_STAT_OK;
    success &= icm20948_enable_dmp(&device_, true) == ICM_20948_STAT_OK;

    icm20948_int_enable_t int_en{};
    int_en.RAW_DATA_0_RDY_EN = 1;
    success &= icm20948_int_enable(&device_, &int_en, nullptr) == ICM_20948_STAT_OK;

    success &= icm20948_reset_fifo(&device_) == ICM_20948_STAT_OK;

    if (!success) {
        ESP_LOGE(TAG, "DMP initialization failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DMP initialized successfully");
    return ESP_OK;
}


void Icm20948Driver::isrThunk(void *arg) {
    static_cast<Icm20948Driver *>(arg)->isr();
}

void Icm20948Driver::isr() {
    const uint32_t token = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (int_queue_) {
        xQueueSendFromISR(int_queue_, &token, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void Icm20948Driver::taskEntry(void *arg) {
    static_cast<Icm20948Driver *>(arg)->run();
}

void Icm20948Driver::run() {
    icm_20948_DMP_data_t dmp_data{};
    uint32_t token;

    static profiler::Measure measure;

    for (;;) {
        if (xQueueReceive(int_queue_, &token, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        measure.start();

        icm20948_status_e st;
        do {
            st = inv_icm20948_read_dmp_data(&device_, &dmp_data);
            if (st == ICM_20948_STAT_OK || st == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
                if (dmp_data.header & DMP_header_bitmap_Accel) {
                    Accel accel{};
                    accel.x = static_cast<int16_t>(dmp_data.Raw_Accel.Data.X - 79);
                    accel.y = static_cast<int16_t>(dmp_data.Raw_Accel.Data.Y + 30);
                    accel.z = static_cast<int16_t>(dmp_data.Raw_Accel.Data.Z + 0);
                    Imu::instance().update(accel);
                }

                if (dmp_data.header & DMP_header_bitmap_Gyro_Calibr) {
                    Gyro gyro{};

                    gyro.x = static_cast<int16_t>(dmp_data.Raw_Gyro.Data.X - 3);
                    gyro.y = static_cast<int16_t>(dmp_data.Raw_Gyro.Data.Y - 8);
                    gyro.z = static_cast<int16_t>(dmp_data.Raw_Gyro.Data.Z - 0);
                    Imu::instance().update(gyro);
                }

                if (dmp_data.header & DMP_header_bitmap_Quat6) {
                    Quat6 quat6{};
                    quat6.x = dmp_data.Quat6.Data.Q1;
                    quat6.y = dmp_data.Quat6.Data.Q2;
                    quat6.z = dmp_data.Quat6.Data.Q3;
                    Imu::instance().update(quat6);
                }

                if (dmp_data.header & DMP_header_bitmap_Quat9) {
                    Quat9 quat9{};
                    quat9.x = dmp_data.Quat9.Data.Q1;
                    quat9.y = dmp_data.Quat9.Data.Q2;
                    quat9.z = dmp_data.Quat9.Data.Q3;
                    quat9.accuracy = dmp_data.Quat9.Data.Accuracy;
                    Imu::instance().update(quat9);
                }
            }
        } while (st == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL);

        measure.end();
        static size_t count = 0;
        if (++count % 100 == 0) {
            count = 0;
            Stats stats;
            stats.average_freq_mill_hz = measure.average_freq_mill_hz();
            stats.duty_cycle_permille = measure.duty_cycle_permille();

            vTaskGetRunTimeStats(stats.run_time_stats);
            Imu::instance().update(stats);
        }
    }
}

