// ICM-42670-P over I2C, read directly by register. Registers are from TDK's
// datasheet. imu_init() checks WHO_AM_I, so a wrong address fails loudly.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "imu.h"

#define I2C_PORT I2C_NUM_0
#define PIN_SCL 39
#define PIN_SDA 40
#define TIMEOUT pdMS_TO_TICKS(20)

#define REG_SIGNAL_PATH_RESET 0x02
#define REG_ACCEL_DATA_X1 0x0B
#define REG_PWR_MGMT0 0x1F
#define REG_GYRO_CONFIG0 0x20
#define REG_ACCEL_CONFIG0 0x21
#define REG_WHO_AM_I 0x75
#define WHO_AM_I_VALUE 0x67

#define GYRO_2000DPS_400HZ 0x07
#define ACCEL_4G_400HZ 0x47
#define BOTH_LOW_NOISE 0x0F

#define G 9.80665f
#define DEG_TO_RAD 0.01745329252f
#define ACCEL_LSB_PER_G 8192.0f     // +/-4 g
#define GYRO_LSB_PER_DPS 16.4f      // +/-2000 deg/s

static uint8_t address;
static bool ready;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_PORT, address, buf, 2, TIMEOUT);
}

bool imu_init(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    const uint8_t addresses[] = {0x68, 0x69};
    for (int i = 0; i < 2 && !ready; i++) {
        uint8_t reg = REG_WHO_AM_I, who = 0;
        if (i2c_master_write_read_device(I2C_PORT, addresses[i], &reg, 1, &who, 1, TIMEOUT) == ESP_OK
            && who == WHO_AM_I_VALUE) {
            address = addresses[i];
            ready = true;
        }
    }
    if (!ready) return false;

    write_reg(REG_SIGNAL_PATH_RESET, 0x10);   // Soft reset
    vTaskDelay(pdMS_TO_TICKS(10));
    bool ok = write_reg(REG_GYRO_CONFIG0, GYRO_2000DPS_400HZ) == ESP_OK
           && write_reg(REG_ACCEL_CONFIG0, ACCEL_4G_400HZ) == ESP_OK
           && write_reg(REG_PWR_MGMT0, BOTH_LOW_NOISE) == ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(50));
    ready = ok;
    return ok;
}

bool imu_read(float accel_mps2[3], float gyro_rads[3])
{
    if (!ready) return false;
    uint8_t reg = REG_ACCEL_DATA_X1, raw[12];
    if (i2c_master_write_read_device(I2C_PORT, address, &reg, 1, raw, 12, TIMEOUT) != ESP_OK) return false;
    for (int i = 0; i < 3; i++) {
        int16_t a = (int16_t)((raw[2 * i] << 8) | raw[2 * i + 1]);
        int16_t g = (int16_t)((raw[6 + 2 * i] << 8) | raw[7 + 2 * i]);
        accel_mps2[i] = a / ACCEL_LSB_PER_G * G;
        gyro_rads[i] = g / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    }
    return true;
}
