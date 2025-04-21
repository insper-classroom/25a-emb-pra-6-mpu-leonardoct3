#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include "Fusion.h"

#define SAMPLE_PERIOD   (0.01f)   // 10 ms
#define MPU_ADDRESS     0x68
#define I2C_SDA_GPIO    4
#define I2C_SCL_GPIO    5

// UART configuration
#define UART_ID         uart0
#define UART_TX_PIN     0  // GP0 TX
#define UART_RX_PIN     1  // GP1 RX (se precisar)
#define UART_BAUDRATE   115200

// ----------------------------------------------------------------
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};  // PWR_MGMT_1 = 0
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;

    // Leitura acelerômetro
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // Leitura giroscópio
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // Leitura temperatura
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

// ----------------------------------------------------------------
void mpu6050_task(void *p) {
    // Inicializa I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Reset do sensor
    mpu6050_reset();

    int16_t rawAccel[3], rawGyro[3], rawTemp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    char outbuf[64];
    while (1) {
        mpu6050_read_raw(rawAccel, rawGyro, &rawTemp);

        FusionVector gyroscope = {
            .axis.x = rawGyro[0] / 131.0f,
            .axis.y = rawGyro[1] / 131.0f,
            .axis.z = rawGyro[2] / 131.0f,
        };
        FusionVector accelerometer = {
            .axis.x = rawAccel[0] / 16384.0f,
            .axis.y = rawAccel[1] / 16384.0f,
            .axis.z = rawAccel[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Monta string CSV: roll,pitch,yaw,click\n  
        int click = (accelerometer.axis.y > 2.0f) ? 1 : 0;
        int len = snprintf(outbuf, sizeof(outbuf), "%.2f,%.2f,%.2f,%d\n",
                           euler.angle.roll,
                           euler.angle.pitch,
                           euler.angle.yaw,
                           click);
        // Envia via UART
        uart_write_blocking(UART_ID, (uint8_t *)outbuf, len);

        vTaskDelay(pdMS_TO_TICKS(10));  // 10 ms
    }
}

int main() {
    // Inicializa stdio e UART
    stdio_init_all();
    uart_init(UART_ID, UART_BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    xTaskCreate(mpu6050_task, "mpu6050", 4096, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1) { tight_loop_contents(); }
    return 0;
}
