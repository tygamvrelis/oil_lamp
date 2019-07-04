/**
 * @file MPU6050.c
 * @author Tyler
 * @brief Driver for MPU6050
 *
 * @ingroup MPU6050_Driver
 * @{
 */

/********************************** Includes *********************************/
#include "MPU6050/MPU6050.h"




/********************************* Constants *********************************/
/**
 * @brief Any I2C transfer will wait up to 1 ms on a semaphore before timing
 *        out
 */
static const TickType_t MAX_SEM_WAIT = 1;

/* @brief The slave address that should be used when trying to get the
 *        acceleration and gyroscope data from pg. 32 of the register map
 * @note  This is shifted 1 bit to the left because that's what ST's I2C HAL
 *        libraries expect, apparently
 */
static const uint8_t MPU6050_ACCEL_AND_GYRO_ADDR = 0x68 << 1;


// Register addresses for configuration stuff
// Accel and gyro addresses
#define SMPLRT_DIV 0x19       /**< Sample rate divider                       */
#define CONFIG 0x1A           /**< Can be used to configure the DLPF         */
#define GYRO_CONFIG 0x1B      /**< Can be used to change gyroscope range     */
#define ACCEL_CONFIG 0x1C     /**< Can be used to change accelerometer range */
#define ACCEL_CONFIG_2 0x1D   /**< Can be used to change gyroscope and
                               *   accelerometer data rate and bandwidth     */
#define I2C_MST_CTRL 0x24     /**< Used to set I2C clock speed               */
#define USER_CTRL 0x6A        /**< Used to enable I2C interface module       */
#define PWR_MGMT_1 0x6B       /**< Used to set the clock source for the accel
                               *   & gyro                                    */

#define PWR_MGMT_2 0x6C       /**< Used to force accelerometer and gyroscope
                               *   on                                        */

#define WHO_AM_I 0x75         /**< Should ALWAYS be 0x68 so this is a good
                               *   test of communication                     */

// Register addresses for accelerometer data (pg. 8 register map)
#define MPU6050_ACCEL_X_ADDR_H 0x3B /**< AXH */
#define MPU6050_ACCEL_X_ADDR_L 0x3C /**< AXL */
#define MPU6050_ACCEL_Y_ADDR_H 0x3D /**< AYH */
#define MPU6050_ACCEL_Y_ADDR_L 0x3E /**< AYL */
#define MPU6050_ACCEL_Z_ADDR_H 0x3F /**< AZH */
#define MPU6050_ACCEL_Z_ADDR_L 0x40 /**< AZL */


// Register addresses for gyroscope data (pg. 8 register map)
#define MPU6050_GYRO_X_ADDR_H 0x43 /**< VXH */
#define MPU6050_GYRO_X_ADDR_L 0x44 /**< VXL */
#define MPU6050_GYRO_Y_ADDR_H 0x45 /**< VYH */
#define MPU6050_GYRO_Y_ADDR_L 0x46 /**< VYL */
#define MPU6050_GYRO_Z_ADDR_H 0x47 /**< VZH */
#define MPU6050_GYRO_Z_ADDR_L 0x48 /**< VZL */


// Scales for readings
/** @brief Accelerometer full-scale range in m/s^2 */
static const float MPU6050_ACCEL_FULL_SCALE = 2 * 9.807;

/** @brief Gyroscope full-scale range in degrees/s */
static const float MPU6050_GYRO_FULL_SCALE = 250.0;


/********************************* Functions *********************************/
/**
 * @brief  Initializes the MPU6050 sensor by writing various settings to its
 *         registers, and reading a few of them for verification
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU6050 sensor
 * @param  hi2c I2C instance the sensor is attached to
 * @return 1 if successful, otherwise a negative error code
 */
int MPU6050Init(MPU6050_t* myMPU, I2C_HandleTypeDef* hi2c){
	myMPU->hi2c = hi2c;
    myMPU->az = NAN;
    myMPU->ay = NAN;
    myMPU->ax = NAN;
    myMPU->vz = NAN;
    myMPU->vy = NAN;
    myMPU->vx = NAN;

    uint8_t buff[1];

    // Use the best available clock source
    uint8_t dataToWrite = 0x01;
    if(HAL_I2C_Mem_Write(myMPU->hi2c, MPU6050_ACCEL_AND_GYRO_ADDR, PWR_MGMT_1,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -1;
    }

    // Set I2C module to use 400 kHz speed (pg. 19 of register map)
    dataToWrite = 0x0D;
    if(HAL_I2C_Mem_Write(myMPU->hi2c, MPU6050_ACCEL_AND_GYRO_ADDR, I2C_MST_CTRL,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -2;
    }

    // Check for bus communication essentially. If any function should fail and issue an early return, it would most likely
    // be this one.
    if(HAL_I2C_Mem_Read(myMPU->hi2c, MPU6050_ACCEL_AND_GYRO_ADDR, WHO_AM_I,
    I2C_MEMADD_SIZE_8BIT, buff, 1, 100) != HAL_OK){
        return -3;
    }

    // Check that the WHO_AM_I register is 0x68
    if(buff[0] != MPU6050_ACCEL_AND_GYRO_ADDR >> 1){
        return -4;
    }

    // Force accelerometer and gyroscope to ON
    dataToWrite = 0x00;
    if(HAL_I2C_Mem_Write(myMPU->hi2c, MPU6050_ACCEL_AND_GYRO_ADDR, PWR_MGMT_2,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -5;
    }

    /* Return success */
    return 1;
}

/**
 * @brief  Reset the inertial measurement unit using blocking IO
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU6050 sensor
 * @return None
 */
void resetIMUBlocking(MPU6050_t* myMPU){
    uint8_t dataToWrite = 0x80;
    HAL_I2C_Mem_Write(myMPU->hi2c, MPU6050_ACCEL_AND_GYRO_ADDR, PWR_MGMT_1,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100);
}

void attachSemaphore(MPU6050_t* myMPU, osSemaphoreId sem)
{
	myMPU->sem = sem;
}

int accelReadIT(MPU6050_t* myMPU){
    uint8_t mpu_buff[6];
    if(HAL_I2C_Mem_Read_DMA(myMPU->hi2c, MPU6050_ACCEL_AND_GYRO_ADDR,
            MPU6050_ACCEL_X_ADDR_H,
            I2C_MEMADD_SIZE_8BIT, mpu_buff, 6) != HAL_OK){
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmarx);
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmatx);
    	generateClocks(myMPU->hi2c, 1, 1);
        myMPU->ax = NAN;
        myMPU->ay = NAN;
        myMPU->az = NAN;
        return -1;
    }
    if(xSemaphoreTake(myMPU->sem, MAX_SEM_WAIT) != pdTRUE){
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmarx);
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmatx);
        myMPU->ax = NAN;
        myMPU->ay = NAN;
        myMPU->az = NAN;
        return -2;
    }

    // Process data; scale to physical units
    int16_t temp = (mpu_buff[0] << 8 | mpu_buff[1]);
    myMPU->ax = -1.0 * (temp * MPU6050_ACCEL_FULL_SCALE / (32767.0));

    temp = (mpu_buff[2] << 8 | mpu_buff[3]);
    myMPU->ay = -1.0 * (temp * MPU6050_ACCEL_FULL_SCALE / (32767.0));

    temp = (mpu_buff[4] << 8 | mpu_buff[5]);
    myMPU->az = -1.0 * (temp * MPU6050_ACCEL_FULL_SCALE / (32767.0));

    return 1;
}

int gyroReadIT(MPU6050_t* myMPU){
    uint8_t mpu_buff[6];
    if(HAL_I2C_Mem_Read_DMA(myMPU->hi2c, MPU6050_ACCEL_AND_GYRO_ADDR,
            MPU6050_GYRO_X_ADDR_H,
            I2C_MEMADD_SIZE_8BIT, mpu_buff, 6) != HAL_OK){
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmarx);
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmatx);
    	generateClocks(myMPU->hi2c, 1, 1);
        myMPU->vx = NAN;
        myMPU->vy = NAN;
        myMPU->vz = NAN;
        return -1;
    }
    if(xSemaphoreTake(myMPU->sem, MAX_SEM_WAIT) != pdTRUE){
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmarx);
        HAL_DMA_Abort_IT(myMPU->hi2c->hdmatx);
        myMPU->vx = NAN;
        myMPU->vy = NAN;
        myMPU->vz = NAN;
        return -2;
    }

    // Process data; scale to physical units
    int16_t temp = (mpu_buff[0] << 8 | mpu_buff[1]);
    myMPU->vx = (temp / (32767.0) * MPU6050_GYRO_FULL_SCALE);

    temp = (mpu_buff[2] << 8 | mpu_buff[3]);
    myMPU->vy = (temp / (32767.0) * MPU6050_GYRO_FULL_SCALE);

    temp = (mpu_buff[4] << 8 | mpu_buff[5]);
    myMPU->vz = (temp / (32767.0) * MPU6050_GYRO_FULL_SCALE);

    return 1;
}

imu_data_t get_data(MPU6050_t* myMPU)
{
	imu_data_t data;
	data.az = myMPU->az;
	data.ay = myMPU->ay;
	data.ax = myMPU->ax;
	data.vz = myMPU->vz;
	data.vy = myMPU->vy;
	data.vx = myMPU->vx;
	return data;
}

/**
 * @brief  Helper function for generateClocks
 * @param  port Pointer to the SDA port
 * @param  pin The SDA pin number
 * @param  state The state to compare SDA to (i.e. desired SDA state)
 * @param  timeout The number of ms to wait before leaving function
 * @return 1 if SDA pin is read to be state, 0 if timeout
 */
static uint8_t wait_for_gpio_state_timeout(
        GPIO_TypeDef* port,
        uint16_t pin,
        GPIO_PinState state,
        uint8_t timeout
)
{
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;

    // Wait until flag is set
    while ((state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret)){

        // Check for the timeout
        if((timeout == 0U) || (HAL_GetTick() - Tickstart >= timeout)){
            ret = 0;
        }
        asm("nop");
    }
    return ret;
}

/**
 * @brief This function bit-bangs the I2C master clock, with the option of
 *        sending stop bits
 * @details
 * This function is used as a workaround for an issue where the BUSY flag of
 * the I2C module is erroneously asserted in the hardware (a silicon bug,
 * essentially).
 *
 * Overall, use with EXTREME caution.
 *
 * Resources used for resolution:
 *  - https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour/281046#281046
 *  - https://community.st.com/thread/35884-cant-reset-i2c-in-stm32f407-to-release-i2c-lines
 *  - https://electronics.stackexchange.com/questions/272427/stm32-busy-flag-is-set-after-i2c-initialization
 *  - http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
 *
 * @param hi2c Pointer to the I2C handle corresponding to the sensor
 * @param numClocks The number of times to cycle the I2C master clock
 * @param sendStopBits 1 if stop bits are to be sent on SDA
 */
void generateClocks(
    I2C_HandleTypeDef* hi2c,
    uint8_t numClocks,
    uint8_t sendStopBits
)
{
    struct I2C_Module{
        I2C_HandleTypeDef* instance;
        uint16_t sdaPin;
        GPIO_TypeDef* sdaPort;
        uint16_t sclPin;
        GPIO_TypeDef* sclPort;
    };

    static struct I2C_Module i2c1module = {
		&hi2c1,
		IMU_BASE_SDA_Pin,
		IMU_BASE_SDA_GPIO_Port,
		IMU_BASE_SCL_Pin,
		IMU_BASE_SCL_GPIO_Port
    };

    static struct I2C_Module i2c3module = {
		&hi2c3,
		IMU_LAMP_SDA_Pin,
		IMU_LAMP_SDA_GPIO_Port,
		IMU_LAMP_SCL_Pin,
		IMU_LAMP_SCL_GPIO_Port,
    };

    struct I2C_Module* i2c = NULL;
    if(hi2c == &hi2c1){
        i2c = &i2c1module;
    }
    else{
        i2c = &i2c3module;
    }

    static uint8_t timeout = 1;
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_HandleTypeDef* handler = NULL;
    handler = i2c->instance;

    // 1. Clear PE bit.
    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_PE);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStructure.Pin = i2c->sclPin;
    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = i2c->sdaPin;
    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

    for(uint8_t i = 0; i < numClocks; i++){
        // 3. Check SCL and SDA High level in GPIOx_IDR.
        if(sendStopBits){
            HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);
        }
        HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

        wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET,
                timeout);
        if(sendStopBits){
            wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET,
                    timeout);
        }

        // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        if(sendStopBits){
            HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);
            wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin,
                    GPIO_PIN_RESET, timeout); // 5. Check SDA Low level in GPIOx_IDR
        }

        // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);
        wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET,
                timeout); // 7. Check SCL Low level in GPIOx_IDR.

        // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
        HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);
        wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET,
                timeout); // 9. Check SCL High level in GPIOx_IDR.

        // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
        if(sendStopBits){
            HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);
            wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET,
                    timeout); // 11. Check SDA High level in GPIOx_IDR.
        }
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C3;

    GPIO_InitStructure.Pin = i2c->sclPin;
    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = i2c->sdaPin;
    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(handler->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    HAL_I2C_Init(handler);
}

/**
 * @}
 */
/* end - MPU6050_Driver */
