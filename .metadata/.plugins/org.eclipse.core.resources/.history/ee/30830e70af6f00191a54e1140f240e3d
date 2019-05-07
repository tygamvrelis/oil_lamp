/**
 * @file MPU9250.c
 * @author Tyler
 * @brief Driver for MPU9250
 *
 * @defgroup MPU9250_Driver MPU9250 Driver
 * @brief Library for interfacing with MPU9250
 * @{
 */

/********************************** Includes *********************************/
#include "MPU9250.h"




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
static const uint8_t MPU9250_ACCEL_AND_GYRO_ADDR = 0x68 << 1;

/* @brief The slave address that should be used when trying to get the
 *        magnetometer data from pg. 24 of the register map
 * @note  This is shifted 1 bit to the left because that's what ST's I2C HAL
 *        libraries expect, apparently
 */
static const uint8_t MPU9250_MAG_ADDR = 0x0C << 1;


// Register addresses for configuration stuff
// Accel and gyro addresses
#define SMPLRT_DIV 0x19       /**< Sample rate divider                       */
#define CONFIG 0x1A           /**< Can be used to configure the DLPF         */
#define GYRO_CONFIG 0x1B      /**< Can be used to change gyroscope range     */
#define ACCEL_CONFIG 0x1C     /**< Can be used to change accelerometer range */
#define ACCEL_CONFIG_2 0x1D   /**< Can be used to change gyroscope and
                               *   accelerometer data rate and bandwidth     */

#define I2C_MST_CTRL 0x24     /**< Used to set I2C clock speed               */
#define I2C_SLV0_ADDR 0x25    /**< Physical address of I2C slave 0           */
#define I2C_SLV0_REG 0x26     /**< Slave 0 register from where to begin data
                               *   transfer                                  */

#define I2C_SLV0_CTRL 0x27    /**< Control register for data transactions
                               *   with slave 0                              */

#define I2C_SLV1_ADDR 0x2B    /**< Physical address of I2C slave 1           */
#define I2C_SLV1_REG 0x2C     /**< Slave 1 register from where to begin data
                               *   transfer                                  */

#define I2C_SLV1_CTRL 0x2D    /**< Control register for data transactions
                               *   with slave 1                              */

#define INT_PIN_CFG 0x37      /**< Used to configure I2C bypass to
                               *   external sensor (i.e. magnetometer)       */

#define EXT_SENS_DATA_00 0x49 /**< Holds data from external sensors (i.e.
                               *   magnetometer)                             */

#define I2C_SLV0_DO 0x63      /**< Data out when writing to slave 0          */
#define I2C_SLV1_DO 0x64      /**< Data out when writing to slave 1          */
#define USER_CTRL 0x6A        /**< Used to enable I2C interface module       */
#define PWR_MGMT_1 0x6B       /**< Used to set the clock source for the accel
                               *   & gyro                                    */

#define PWR_MGMT_2 0x6C       /**< Used to force accelerometer and gyroscope
                               *   on                                        */

#define WHO_AM_I 0x75         /**< Should ALWAYS be 0x71 so this is a good
                               *   test of communication                     */

// Magnetometer data
#define WIA 0x00    /**< Device ID, should always read 0x48                  */
#define ST1 0x02    /**< Data ready                                          */
#define ST2 0x09    /**< Read this to indicate "read end". Also indicates if
                     *   sensor overflow occurs                              */

#define CNTL1 0x0A  /**< Changes bit depth and operating mode                */
#define CNTL2 0x0B  /**< Set the lsb to reset the sensor                     */

// Register addresses for accelerometer data (pg. 8 register map)
#define MPU9250_ACCEL_X_ADDR_H 0x3B /**< AXH */
#define MPU9250_ACCEL_X_ADDR_L 0x3C /**< AXL */
#define MPU9250_ACCEL_Y_ADDR_H 0x3D /**< AYH */
#define MPU9250_ACCEL_Y_ADDR_L 0x3E /**< AYL */
#define MPU9250_ACCEL_Z_ADDR_H 0x3F /**< AZH */
#define MPU9250_ACCEL_Z_ADDR_L 0x40 /**< AZL */


// Register addresses for gyroscope data (pg. 8 register map)
#define MPU9250_GYRO_X_ADDR_H 0x43 /**< VXH */
#define MPU9250_GYRO_X_ADDR_L 0x44 /**< VXL */
#define MPU9250_GYRO_Y_ADDR_H 0x45 /**< VYH */
#define MPU9250_GYRO_Y_ADDR_L 0x46 /**< VYL */
#define MPU9250_GYRO_Z_ADDR_H 0x47 /**< VZH */
#define MPU9250_GYRO_Z_ADDR_L 0x48 /**< VZL */


// Register addresses for magnetometer data (pg. 47 register map)
#define MPU9250_MAG_X_ADDR_L 0x03 /**< HXL */
#define MPU9250_MAG_X_ADDR_H 0x04 /**< HXH */
#define MPU9250_MAG_Y_ADDR_L 0x05 /**< HYL */
#define MPU9250_MAG_Y_ADDR_H 0x06 /**< HYH */
#define MPU9250_MAG_Z_ADDR_L 0x07 /**< HZL */
#define MPU9250_MAG_Z_ADDR_H 0x08 /**< HZH */


// Scales for readings
/** @brief Accelerometer full-scale range in m/s^2 */
static const float MPU9250_ACCEL_FULL_SCALE = 2 * 9.807;

/** @brief Gyroscope full-scale range in degrees/s */
static const float MPU9250_GYRO_FULL_SCALE = 250.0;

/**
 * @brief Magnetometer full-scale range in microTeslas. See pg. 50 of register
 *        map
 */
static const float MPU9250_MAG_FULL_SCALE = 4912.0;




/********************************* Functions *********************************/
/**
 * @defgroup MPU9250_Driver_Init_Functions Initialization Functions
 * @ingroup MPU9250_Driver
 * @brief Initialization routines
 * @{
 */

/**
 * @brief  Initializes the MPU9250 sensor by writing various settings to its
 *         registers, and reading a few of them for verification
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU9250 sensor
 * @return 1 if successful, otherwise a negative error code
 */
int MPU9250Init(MPU9250_t* myMPU){
    myMPU->az = NAN;
    myMPU->ay = NAN;
    myMPU->ax = NAN;
    myMPU->A = NAN;
    myMPU->vz = NAN;
    myMPU->vy = NAN;
    myMPU->vx = NAN;
    myMPU->hx = NAN;
    myMPU->hy = NAN;
    myMPU->hz = NAN;

    /********** Check that MPU9250 is connected **********/
    uint8_t buff[1];

    // Check for bus communication essentially. If any function should fail and issue an early return, it would most likely
    // be this one.
    if(HAL_I2C_Mem_Read(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, WHO_AM_I,
    I2C_MEMADD_SIZE_8BIT, buff, 1, 100) != HAL_OK){
        return -1;
    }

    // Check that the WHO_AM_I register is 0x71
    if(buff[0] != 0x71){
        return -2;
    }

    /********** Configure accelerometer and gyroscope **********/
    // Use the best available clock source
    uint8_t dataToWrite = 0x01;
    if(HAL_I2C_Mem_Write(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, PWR_MGMT_1,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -3;
    }

    // Enable I2C master interface module
    dataToWrite = 0x20;
    if(HAL_I2C_Mem_Write(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, USER_CTRL,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -4;
    }

    // Set I2C module to use 400 kHz speed (pg. 19 of register map)
    dataToWrite = 0x0D;
    if(HAL_I2C_Mem_Write(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, I2C_MST_CTRL,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -5;
    }

    // Force accelerometer and gyroscope to ON
    dataToWrite = 0x00;
    if(HAL_I2C_Mem_Write(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, PWR_MGMT_2,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -6;
    }

    // Enable I2C bypass
    dataToWrite = 0x02;
    if(HAL_I2C_Mem_Write(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, INT_PIN_CFG,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100)
            != HAL_OK){
        return -7;
    }

    /********** Configure magnetometer **********/
    // Check that correct device ID is read
    if(HAL_I2C_Mem_Read(&hi2c1, MPU9250_MAG_ADDR, WIA, I2C_MEMADD_SIZE_8BIT,
            buff, 1, 100) != HAL_OK){
        return -8;
    }
    if(buff[0] != 0x48){
        return -9;
    }

    // Reset
    dataToWrite = 0x01;
    if(HAL_I2C_Mem_Write(&hi2c1, MPU9250_MAG_ADDR, CNTL2, I2C_MEMADD_SIZE_8BIT,
            &dataToWrite, sizeof(dataToWrite), 100) != HAL_OK){
        return -10;
    }

    HAL_Delay(10); // Resetting might take some time

    // Enable continuous measurement
    dataToWrite = 0x16;
    if(HAL_I2C_Mem_Write(&hi2c1, MPU9250_MAG_ADDR, CNTL1, I2C_MEMADD_SIZE_8BIT,
            &dataToWrite, sizeof(dataToWrite), 100) != HAL_OK){
        return -11;
    }

    /* Return success */
    return 1;
}

/**
 * @brief  Reset the inertial measurement unit using blocking IO
 * @return None
 */
void resetIMUBlocking(void){
    uint8_t dataToWrite = 0x80;
    HAL_I2C_Mem_Write(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, PWR_MGMT_1,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100);
}

/**
 * @brief  Reset the magnetometer using blocking IO
 * @return None
 */
void resetMagnetometerBlocking(void){
    uint8_t dataToWrite = 0x80;
    HAL_I2C_Mem_Write(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR, PWR_MGMT_1,
            I2C_MEMADD_SIZE_8BIT, &dataToWrite, sizeof(dataToWrite), 100);
}

/**
 * @}
 */
/* end - MPU9250_Driver_Init_Functions */

/**
 * @defgroup MPU9250_Driver_Threaded Readers
 * @ingroup MPU9250_Driver
 * @brief Sensor data readers used once the scheduler has started
 * @{
 */

/**
 * @brief  Read from the az, ay, ax register addresses and stores the results in
 *         the myMPU object passed in
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU9250 sensor
 * @param  sem Handle for the semaphore to take while transferring data
 * @return 1 if successful, -1 otherwise
 */
int accelReadDMA(MPU9250_t* myMPU, osSemaphoreId sem){
    uint8_t mpu_buff[6]; // Temporary buffer to hold data from sensor
    int16_t temp;

    if(HAL_I2C_Mem_Read_DMA(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR,
            MPU9250_ACCEL_X_ADDR_H,
            I2C_MEMADD_SIZE_8BIT, mpu_buff, 6) != HAL_OK){
        myMPU->ax = NAN;
        myMPU->ay = NAN;
        myMPU->az = NAN;
        return -1;
    }
    if(xSemaphoreTake(sem, MAX_SEM_WAIT) != pdTRUE){
        myMPU->ax = NAN;
        myMPU->ay = NAN;
        myMPU->az = NAN;
        return -2;
    }

    // Process data; scale to physical units
    temp = (mpu_buff[0] << 8 | mpu_buff[1]); // Shift bytes into appropriate positions
    myMPU->ax = (temp * MPU9250_ACCEL_FULL_SCALE / (32767.0)); // Scale to physical units

    temp = (mpu_buff[2] << 8 | mpu_buff[3]);
    myMPU->ay = (temp * MPU9250_ACCEL_FULL_SCALE / (32767.0));

    temp = (mpu_buff[4] << 8 | mpu_buff[5]);
    myMPU->az = (temp * MPU9250_ACCEL_FULL_SCALE / (32767.0));

    return 1;
}

/**
 * @brief  Read from the az, ay, ax register addresses and stores the results in
 *         the myMPU object passed in
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU9250 sensor
 * @param  sem Handle for the semaphore to take while transferring data
 * @return 1 if successful, -1 otherwise
 */
int gyroReadDMA(MPU9250_t* myMPU, osSemaphoreId sem){
    uint8_t mpu_buff[6]; // Temporary buffer to hold data from sensor
    int16_t temp;

    if(HAL_I2C_Mem_Read_DMA(&hi2c3, MPU9250_ACCEL_AND_GYRO_ADDR,
            MPU9250_GYRO_X_ADDR_H,
            I2C_MEMADD_SIZE_8BIT, mpu_buff, 6) != HAL_OK){
        myMPU->vx = NAN;
        myMPU->vy = NAN;
        myMPU->vz = NAN;
        return -1;
    }
    if(xSemaphoreTake(sem, MAX_SEM_WAIT) != pdTRUE){
        myMPU->vx = NAN;
        myMPU->vy = NAN;
        myMPU->vz = NAN;
        return -1;
    }

    // Process data; scale to physical units
    temp = (mpu_buff[0] << 8 | mpu_buff[1]);
    myMPU->vx = (temp / (32767.0) * MPU9250_GYRO_FULL_SCALE);

    temp = (mpu_buff[2] << 8 | mpu_buff[3]);
    myMPU->vy = (temp / (32767.0) * MPU9250_GYRO_FULL_SCALE);

    temp = (mpu_buff[4] << 8 | mpu_buff[5]);
    myMPU->vz = (temp / (32767.0) * MPU9250_GYRO_FULL_SCALE);

    return 1;
}

/**
 * @brief  Reads from the magnetometer and stores the results in the myMPU
 *         object passed in
 * @note   The high and low bytes switch places for the magnetic field readings
 *         due to the way the registers are mapped. Note that 7 bytes are read
 *         because the magnetometer requires the ST2 register to be read in
 *         addition to other data
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU9250 sensor
 * @param  sem Handle for the semaphore to take while transferring data
 * @return 1 if successful, otherwise a negative error code
 */
int magFluxReadDMA(MPU9250_t* myMPU, osSemaphoreId sem){
    uint8_t mpu_buff[7]; // Temporary buffer to hold data from sensor
    int16_t temp;

    if(HAL_I2C_Mem_Read_DMA(&hi2c1, MPU9250_MAG_ADDR, MPU9250_MAG_X_ADDR_L,
            I2C_MEMADD_SIZE_8BIT, mpu_buff, sizeof(mpu_buff)) != HAL_OK){
        myMPU->hx = NAN;
        myMPU->hy = NAN;
        myMPU->hz = NAN;
        return -1;
    }
    if(xSemaphoreTake(sem, MAX_SEM_WAIT) != pdTRUE){
        myMPU->hx = NAN;
        myMPU->hy = NAN;
        myMPU->hz = NAN;
        return -2;
    }

    // Process data; scale to physical units
    temp = (mpu_buff[1] << 8 | mpu_buff[0]);
    myMPU->hx = (temp / (32760.0) * MPU9250_MAG_FULL_SCALE);

    temp = (mpu_buff[3] << 8 | mpu_buff[2]);
    myMPU->hy = (temp / (32760.0) * MPU9250_MAG_FULL_SCALE);

    temp = (mpu_buff[5] << 8 | mpu_buff[4]);
    myMPU->hz = (temp / (32760.0) * MPU9250_MAG_FULL_SCALE);

    return 1;
}

/**
 * @}
 */
/* end - MPU9250_Driver_Threaded */

/**
 * @defgroup MPU9250_Driver_Error_Handlers Errata handlers
 * @brief Handles a silicon bug in the I2C module that shows up occasionally
 * @ingroup MPU9250_Driver
 *
 * @details
 * These functions are used as a workaround for an issue where the BUSY flag of
 * the I2C module is erroneously asserted in the hardware (a silicon bug,
 * essentially). By checking the logs for "nan", I have been able to see that
 * this fix indeed will resolve I2C bus conflict. So it is useful to have.
 *
 * Overall, use these functions with EXTREME caution.
 *
 * Resources used for resolution:
 *  - https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour/281046#281046
 *  - https://community.st.com/thread/35884-cant-reset-i2c-in-stm32f407-to-release-i2c-lines
 *  - https://electronics.stackexchange.com/questions/272427/stm32-busy-flag-is-set-after-i2c-initialization
 *  - http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
 *
 * @{
 */

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

    static struct I2C_Module i2c3module = {
            &hi2c3,
            MPU_SDA_Pin,
            MPU_SDA_GPIO_Port,
            MPU_SCL_Pin,
            MPU_SCL_GPIO_Port
    };

    static struct I2C_Module i2c1module = {
            &hi2c1,
            MAG_SDA_Pin,
            MAG_SDA_GPIO_Port,
            MAG_SCL_Pin,
            MAG_SCL_GPIO_Port
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
/* end - MPU9250_Driver */
