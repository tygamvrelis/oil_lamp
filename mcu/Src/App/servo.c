/**
 * @file Servo.c
 * @author Tyler
 *
 * @ingroup Servo
 * @{
 */




/********************************** Includes *********************************/
#include "App/servo.h"
#include "App/util.h"




/****************************** Public variables ****************************/
const int8_t POS_MAX = 45;  /**< Degrees */
const int8_t POS_MIN = -45; /**< Degrees */




/****************************** Private variables ****************************/
static const uint32_t POS_MAX_DUTY_CYCLE = 2000 * 90; // 2 ms * 90 MHz
static const uint32_t POS_MIN_DUTY_CYCLE = 1000 * 90; // 1 ms * 90 MHz




/***************************** Public Functions ******************************/
void servo_init(
    Servo_t* p_servo,
    Servo_e id,
    TIM_HandleTypeDef* htim,
    uint32_t channel
)
{
    p_servo->id = id;
    p_servo->htim = htim;
    p_servo->channel = channel;
    HAL_TIM_PWM_Start(htim, channel);
}

//-----------------------------------------------------------------------------

void servo_set_position(Servo_t* p_servo, int8_t angle)
{
    float fpos = (float)bound_int8_t(angle, POS_MIN, POS_MAX);
    float FPOS_MAX = POS_MAX;
    float FPOS_MAX_DC = POS_MAX_DUTY_CYCLE;
    float FPOS_MIN_DC = POS_MIN_DUTY_CYCLE;

    uint16_t tim = (fpos / FPOS_MAX) * (FPOS_MAX_DC - FPOS_MIN_DC) + FPOS_MIN_DC;
    __HAL_TIM_SET_COMPARE(p_servo->htim, p_servo->channel, tim);
}

/**
 * @}
 */
/* end - Servo */
