#define SERVO_MIN_PULSEWIDTH 400 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2400 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#include "driver/mcpwm.h"

static unsigned long lastStateTime;


uint32_t angle, count;

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


void setup() {
    // MCPWM_UNIT_0/1 /MCPWM_TIMER_0/1/2/ MCPWM0/1/2A/MCPWM0/1/2B  
    
    //1. initial GPIO configuration
     mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, 18);    //Set GPIO 18 as PWM0A, to which servo is connected
     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 19);    //Set GPIO 18 as PWM0A, to which servo is connected 
   
    //2. initial mcpwm configuration
    
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings


    Serial.begin(115200); // USB serial (for DEBUG) 
}

void loop() {




  
for (count = 0; count < SERVO_MAX_DEGREE; count++) {
           
            angle = servo_per_degree_init(count);
           
            mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            delay(50);

 if (millis() - lastStateTime > 100)     // Print the data every 300ms
  {
    lastStateTime = millis();
    Serial.println(mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B));
    Serial.println(mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1));
    Serial.println(mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B)*200);
   
    Serial.println("");}           
}

}
