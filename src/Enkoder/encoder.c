#include "encoder.h"
static int left_rotation, right_rotation;
static int digit_to_change;


void encoder_callback(uint gpio, uint32_t events){

    uint32_t gpio_state = gpio_get_all();

    static bool ccw_fall = 0;
	static bool cw_fall = 0;

    uint32_t enc_value = (gpio_state & ((1 << ENC_A) | (1 << ENC_B)));

    if (gpio == ENC_A) 
    {
        if ((!cw_fall) && (enc_value == (1 << ENC_B))) // cw_fall is set to TRUE when phase A interrupt is triggered
            cw_fall = 1;

        if ((ccw_fall) && (enc_value == 0b00)) // if ccw_fall is already set to true from a previous B phase trigger, the ccw event will be triggered 
        {
            cw_fall = 0;
            ccw_fall = 0;
            left_rotation = 1;
            printf("%d \r\n", left_rotation);
            // if(left_rotation==9){
            //     left_rotation=0;    
            // }
            // else{
            //     left_rotation++;
            // }    
        }
    }	
    else if (gpio == ENC_B) 
    {
        if ((!ccw_fall) && (enc_value == 1 << ENC_A)) //ccw leading edge is true
            ccw_fall = 1;

        if ((cw_fall) && (enc_value == 0b00)) //cw trigger
        {
            cw_fall = 0;
            ccw_fall = 0;
            right_rotation = 1;
            printf("%d \r\n", right_rotation);
            // if(right_rotation == 9){
            //     right_rotation=0;
            // }
            // else{
            //     right_rotation++;
            // }  
        }
    }

    if(gpio == ENC_SW){
        digit_to_change = (digit_to_change + 1) % 2;
        printf("digit to change: %d \r\n", digit_to_change); 
    }
}

void encoder_init(){

    gpio_init(ENC_SW);					
    gpio_set_dir(ENC_SW,GPIO_IN);
	gpio_disable_pulls(ENC_SW);

	gpio_init(ENC_A);
    gpio_set_dir(ENC_A,GPIO_IN);
	gpio_disable_pulls(ENC_A);

	gpio_init(ENC_B);
    gpio_set_dir(ENC_B,GPIO_IN);
	gpio_disable_pulls(ENC_B);

    encoder_enable_interrupts();
}


void encoder_enable_interrupts(){
    gpio_set_irq_enabled_with_callback(ENC_SW, GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled(ENC_A, GPIO_IRQ_EDGE_FALL, true);
	gpio_set_irq_enabled(ENC_B, GPIO_IRQ_EDGE_FALL, true);
}

void set_time(datetime_t *time){
    if (left_rotation || right_rotation){
        rtc_disable_alarm();
        if(digit_to_change == 0){
            time->hour += left_rotation;
            time->hour -= right_rotation;
            if(time->hour >= 24) time->hour = 0;
            else if (time->hour < 0) time->hour = 23;
        } else {
            time->min += left_rotation;
            time->min -= right_rotation;
            if (time->min >= 60) time->min = 0;
            else if (time->min < 0) time->min = 59;
        }
        
        left_rotation = 0;
        right_rotation = 0;
        
        rtc_set_datetime(time);
        rtc_enable_alarm();
    }
}