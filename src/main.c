#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/cyw43_arch.h>

#include <hardware/gpio.h>

#define LED 16

int main(){
    stdio_init_all();
    
    if(cyw43_arch_init()){
        printf("WiFi init failed\n");
        return -1;
    }
    
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    while(1){
        gpio_put(LED, !gpio_get(LED));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
        printf("Hello world!\n");
        sleep_ms(500);
    }

}
