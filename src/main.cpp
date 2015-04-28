/**
 * @file
 * @date 	28 April 2015
 * @author	Richard Osterloh <richard.osterloh@gmail.com>
 */

#include "mbed.h"
#include "rtos.h"
#include "EPD_GDE021A1.h"

EPD_GDE021A1 epd(PinName::PA_15, PinName::PB_11, PinName::PB_2, PinName::PA_8, PinName::PB_10, PinName::PB_5, PinName::PB_4, PinName::PB_3);
DigitalOut led1(PinName::LED1);
DigitalOut led2(PinName::LED2);

void led2_thread(void const *args) {
    while (true) {
        led2 = !led2;
        Thread::wait(1000);
    }
}

int main() {
    epd.Clear(EPD_COLOR_WHITE);
    epd.DisplayStringAtLine(5, (uint8_t*)"STM32L053C8", CENTER_MODE);
    epd.DisplayStringAtLine(3, (uint8_t*)"Epaper display", LEFT_MODE);
    epd.DisplayStringAtLine(2, (uint8_t*)"demo", LEFT_MODE);
    epd.RefreshDisplay();

    Thread thread(led2_thread);

    while (true) {
        led1 = !led1;
        Thread::wait(500);
    }
}
