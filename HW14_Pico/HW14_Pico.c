#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID     uart0
#define BAUD_RATE   115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define LED_PIN     PICO_DEFAULT_LED_PIN   // GP25 on regular Pico

int main() {
    stdio_init_all();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    int counter = 0;
    while (true) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Hello from Pico! count=%d\r\n", counter++);
        uart_puts(UART_ID, msg);
        sleep_ms(1000);
    }
}