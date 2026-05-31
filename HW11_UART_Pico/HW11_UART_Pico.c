#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID     uart0
#define BAUD_RATE   115200
#define UART_TX_PIN 0   // GP0 -> STM32 RX (PA1)
#define UART_RX_PIN 1   // GP1 <- STM32 TX (PA0)

int main() {
    stdio_init_all();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Wait until the host actually opens the USB serial port
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);  // small extra delay so the terminal is fully ready

    printf("Pico ready - pass-through active\r\n");

    while (true) {
        // PC (USB) -> STM32 (UART)
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uart_putc_raw(UART_ID, (char)c);
        }

        // STM32 (UART) -> PC (USB)
        if (uart_is_readable(UART_ID)) {
            char ch = uart_getc(UART_ID);
            putchar(ch);
        }
    }
}