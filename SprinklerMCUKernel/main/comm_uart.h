#ifndef COMM_UART_H
#define COMM_UART_H

static void echo_send(const int port, const char *str, uint8_t length);
void uart_init();
void uart_receive_task(void *arg);

#endif // COMM_UART_H