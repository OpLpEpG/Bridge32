
#ifndef SERIAL2_H
#define SERIAL2_H
#pragma once

#define UART_BUF_LEN 0x10000
extern uint8_t buf[UART_BUF_LEN];
extern volatile int read_count; 
extern volatile int write_count;
extern volatile bool UartBreack;

void Uart2Start(void);
void Uart2TurboHandler(void);
void Uart2Send(const uint8_t* buf, size_t len);

#endif
