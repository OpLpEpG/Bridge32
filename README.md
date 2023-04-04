# Bridge32
Arduino esp32  to RS485 bridge (wifi(qwerty, 12345678), udp(192.168.4.1:5000), uart2(UART_MODE_RS485_HALF_DUPLEX) 
## Baudrate
### base baudrate 125000
#### turbo 500_000 cmd (0xFD 01 crc16)
#### turbo 1000_000 cmd (0xFD 02 crc16)
#### unturbo timeout 16000ms or cmd(0xFD 00 crc16)
