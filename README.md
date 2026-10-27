MFM384 RS485 MODBUS RTU - FINAL README

Board: DC_UPS_Mother_V2 - STM32G070RBT6

Settings
Pins: PB6 = TX, PB7 = RX, PA12 = DE, LCD = PC0-PC10
Baud: 9600
ID: 1
Register: 0x0000 (V1N, V2N, V3N)
This code has NO delay(), only millis() and micros()
Connection
MFM384 A+ (Red) to RS485 A
MFM384 B- (Brown) to RS485 B
GND to GND
Protocol 2 Pass/Fail Tasks Covered
RS485 driver direction control timing, correct turnaround
Code: PA12 HIGH for TX, wait for TC flag, PA12 LOW for RX
CRC16 correct
Code: crc16() polynomial 0xA001 + check
Timeout retry policy
Code: 300 ms timeout + 3 retries using millis()
Exception handling
Code: check 0x84 exception + error clear
Safe register map access
Code: isSafeRegister() allows only 0x0000, 0x0002, 0x0004, 0x000E
Output on LCD
MFM384 STATUS: OK
V1N: 244.3 V
V2N: 0.0 V
V3N: 0.0 V

If wire disconnected:

COMM FAIL TIMEOUT
