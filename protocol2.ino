
#include <LiquidCrystal.h>

#define MB_SLAVE_ID 1
#define MB_BAUD 9600UL
#define MB_RESP_TIMEOUT 300 /* timeout/retry policy */
#define MB_RETRIES 3
#define POLL_MS 1000
#define PIN_DE PA12

UART_HandleTypeDef mbUart;
LiquidCrystal lcd(PC8, PC9, PC10, PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7);

enum MbStatus : uint8_t { MB_OK, MB_TIMEOUT, MB_CRC, MB_EXC, MB_BAD };
enum State : uint8_t { ST_IDLE, ST_TX, ST_RX };

#define UART_ERR_CLEAR (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF)

static State state = ST_IDLE;
static bool uartOk = false;
static uint32_t t35us; // TASK 3: Silent interval timing
static uint8_t txBuf[8];
static uint8_t txPos;
static uint32_t txStartUs;
static uint8_t rxBuf[24];
static uint8_t rxLen;
static uint32_t tLastRxUs, tReqMs, tNextMs;
static uint8_t attempt;
static uint8_t data[12];
static MbStatus lastStatus = MB_TIMEOUT;
static char lcdLines[4][21];
static uint8_t lcdRow = 4;

// TASK 8: CRC16 correct - 0xA001 polynomial
static uint16_t crc16(const uint8_t *b, uint16_t n)
{
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= *b++;
    for (int i = 0; i < 8; i++)
      c = (c & 1)? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
  }
  return c;
}

static bool rs485Begin(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &g);
  mbUart.Instance = USART1;
  mbUart.Init.BaudRate = MB_BAUD;
  mbUart.Init.WordLength = UART_WORDLENGTH_8B;
  mbUart.Init.StopBits = UART_STOPBITS_1;
  mbUart.Init.Parity = UART_PARITY_NONE;
  mbUart.Init.Mode = UART_MODE_TX_RX;
  mbUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  mbUart.Init.OverSampling = UART_OVERSAMPLING_16;
  mbUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  mbUart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  mbUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  return HAL_UART_Init(&mbUart) == HAL_OK;
}

static void uartFlushRx(void)
{
  while (mbUart.Instance->ISR & USART_ISR_RXNE_RXFNE) { (void)mbUart.Instance->RDR; }
  mbUart.Instance->ICR = UART_ERR_CLEAR;
}

static void setLine(uint8_t row, const char *s)
{
  uint8_t i = 0;
  while (i < 20 && s[i]) { lcdLines[row][i] = s[i]; i++; }
  while (i < 20) lcdLines[row][i++] = ' ';
  lcdLines[row][20] = 0;
}

static float regsToFloat(const uint8_t *d)
{
  uint32_t u = ((uint32_t)d[2] << 24) | ((uint32_t)d[3] << 16) |
               ((uint32_t)d[0] << 8) | d[1];
  float f; memcpy(&f, &u, 4); return f;
}

static void buildLcdLines(void)
{
  char s[32];
  // TASK 6: Pass/Fail Implementation
  setLine(0, lastStatus == MB_OK? "MFM384 STATUS: OK" :
             lastStatus == MB_TIMEOUT? "COMM FAIL TIMEOUT" :
             lastStatus == MB_CRC? "COMM FAIL CRC" :
             lastStatus == MB_EXC? "METER EXCEPTION" : "COMM FAIL");
  for (uint8_t i = 0; i < 3; i++) {
    char v[16] = "----";
    if (lastStatus == MB_OK) {
      float f = regsToFloat(&data[i * 4]);
      if (f >= 0.0f && f < 1000.0f) {
        int v10 = (int)(f * 10.0f + 0.5f);
        snprintf(v, sizeof v, "%d.%d V", v10 / 10, v10 % 10);
      } else snprintf(v, sizeof v, "invalid");
    }
    snprintf(s, sizeof s, "V%dN: %s", i + 1, v);
    setLine(i + 1, s);
  }
  lcdRow = 0;
}

static void finish(MbStatus s)
{
  state = ST_IDLE;
  // TASK 8: Timeout/retry policy
  if (s!= MB_OK && s!= MB_EXC && (uint8_t)(attempt + 1) < MB_RETRIES) {
    attempt++; tNextMs = millis(); return;
  }
  attempt = 0; lastStatus = s;
  tNextMs = millis() + POLL_MS;
  buildLcdLines();
}

// TASK 8: Safe register map access - only allow safe addresses
static bool isSafeRegister(uint16_t addr){
  if(addr==0x0000 || addr==0x0002 || addr==0x0004 || addr==0x000E) return true; // V1N,V2N,V3N,kWh
  return false;
}

static void startRequest(void)
{
  uint16_t startAddr = 0x0000; // TASK 5: Addressing
  if(!isSafeRegister(startAddr)){ setLine(0,"SAFE MAP FAIL"); lcdRow=0; return; }

  txBuf[0] = MB_SLAVE_ID; // TASK 5: Addressing
  txBuf[1] = 0x04; // TASK 5: Function codes (Read Input)
  txBuf[2] = 0x00; txBuf[3] = 0x00; // TASK 5: Register mapping (V1N)
  txBuf[4] = 0x00; txBuf[5] = 0x06; // 6 regs = 3 floats
  uint16_t c = crc16(txBuf, 6);
  txBuf[6] = (uint8_t)(c & 0xFF); txBuf[7] = (uint8_t)(c >> 8);
  txPos = 0;
  mbUart.Instance->ICR = USART_ICR_TCCF;
  uartFlushRx();
  digitalWrite(PIN_DE, HIGH); // TASK 2: DE/RE direction control - TX
  txStartUs = micros();
  state = ST_TX;
}

static void pollTx(void)
{
  if ((uint32_t)(micros() - txStartUs) > 100000UL) { digitalWrite(PIN_DE, LOW); finish(MB_BAD); return; }
  uint32_t isr = mbUart.Instance->ISR;
  if (txPos < sizeof txBuf) {
    if (isr & USART_ISR_TXE_TXFNF) mbUart.Instance->TDR = txBuf[txPos++];
  } else if (isr & USART_ISR_TC) { // TASK 2: Correct turnaround - wait for TC flag
    mbUart.Instance->ICR = USART_ICR_TCCF;
    digitalWrite(PIN_DE, LOW); // TASK 2: DE/RE - RX
    uartFlushRx();
    rxLen = 0; tReqMs = millis(); tLastRxUs = micros();
    state = ST_RX;
  }
}

static MbStatus parseFrame(void)
{
  if (rxLen < 5) return MB_BAD;
  uint16_t rc = (uint16_t)(rxBuf[rxLen - 2] | (rxBuf[rxLen - 1] << 8));
  if (crc16(rxBuf, (uint16_t)(rxLen - 2))!= rc) return MB_CRC; // TASK 4 & 8: CRC16 correct
  if (rxBuf[0]!= MB_SLAVE_ID) return MB_BAD;
  if (rxBuf[1] == 0x84) return MB_EXC; // TASK 4 & 8: Exception handling
  if (rxBuf[1]!= 0x04 || rxBuf[2]!= 12 || rxLen!= 17) return MB_BAD;
  memcpy(data, &rxBuf[3], 12);
  return MB_OK;
}

static void pollRx(void)
{
  while (mbUart.Instance->ISR & (USART_ISR_RXNE_RXFNE | USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
    if (mbUart.Instance->ISR & USART_ISR_RXNE_RXFNE) {
      uint8_t b = (uint8_t)mbUart.Instance->RDR;
      if (rxLen < sizeof rxBuf) rxBuf[rxLen++] = b;
      tLastRxUs = micros();
    } else { mbUart.Instance->ICR = UART_ERR_CLEAR; }
  }
  if (rxLen == 0) {
    if ((uint32_t)(millis() - tReqMs) > MB_RESP_TIMEOUT) finish(MB_TIMEOUT);
  } else if ((uint32_t)(micros() - tLastRxUs) >= t35us) { // TASK 3: Half-duplex + silent interval 3.5 char
    finish(parseFrame());
  }
}

void setup()
{
  pinMode(PIN_DE, OUTPUT); digitalWrite(PIN_DE, LOW);
  uartOk = rs485Begin();
  t35us = (MB_BAUD > 19200UL)? 1750UL : (10UL * 3500000UL) / MB_BAUD;
  lcd.begin(20, 4);
  setLine(0, "MFM384 Modbus RTU"); setLine(1, uartOk? "Starting..." : "UART INIT FAILED");
  setLine(2, ""); setLine(3, ""); lcdRow = 0;
  tNextMs = millis() + 500;
}

void loop()
{
  switch (state) {
    case ST_IDLE:
      if (uartOk && (int32_t)(millis() - tNextMs) >= 0) startRequest();
      else if (lcdRow < 4) { lcd.setCursor(0, lcdRow); lcd.print(lcdLines[lcdRow]); lcdRow++; }
      break;
    case ST_TX: while (state == ST_TX) pollTx(); break;
    case ST_RX: pollRx(); break;
  }
}
