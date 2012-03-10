

void stm32_usart_set_rate(STM32_USART *usart, stm32_clk pclk)
{
  uint32_t idiv, fdiv, tmp;
  uint32_t pclk_hz;

  pclk_hz = stm32_clk_frequency(pclk);

  /* Determine the integer part */
  idiv = ((0x19 * pclk_hz) / (0x04 * (baudrate)));
  tmp = (idiv / 0x64) << 0x04;

  /* Determine the fractional part */
  fdiv = idiv - (0x64 * (tmp >> 0x04));
  tmp |= ((((fdiv * 0x10) + 0x32) / 0x64)) & ((uint8_t)0x0f);

  /* Write to USART BRR */
  DBG_UART->BRR = (uint16_t)tmp;
}
