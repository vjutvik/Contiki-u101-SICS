
#include <dev/leds.h>
#include <stm32l-gpio.h>
#include <stm32-clk.h>

void
leds_arch_init(void)
{
  /* Enable clock to the GPIO peripherals */
  stm32_clk_pclk_enable(GPIOC);
  stm32_clk_pclk_enable(GPIOD);
  /* Configure GPIO as outputs */
  stm32l_gpio_conf_output(GPIOC, 6, GPIO_OUTPUT_TYPE_PPULL, GPIO_OSPEED_400KHZ);
  stm32l_gpio_conf_output(GPIOD, 2, GPIO_OUTPUT_TYPE_PPULL, GPIO_OSPEED_400KHZ);
  /* HIGH means LED off */
  stm32l_gpio_output_set(GPIOC, 6);
  stm32l_gpio_output_set(GPIOD, 2);
}

unsigned char
leds_arch_get(void)
{
  unsigned char leds;
  uint16_t gpio;
  leds = 0;
  if (stm32l_gpio_output_get(GPIOC, 6)) {
    leds |= LEDS_GREEN;
  }
  if (stm32l_gpio_output_get(GPIOD, 2)) {
    leds |= LEDS_RED;
  }
  return leds;
}

void
leds_arch_set(unsigned char leds)
{

  if ((leds & LEDS_GREEN) || (leds == LEDS_ALL)) {
    stm32l_gpio_output_clear(GPIOC, 6);
  } else {
    stm32l_gpio_output_set(GPIOC, 6);
  }

  if ((leds & LEDS_RED) || (leds == LEDS_ALL)) {
    stm32l_gpio_output_clear(GPIOD, 2);
  } else {
    stm32l_gpio_output_set(GPIOD, 2);
  }
}
