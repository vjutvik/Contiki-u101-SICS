#ifndef STM32L_SPI_H
#define STM32L_SPI_H

#include <stdint.h>

typedef struct {
        volatile uint16_t CR1;
        uint16_t R0;
        volatile uint16_t CR2;
        uint16_t R1;
        volatile uint16_t SR;
        uint16_t R2;
        volatile uint16_t DR;
        uint16_t R3;
        volatile uint16_t CRCPR;
        uint16_t R4;
        volatile uint16_t RXCRCR;
        uint16_t R5;
        volatile uint16_t TXCRCR;
        uint16_t R6;  
} STM32_SPI;

#define SPI_CR1_CPHA                    ((uint16_t)0x0001)
#define SPI_CR1_CPOL                    ((uint16_t)0x0002)
#define SPI_CR1_MSTR                    ((uint16_t)0x0004)

#define SPI_CR1_BR                      ((uint16_t)0x0038)

#define SPI_CR1_SPE                     ((uint16_t)0x0040)
#define SPI_CR1_LSBFIRST                ((uint16_t)0x0080)
#define SPI_CR1_SSI                     ((uint16_t)0x0100)
#define SPI_CR1_SSM                     ((uint16_t)0x0200)
#define SPI_CR1_RXONLY                  ((uint16_t)0x0400)
#define SPI_CR1_DFF                     ((uint16_t)0x0800)
#define SPI_CR1_CRCNEXT                 ((uint16_t)0x1000)
#define SPI_CR1_CRCEN                   ((uint16_t)0x2000)
#define SPI_CR1_BIDIOE                  ((uint16_t)0x4000)
#define SPI_CR1_BIDIMODE                ((uint16_t)0x8000)

#define SPI_CR2_RXDMAEN                 ((uint8_t)0x01)
#define SPI_CR2_TXDMAEN                 ((uint8_t)0x02)
#define SPI_CR2_SSOE                    ((uint8_t)0x04)
#define SPI_CR2_ERRIE                   ((uint8_t)0x20)
#define SPI_CR2_RXNEIE                  ((uint8_t)0x40)
#define SPI_CR2_TXEIE                   ((uint8_t)0x80)

#define SPI_SR_RXNE                     ((uint8_t)0x01)
#define SPI_SR_TXE                      ((uint8_t)0x02)
#define SPI_SR_CRCERR                   ((uint8_t)0x10)
#define SPI_SR_MODF                     ((uint8_t)0x20)
#define SPI_SR_OVR                      ((uint8_t)0x40)
#define SPI_SR_BSY                      ((uint8_t)0x80)

void stm32_spi_init(STM32_SPI *spi);
void stm32_spi_set_rate(STM32_SPI *spi, uint32_t max_rate);
uint16_t stm32_spi_rx(STM32_SPI *spi);
void stm32_spi_tx(STM32_SPI *spi, uint16_t data);
uint16_t stm32_spi_txrx(STM32_SPI *spi, uint16_t data);
void stm32_spi_enable(STM32_SPI *spi);
void stm32_spi_disable(STM32_SPI *spi);

#endif
