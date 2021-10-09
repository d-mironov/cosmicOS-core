/*
 * cosmicOS usart.h - USART HAL
 *
 * @author:     cosmicfennek
 * @version:    v2.1
 *
 * ├ ╰ ─ │
 * Changelog v2:
 *   ├── v2.1
 *   │    ├── changed to USART port struct
 *   │    ├── added interrupt setting
 *   │    ├── added interrupt driven write/printf
 *   │    ├── added interrupt driven read
 *   │    ├── added interrupt scan function
 *   │    ├── added RX available function (`USART_available()`)
 *   │    ╰── 
 *
 *
 * TODO: [x] select option on interrupt driven
 * TODO: [x] implement interrupt driven write 
 * TODO: [x] implement interrupt driven read 
 * TODO: [x] fix interrupt read string fracturing error
 * TODO: [x] write interrupt for all other USART ports
 * TODO: [ ] write docs
 */




#ifndef _STM_UART_H
#define _STM_UART_H

#include <stm32f4xx.h>
#include <stdbool.h>

#define USART2_CLK  16000000
//#define USARTx_CLK  50000000
#define USARTx_CLK  16000000


#define USART1_PORT     GPIOA
#define USART2_PORT     GPIOA
#define USART6_PORT     GPIOC
// USART1 pins
#define USART1_RX       PA10
#define USART1_TX       PA9
#define USART1_CTS      PA11
#define USART1_CK       PA8
#define USART1_RTS      PA12
// USART2 pins
#define USART2_CTS      PA0
#define USART2_RTS      PA1
#define USART2_TX       PA2
#define USART2_RX       PA3
#define USART2_CK       PA4
// USART6 pins
#define USART6_TX       PA11
#define USART6_RX       PA12
// USART settings
#define USART_CR2_STOPBITS_OFFSET       0x0C
#define USART_CR1_WORDLEN_OFFSET        0x0C
#define USART_PARITY_EN_OFFSET          0x0A
#define USART_PARITY_EVEN_ODD_OFFSET    0x09

#define USART_TX_MODE       USART_CR1_TE
#define USART_RX_MODE       USART_CR1_RE
#define USART_RX_TX_MODE    (USART_TX_MODE | USART_RX_MODE)
#define USART_EN            USART_CR1_UE
#define USART_LIN_EN        USART_CR2_LINEN

#define USART_CHECK_RX_MODE USART_RX_MODE

#define USART_STOPBITS_1    (0x00 << USART_CR2_STOPBITS_OFFSET)
#define USART_STOPBITS_0_5  (0x01 << USART_CR2_STOPBITS_OFFSET)
#define USART_STOPBITS_2    (0x02 << USART_CR2_STOPBITS_OFFSET)
#define USART_STOPBITS_1_5  (0x03 << USART_CR2_STOPBITS_OFFSET)

#define USART_PARITY_NEN    (0x00 << USART_PARITY_EN_OFFSET)
#define USART_PARITY_EN     (0x01 << USART_PARITY_EN_OFFSET)

#define USART_PARITY_EVEN   (0x00 << USART_PARITY_EVEN_ODD_OFFSET)
#define USART_PARITY_ODD    (0x01 << USART_PARITY_EVEN_ODD_OFFSET)

#define USART_TXE_TIMEOUT       100000
#define USART_RXNE_TIMEOUT      USART_TXE_TIMEOUT
// USART buffer length
#define USART_CHAR_BUFFER_LEN   255

// USART Interrup enable
#define USART_TXE_IE        (1 << 7)
#define USART_RXNE_IE       (1 << 5)

#define USART_FLAG_TXE      (1 << 7)        /*!< USART TX empty flag */
#define USART_FLAG_RXNE     (1 << 5)        /*!< USART RX not empty flag */

// USART TX/RX Stream numbers
#define USART1_TX_DMA_STREAM  	7
#define USART1_RX_DMA_STREAM 	5
#define USART2_TX_DMA_STREAM  	6
#define USART2_RX_DMA_STREAM 	5
#define USART6_TX_DMA_STREAM  	7
#define USART6_RX_DMA_STREAM 	2
// USARTx DMA channel
#define USART1_DMA_CHANNEL 	4
#define USART2_DMA_CHANNEL 	4
#define USART6_DMA_CHANNEL  5
// USARTx DMA controller
#define USART1_DMA 	DMA2
#define USART2_DMA 	DMA1
#define USART6_DMA 	DMA2

#define USART_BUF_SIZE  	    1024        /*!< USART buffer size for printf */
#define USART_IT_RX_BUF_SIZE    512         /*!< USART interrupt driven RX buffer size (Must be power of 2!!!)*/
#define USART_IT_TX_BUF_SIZE    512         /*!< USART interrupt driven TX buffer size (Must be power of 2!!!)*/


// Check if interrupt RX buffer is larger than 1 and power of 2
#if USART_IT_RX_BUF_SIZE < 2
#error USART_IT_RX_BUF_SIZE is too small. Needs to be larger than 1
#elif ((USART_IT_RX_BUF_SIZE & (USART_IT_RX_BUF_SIZE-1)) != 0)
#error USART_IT_RX_BUF_SIZE must be power of 2.
#endif

// Check if interrupt TX buffer is larger than 1 and power of 2
#if USART_IT_TX_BUF_SIZE < 2
#error USART_IT_TX_BUF_SIZE is too small. Needs to be larger than 1
#elif ((USART_IT_TX_BUF_SIZE & (USART_IT_TX_BUF_SIZE-1)) != 0)
#error USART_IT_TX_BUF_SIZE must be power of 2.
#endif


#define ENABLE      1
#define DISABLE     0
/**
 * USART buffer when interrupt driven USART
 */
typedef struct __usart_interrupt_buffer {
    char tx_buf[USART_IT_TX_BUF_SIZE];    /*!< TX buffer */
    char rx_buf[USART_IT_RX_BUF_SIZE];    /*!< RX buffer */
    uint32_t tx_in;                     /*!< tx buffer index in     */
    uint32_t tx_out;                    /*!< tx buffer index out    */
    uint32_t rx_in;                     /*!< RX buffer index in     */
    uint32_t rx_out;                    /*!< RX buffer index out    */
    bool tx_restart;                    /*!< restart TX buffer      */
} __usart_it_handle;

/**
 * @brief USART port settings structure
 * TODO: [ ] fix printf bug where it shows wrong big value on 64-bit integers
 */
typedef struct _USART_port {
    USART_TypeDef *usart;           /*!< USART port */
    __usart_it_handle *__it_buf;
    uint32_t baud;                  /*!< USART baud rate */
    uint32_t mode;                  /*!< USART mode */
    uint32_t stop_bits;             /*!< USART stop bit setting */
    uint32_t parity_enable;         /*!< USART enable parity */
    uint32_t parity_even_odd;       /*!< USART parity even or odd */
    bool interrupt_driven;          /*!< USART interrupt setting */
} USART_port;


/**
 * USART error codes
 */
typedef enum usart_err {
    USART_OK,               /*!< USART function success */
    USART_ERR_UNDEFINED,        /*!< USART port undefined (NULL) */
    USART_ERR_IT_BUF_FULL,      /*!< USART full interrupt mode buffer */
    USART_ERR_RX_EMPTY,
} usart_err_t;


static __usart_it_handle __buf_usart1 = {
    .rx_in = 0,
    .rx_out = 0,
    .tx_in = 0,
    .tx_out = 0,
    .tx_restart = true
};

static __usart_it_handle __buf_usart2 = {
    .rx_in = 0,
    .rx_out = 0,
    .tx_in= 0,
    .tx_out = 0,
    .tx_restart = true
};

static __usart_it_handle __buf_usart6 = {
    .rx_in = 0,
    .rx_out = 0,
    .tx_in = 0,
    .tx_out = 0,
    .tx_restart = true
};


uint16_t USART_compute_div(uint32_t periph_clk, uint32_t baud);


// NEW FUNCTIONS


usart_err_t USART_init(USART_port *port);

usart_err_t USART_write(USART_port port, int ch);
usart_err_t USART_printf(USART_port port, const char *format, ...);

int16_t USART_read(USART_port port);
uint8_t USART_getc(USART_port port);
usart_err_t USART_scan(USART_port port, char *buf, int len);
usart_err_t USART_input_echo(USART_port *port);
bool USART_available(USART_port port);

void USART_interrupt_enable(USART_port *port);
void USART_interrupt_disable(USART_port *port);

void USART_TXE_interrupt_enable(USART_port *port);
void USART_TXE_interrupt_disable(USART_port *port);

void USART_RXNE_interrupt_enable(USART_port *port);
void USART_RXNE_interrupt_disable(USART_port *port);

void USART_CLK_enable(USART_port *port);
void USART_CLK_disable(USART_port *port);

void USART_DMA_enable(USART_port *port);
void USART_DMA_disable(USART_port *port);

void USART_disable(USART_port *port);

usart_err_t __USART_IT_TX_BUF_LEN(__usart_it_handle *buf);
usart_err_t __USART_IT_RX_BUF_LEN(__usart_it_handle *buf);


#endif
