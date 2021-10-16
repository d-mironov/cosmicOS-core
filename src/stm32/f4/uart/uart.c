#include "uart.h"
#include <stm32f4xx.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <core_cm4.h>

#include "../gpio/gpio.h"
#include "../rcc/rcc.h"


/**
 * USART
 * Initializes the USART with the given `port`
 * `mode`, `stop_bits`, `parity_enable` and `parity_even_odd` can 
 * remain on the default value if no change is needed.
 *
 * @param port USART port with settings
 *
 * @return usart_err_t `USART_UNDEFINED` if `port.usart=NULL`, `USART_OK` on success
 */
usart_err_t USART_init(USART_port *port) {
    if (port == NULL) {
        return USART_ERR_UNDEFINED;
    }

    if ( port->usart == USART1) {
        //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART1_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART1_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART1_RX, GPIO_AF07);
        GPIO_select_alternate(USART1_TX, GPIO_AF07);
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        (port->usart)->BRR = USART_compute_div(apb2_freq, port->baud); 
        port->__it_buf = &__buf_usart1;
    } else if (port->usart == USART2) {
        //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART2_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART2_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART2_RX, GPIO_AF07);
        GPIO_select_alternate(USART2_TX, GPIO_AF07);
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        (port->usart)->BRR = USART_compute_div(apb1_freq, port->baud); 
        port->__it_buf = &__buf_usart2;
    } else if (port->usart == USART6) {
        //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART6_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART6_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART6_RX, GPIO_AF07);
        GPIO_select_alternate(USART6_TX, GPIO_AF07);
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        (port->usart)->BRR = USART_compute_div(apb2_freq, port->baud); 
        port->__it_buf = &__buf_usart6;
        
    } else {
        return USART_ERR_UNDEFINED;
    }

    (port->usart)->CR1 = 0x00;
    (port->usart)->CR2 = 0x00;
    (port->usart)->CR3 = 0x00;


    (port->usart)->CR1 |= port->parity_enable | port->parity_even_odd;
    (port->usart)->CR2 |= port->stop_bits;
    if (port->interrupt_driven) {
        USART_interrupt_enable(port);
        if ((port->mode & USART_CHECK_RX_MODE) != 0) {
            (port->usart)->CR1 |= USART_RXNE_IE;
        }
    }
     
    if (port->mode == 0) {
        (port->usart)->CR1 |= USART_RX_TX_MODE;
    } else {
        (port->usart)->CR1 |= port->mode;
    }

    (port->usart)->CR1 |= USART_EN;

    return USART_OK; }

/**
 * USART write
 * \brief Writes one character to the Data register.
 * Mode to write depends on the `interrupt_driven` setting in the 
 * port struct
 *
 * @param port Port of USART (needs to be initialized)
 * @param ch character to write into data-register
 *
 * @return Error code (USART_OK on success, USART_IT_BUF_FULL on interrupt buffer overflow)
 */
usart_err_t USART_write(USART_port port, int ch) {
    if (!port.interrupt_driven) {
        while(!((port.usart)->SR & USART_SR_TXE));
        (port.usart)->DR = (ch & 0xFF);
        return USART_OK;
    } else {
        __usart_it_handle *buf;
        buf = port.__it_buf; 
        if (__USART_IT_TX_BUF_LEN(buf) != USART_OK) {
            return USART_ERR_IT_BUF_FULL;
        }
        buf->tx_buf[ buf->tx_in & (USART_IT_TX_BUF_SIZE-1) ] = ch;
        buf->tx_in++;
        if (buf->tx_restart) {
            buf->tx_restart = 0;
            (port.usart)->CR1 |= USART_FLAG_TXE;
        }
    }
    return USART_OK;
}

/**
 * USART computer divider
 * Computes divider on given peripheral clock speed and baud rate
 * 
 * @param periph_clk clock speed of given USART
 * @param baud baud rate
 */
uint16_t USART_compute_div(uint32_t periph_clk, uint32_t baud) {
    return (periph_clk + (baud/2U)) / baud; 
}


/**
 * Interrupt driven USART read function
 *
 * @param port USART port
 * @return -1 if no input, characte read on input
 */
int16_t USART_read(USART_port port) {
    char ch;
    
    __usart_it_handle *buf;
    buf = port.__it_buf; 
    if ((buf->rx_in - buf->rx_out) == 0) {
        return (-1);
    }
    return (buf->rx_buf[ (buf->rx_out++) & (USART_IT_RX_BUF_SIZE-1) ]);
    return ch;
}


uint8_t USART_getc(USART_port port) {
    volatile int ch;
    do {
        ch = USART_read(port);
    } while (ch == -1);
    //USART_write(port, ch);
    return ch;
}


/**
 * USART scan functions for inputs.
 * Stores input into given buffer when input is available.
 * Limited by the given number of bytes `len`.
 *
 *
 * Make sure that the buffer is big enough to store the number 
 * of characters in, otherwise the behaviour is undefined.
 *
 * @param port USART port
 * @param buf buffer to store input in
 * @param len number of bytes(characters) to store in buf
 *
 * @return usart_err_t `USART_OK` on success
 */
usart_err_t USART_scan(USART_port port, char *buf, int len) {
    volatile int buf_i = 0;
    int c = USART_read(port);
    while (c != -1) {
        if (buf_i >= len) {
            return USART_OK;
        }
        buf[ buf_i++ ] = c;
        buf[ buf_i ] = '\0';
        c = USART_read(port);
    }
    return USART_OK;
}


/**
 * USART printf function with argument formatting
 * Maximum formatted string length defined by `USART_CHAR_BUFFER_LEN` (default 1024)
 * 
 * @param port USART port to print to
 * @param format String to format
 * @param ... argument list
 *
 * @return USART error code (USART_OK on success, USART_IT_BUF_FULL on interrupt buffer overflow)
 */
usart_err_t USART_printf(USART_port port, const char *format, ...) {
    char buff[USART_CHAR_BUFFER_LEN];

    va_list args;
    va_start(args, format);
    
    vsprintf(buff, format, args);    

    for (int i = 0; i < strlen(buff); i++) {
        if ( buff[i] == '\n' && USART_write(port, '\r') != USART_OK) {
            return USART_ERR_IT_BUF_FULL;   
        }
        if (USART_write(port, buff[i]) != USART_OK) {
            return USART_ERR_IT_BUF_FULL;
        }
    }
    va_end(args);
    return USART_OK;
}


bool USART_available(USART_port port) {
    if (port.__it_buf->rx_in != port.__it_buf->rx_out) {
        return true;
    }
    return false;
}



/**
 * Enables interrupts for the given port
 * Using this when the `interrupt_driven` setting is not set
 * may lead to undefined behaviour
 * 
 * @param `port` USART port struct to enable interrupt
 */
inline void USART_interrupt_enable(USART_port *port) {
    if (port->usart == USART1) {
        NVIC_EnableIRQ(USART1_IRQn);
    } else if (port->usart == USART2) {
        NVIC_EnableIRQ(USART2_IRQn);
    } else if (port->usart == USART6) {
        NVIC_EnableIRQ(USART6_IRQn);
    }
}


/**
 * Disables interrupts for the given port
 * Using this when the `interrupt_driven` setting is set
 * may lead to undefined behaviour
 * 
 * @param `port` USART port struct to enable interrupt
 */
inline void USART_interrupt_disable(USART_port *port) {
    if (port->usart == USART1) {
        NVIC_DisableIRQ(USART1_IRQn);
    } else if (port->usart == USART2) {
        NVIC_DisableIRQ(USART2_IRQn);
    } else if (port->usart == USART6) {
        NVIC_DisableIRQ(USART6_IRQn);
    }
}


void USART_disable(USART_port *port) {
    (port->usart)->CR1 &= ~(USART_CR1_UE);
    if ( port->usart == USART1) {
        RCC->APB1RSTR |= RCC_APB2RSTR_USART1RST;
    } else if ( port->usart == USART2 ) {
        RCC->APB2ENR |= RCC_APB1RSTR_USART2RST;
    } else if ( port->usart == USART6 ) {
        RCC->APB1RSTR |= RCC_APB2RSTR_USART6RST;
    }
    
}


usart_err_t __USART_IT_TX_BUF_LEN(__usart_it_handle *buf) { 
    return (buf->tx_in - buf->tx_out >= USART_IT_TX_BUF_SIZE) ?
        (USART_ERR_IT_BUF_FULL) :
        (USART_OK);
}

usart_err_t __USART_IT_RX_BUF_LEN(__usart_it_handle *buf) { 
    return (buf->rx_in - buf->rx_out >= USART_IT_RX_BUF_SIZE) ?
        (USART_ERR_IT_BUF_FULL) :
        (USART_OK);
}

/**
 * Interrupt Request handler for the USART1 port
 */
void USART1_IRQHandler() {
    //TODO
    __usart_it_handle *buf;
    if (USART1->SR & USART_FLAG_TXE) {
        USART1->SR &= ~USART_FLAG_TXE;
        buf = &__buf_usart1;
        if (buf->tx_in != buf->tx_out) {
            USART1->DR = (buf->tx_buf[ buf->tx_out & (USART_IT_TX_BUF_SIZE-1)] & 0xFF); 
            buf->tx_out++;
            buf->tx_restart = false;
        } else {
            buf->tx_restart = true;
            USART1->CR1 &= ~USART_FLAG_TXE;
        }
    }

    if (USART1->SR & USART_FLAG_RXNE) {
        USART1->SR &= ~USART_FLAG_RXNE;
        //GPIO_write(PA8, GPIO_ON);
        buf = &__buf_usart1;
        if (((buf->rx_in - buf->rx_out) & ~(USART_IT_RX_BUF_SIZE-1)) == 0) {
            buf->rx_buf[ buf->rx_in & (USART_IT_RX_BUF_SIZE-1) ] = (USART1->DR & 0xFF);
            buf->rx_in++;
        }
    }
}

/**
 * Interrupt Request handler for the USART2 port
 */
void USART2_IRQHandler() {
    //GPIO_toggle(PB8);
    __usart_it_handle *buf;

    if (USART2->SR & USART_FLAG_TXE) {
        USART2->SR &= ~USART_FLAG_TXE;
        buf = &__buf_usart2;
        if (buf->tx_in != buf->tx_out) {
            USART2->DR = (buf->tx_buf[ buf->tx_out & (USART_IT_TX_BUF_SIZE-1)] & 0xFF); 
            buf->tx_out++;
            buf->tx_restart = false;
        } else {
            buf->tx_restart = true;
            USART2->CR1 &= ~USART_FLAG_TXE;
        }
    }

    if (USART2->SR & USART_FLAG_RXNE) {
        USART2->SR &= ~USART_FLAG_RXNE;
        buf = &__buf_usart2;
        if (((buf->rx_in - buf->rx_out) & ~(USART_IT_RX_BUF_SIZE-1)) == 0) {
            buf->rx_buf[ buf->rx_in & (USART_IT_RX_BUF_SIZE-1) ] = (USART2->DR & 0xFF);
            buf->rx_in++;
        }
    }
}

/**
 * Interrupt Request handler for the USART6 port
 */
void USART6_IRQHandler() {
    __usart_it_handle *buf;
    if (USART6->SR & USART_FLAG_TXE) {
        USART6->SR &= ~USART_FLAG_TXE;
        buf = &__buf_usart6;
        if (buf->tx_in != buf->tx_out) {
            USART6->DR = (buf->tx_buf[ buf->tx_out & (USART_IT_TX_BUF_SIZE-1)] & 0xFF); 
            buf->tx_out++;
            buf->tx_restart = false;
        } else {
            buf->tx_restart = true;
            USART6->CR1 &= ~USART_FLAG_TXE;
        }
    }

    if (USART6->SR & USART_FLAG_RXNE) {
        USART6->SR &= ~USART_FLAG_RXNE;
        buf = &__buf_usart6;
        if (((buf->rx_in - buf->rx_out) & ~(USART_IT_RX_BUF_SIZE-1)) == 0) {
            buf->rx_buf[ buf->rx_in & (USART_IT_RX_BUF_SIZE-1) ] = (USART6->DR & 0xFF);
            buf->rx_in++;
        }
    }
}
