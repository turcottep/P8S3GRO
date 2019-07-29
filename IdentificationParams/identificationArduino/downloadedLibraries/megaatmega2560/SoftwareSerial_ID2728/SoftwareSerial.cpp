#include "mbed.h"
#include "SoftwareSerial.h"

namespace chag {

/*unsigned char SoftwareSerial::inbuf[SOFTWARE_SERIAL_RX_BUFFER_SIZE];
unsigned char SoftwareSerial::qin = 0;
unsigned char SoftwareSerial::qout = 0;
char SoftwareSerial::flag_rx_waiting_for_stop_bit;
char SoftwareSerial::flag_rx_off;
char SoftwareSerial::rx_mask;
char SoftwareSerial::flag_rx_ready;
char SoftwareSerial::flag_tx_ready;
char SoftwareSerial::timer_rx_ctr;
char SoftwareSerial::timer_tx_ctr;
char SoftwareSerial::bits_left_in_rx;
char SoftwareSerial::bits_left_in_tx;
char SoftwareSerial::rx_num_of_bits;
char SoftwareSerial::tx_num_of_bits;
char SoftwareSerial::internal_rx_buffer;
char SoftwareSerial::internal_tx_buffer;
char SoftwareSerial::user_tx_buffer;
*/

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);


SoftwareSerial::SoftwareSerial(PinName tx, PinName rx, const char *name) {
    // save pins
    
    this->tx = (tx == NC) ? (DigitalOut *)NULL : new DigitalOut(tx);
    this->rx = (rx == NC) ? (DigitalIn *)NULL : new DigitalIn(rx);
    
    flag_tx_ready = false;
    flag_rx_ready = false;
    flag_rx_waiting_for_stop_bit = false;
    flag_rx_off = (rx == NC);
    rx_num_of_bits = 10;
    tx_num_of_bits = 10;

    set_tx_pin_low();

    baud(9600);
}

void SoftwareSerial::baud(int baudrate) {
    baud_rate = baudrate;
    ticker.detach();
  
    ticker.attach_us(this, &SoftwareSerial::timer_isr, 1000000.0 / (baudrate * 3.0));
    
    //timer_set( baud_rate );   //    Sets the timer to 3 times the baud rate.
    //set_timer_interrupt( &SoftwareSerial::timer_isr );
}

// putc, etc

int SoftwareSerial::readable() {
    return( kbhit() );
}

int SoftwareSerial::writeable() {
    return ( 1 );
}

// end


int SoftwareSerial::get_rx_pin_status() {
    //    Returns 0 or 1 dependent on whether the receive pin is high or low.
    return( rx->read() );
}

void SoftwareSerial::set_tx_pin_high() {
    //    Sets the transmit pin to the high state.
    tx->write(1);
    //led4 = 1;
}

void SoftwareSerial::set_tx_pin_low() {
    //    Sets the transmit pin to the low state.
    tx->write(0);
    //led4 = 0;
}

void SoftwareSerial::idle() {
    //    Background functions to execute while waiting for input.
    
}


// protected/private members

void SoftwareSerial::timer_isr( void ) {
    char mask, start_bit, flag_in;

    // Transmitter Section
    if ( flag_tx_ready ) {
        if( --timer_tx_ctr<=0 ) {
            mask = internal_tx_buffer&1;
            internal_tx_buffer >>= 1;
            if ( mask ) {
                set_tx_pin_high();
            } else {
                set_tx_pin_low();
            }
            timer_tx_ctr = 3;
            if ( --bits_left_in_tx<=0 ) {
                flag_tx_ready = false;
            }
        }
    }
    
    // Receiver Section
    if ( flag_rx_off==false ) {
        if ( flag_rx_waiting_for_stop_bit ) {
            if ( --timer_rx_ctr<=0 ) {
                flag_rx_waiting_for_stop_bit = false;
                flag_rx_ready = false;
                internal_rx_buffer &= 0xFF;
                if ( internal_rx_buffer!=0xC2 ) {
                    inbuf[qin] = internal_rx_buffer;
                    if ( ++qin>=SOFTWARE_SERIAL_RX_BUFFER_SIZE ) {
                        qin = 0;
                    }
                }
            }
        } else {    // rx_test_busy
            if ( flag_rx_ready==false ) {
                start_bit = get_rx_pin_status();
                // Test for Start Bit
                if ( start_bit==0 ) {
                    flag_rx_ready = true;
                    internal_rx_buffer = 0;
                    timer_rx_ctr = 4;
                    bits_left_in_rx = rx_num_of_bits;
                    rx_mask = 1;
                }
            } else {    // rx_busy
                if ( --timer_rx_ctr<=0 ) {                // rcv
                    timer_rx_ctr = 3;
                    flag_in = get_rx_pin_status();
                    if ( flag_in ) {
                        internal_rx_buffer |= rx_mask;
                    }
                    rx_mask <<= 1;
                    if ( --bits_left_in_rx<=0 ) {
                        flag_rx_waiting_for_stop_bit = true;
                    }
                }
            }
        }
    }
}

int SoftwareSerial::getc( void ) {
    char        ch;

    do {
        while ( qout==qin ) {
            idle();
        }
        ch = inbuf[qout] & 0xFF;
        if ( ++qout>=SOFTWARE_SERIAL_RX_BUFFER_SIZE ) {
            qout = 0;
        }
    } while ( ch==0x0A || ch==0xC2 );
    return( (int)ch );
}

int SoftwareSerial::putc( int c ) {
    led1 = !led1;
    while ( flag_tx_ready );
    user_tx_buffer = c;

    // invoke_UART_transmit
    timer_tx_ctr = 3;
    bits_left_in_tx = tx_num_of_bits;
    internal_tx_buffer = (user_tx_buffer<<1) | 0x200;
    flag_tx_ready = true;
    return(1);
}

void SoftwareSerial::flush_input_buffer( void ) {
    qin = 0;
    qout = 0;
}

char SoftwareSerial::kbhit( void ) {
    return( qin!=qout );
}

void SoftwareSerial::turn_rx_on( void ) {
    flag_rx_off = false;
}

void SoftwareSerial::turn_rx_off( void ) {
    flag_rx_off = true;
}


} // end namespace