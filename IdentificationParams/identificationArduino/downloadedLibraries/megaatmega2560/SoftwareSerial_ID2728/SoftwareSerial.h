// modified from mbed library, original copyright:

/* mbed Microcontroller Library - Serial
 * Copyright (c) 2007-2011 ARM Limited. All rights reserved.
 */ 


#ifndef SOFTWARE_SERIAL_H
#define SOFTWARE_SERIAL_H

#ifndef SOFTWARE_SERIAL_RX_BUFFER_SIZE
#define SOFTWARE_SERIAL_RX_BUFFER_SIZE 256
#endif


#include "mbed.h"


namespace SoftwareSerial {

/* Class: SoftwareSerial
 *  A software implemented serial port (UART) for communication with other serial devices
 *
 * Can be used for Full Duplex communication, or Simplex by specifying 
 * one pin as NC (Not Connected)
 *
 * Example:
 * > // Print "Hello World" to the PC
 * >
 * > #include "mbed.h"
 * > #include "SoftwareSerial.h"
 * >
 * > SoftwareSerial ser(P19, P20);
 * >
 * > int main() {
 * >     ser.printf("Hello World\n");
 * > }
 */
class SoftwareSerial : public Stream {

public:

    /* Constructor: SoftwareSerial
     *  Create a SoftwareSerial port, connected to the specified transmit and receive pins
     *
     * Variables:
     *  tx - Transmit pin 
     *  rx - Receive pin
     *
     *  Note: Either tx or rx may be specified as NC if unused
     */
    SoftwareSerial(PinName tx, PinName rx, const char *name = NULL);

    /* Function: baud
     *  Set the baud rate of the serial port
     *  
     * Variables:
     *  baudrate - The baudrate of the serial port (default = 9600).
     */
    void baud(int baudrate);

    /*enum Parity {
        None = 0
        , Odd
        , Even
        , Forced1    
        , Forced0
    };*/

    enum IrqType {
        RxIrq = 0
        , TxIrq
    };

    /* Function: format
     *  Set the transmission format used by the Serial port
     *
     * Variables:
     *  bits - The number of bits in a word (5-8; default = 8)
     *  parity - The parity used (SoftwareSerial::None, SoftwareSerial::Odd, SoftwareSerial::Even, SoftwareSerial::Forced1, SoftwareSerial::Forced0; default = SoftwareSerial::None)
     *  stop - The number of stop bits (1 or 2; default = 1)
     */
    /*void format(int bits = 8, Parity parity = Serial::None, int stop_bits = 1); */

#if 0 // Inhereted from Stream, for documentation only

    /* Function: putc
     *  Write a character
     *
     * Variables:
     *  c - The character to write to the serial port
     */
    int putc(int c);


    /* Function: getc
     *  Read a character
     *
     * Reads a character from the serial port. This will block until 
     * a character is available. To see if a character is available, 
     * see <readable>
     *
     * Variables:
     *  returns - The character read from the serial port
     */
    int getc();

    /* Function: printf
     *  Write a formated string
     *
     * Variables:
     *  format - A printf-style format string, followed by the 
     *      variables to use in formating the string.
     */
    int printf(const char* format, ...);

    /* Function: scanf
     *  Read a formated string 
     *
     * Variables:
     *  format - A scanf-style format string,
     *      followed by the pointers to variables to store the results. 
     */
    int scanf(const char* format, ...);
 
#endif


#if 1
    /** putc
     * @param int c The byte to write.
     */
    int  putc(int c);
    
    /** puts
     * @param char * The string to print.
     */
    //void puts(char *s);
    
    /** getc
     * @return int c The byte read or -1 if no bytes to read.
     */
    int  getc(void); 
    
    /** gets
     * Get a string. Note, this method blocks until size bytes are read.
     * @param char *s where to place the incoming bytes.
     * @param int size How many bytes to read.
     * @return char * The value of *s passed in.
     */
    //char *gets(char *s, int size);   
    
    /** peek
     * like getc() but does NOT remove the byte from the buffer.
     * @see getc*(
     */
    //int  peek(void);
        
#endif
 
    /* Function: readable
     *  Determine if there is a character available to read
     *
     * Variables:
     *  returns - 1 if there is a character available to read, else 0
     */
    int readable();

    /* Function: writeable
     *  Determine if there is space available to write a character
     * 
     * Variables:
     *  returns - 1 if there is space to write a character, else 0
     */
    int writeable();

    /* Function: attach
     *  Attach a function to call whenever a serial interrupt is generated
     *
     * Variables:
     *  fptr - A pointer to a void function, or 0 to set as none
     *  type - Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    //void attach(void (*fptr)(void), IrqType type = RxIrq);

    /* Function: attach
     *  Attach a member function to call whenever a serial interrupt is generated
     *     
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     *  type - Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void), IrqType type = RxIrq) {
        if((mptr != NULL) && (tptr != NULL)) {
            _irq[type].attach(tptr, mptr);
            setup_interrupt(type);
        }
    }

#ifdef MBED_RPC
    //virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:

    //void setup_interrupt(IrqType type);
    //void remove_interrupt(IrqType type);

    //UARTName _uart;
    //FunctionPointer _irq[2];
    //int _uidx;
    
    virtual int _putc(int c) { return putc(c); }
    virtual int _getc()      { return getc(); }
    


    int get_rx_pin_status();
    void set_tx_pin_high();
    void set_tx_pin_low();
    void idle();
    void timer_set( int baud );
    void set_timer_interrupt( void (*timer_isr)(void) );
  
    
    
    void timer_isr( void );
    void init_uart( void );
    //char _getchar( void );
    //void _putchar( char ch );
    void flush_input_buffer( void );
    char kbhit( void );
    void turn_rx_on( void );
    void turn_rx_off( void );
    

private:
    DigitalOut *tx;
    DigitalIn *rx;
    
    Ticker ticker;
    
    int baud_rate;

    unsigned char inbuf[SOFTWARE_SERIAL_RX_BUFFER_SIZE];
    unsigned char qin;
    unsigned char qout;
    char flag_rx_waiting_for_stop_bit;
    bool flag_rx_off;
    char rx_mask;
    bool flag_rx_ready;
    bool flag_tx_ready;
    char timer_rx_ctr;
    char timer_tx_ctr;
    char bits_left_in_rx;
    char bits_left_in_tx;
    char rx_num_of_bits;
    char tx_num_of_bits;
    char internal_rx_buffer;
    char internal_tx_buffer;
    char user_tx_buffer;

};

} // namespace

using namespace SoftwareSerial;

#endif
