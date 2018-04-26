#ifndef __ARDUINO_PWM__
#define __ARDUINO_PWM__

#include <termios.h>
#include <unistd.h>
#include <queue>
#include <pthread.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

struct PWMMesg
{
    unsigned char address;
    unsigned char bytes;
    unsigned char packets_num;
    unsigned char header;
    unsigned char checksum;
    unsigned char packets[16];
};

#define MAX_ADDR      8  ///< Only 8 mailboxes are addressable
#define MAX_MESG_SIZE 14 ///< 14 bytes maximum message size

/**
 * This class manages the passing of messages to the Arduino. The Arduino is to
 * have its serial line connected to a UART on the Jetson. This may be an onboard
 * UART or an auxiliary one such as an FTDI connected to USB.
 *
 * Upon construction, this class will open the serial line, set the correct baud,
 * and immediately allow for messages to be sent to the Arduino using a
 * specialized UART protocol. See LongDataUARTProtocol.md for more details on
 * this data exchange system. All messages are transmitted in a background
 * thread to prevent blocking and polling. If the serial line cannot be opened
 * or the background thread and associated data structures cannot be created,
 * the constructor of this class will throw a std::runtime_exception().
 *
 * Once constructed, the user can send the Arduino data of variable length up
 * to 14 bytes to various mailboxes - up to 8 - using the send_mesg() method.
 * You do NOT need multiple instances of this class to send data to different
 * mailboxes. See send_mesg() documentation for more details. Currently, the
 * class does not filter redudant messages or cap their arrival rate so the user
 * should be careful to not saturate the Arduino. The transmission rate is set
 * at 4800 baud and a message is a minimum of 4 bytes in length. Please note
 * that Arduino will need to have the motor-ctl.c project flashed onto it and
 * the code updated to register each mailbox you wish to send data to.
 *
 * This class's main use at present is to control the throttle and steering of
 * the car via PWM. Two-byte messages arriving in mailboxes 0 and 1 are the
 * exact microsecond values that the high pulse of the PWM should last for
 * throttle and steering respectively. 1500 is neutral; 2000 is full foward /
 * full right; 1000 is full backward / full left.
 *
 * When this class is destroyed, it will shutdown the background thread and close
 * the serial line if necessary.
 */
class ArduinoMessenger
{
    /**
     * Background loop for sending data to the Arduino. It is a static function
     * to allow it to be used with the pthread library. Its status as a friend
     * function allows it access to this class's private members and its argument
     * is a pointer to an instance of this class making it behave effectively
     * the same as a member function. On each iteration, the thread will extract
     * the next message from the queue and send it to the Arduino.
     *
     * @param arduino_pwm_ptr: pointer to an instance of the ArduinoMessenger class as
     * given by the 'this' keyword.
     * @return Always returns NULL.
     */
    friend void* send_mesg_loop(void* arduino_pwm_ptr); ///< background thread for sending data to the Arduino

private:
    struct termios uart_props; ///< Contains the serial line properties / settings
    int uart_gateway; ///< Unix file handle for the serial line
    bool running; ///< True while background thread should be running. Used to signal thread exit.

    std::queue<struct PWMMesg> mesg_queue; ///< Serial line message queue

    pthread_mutex_t write_lock; ///< Mutex for preventing data races on the queue and thread exit signaler
    pthread_cond_t new_mesg_sig; ///< The background thread sleeps if there is no data to send. This condition is used to wake it.
    pthread_mutex_t new_mesg_lock; ///< dummy mutex. required to use pthread conditions but this class manages its own mutexes for finer control
    pthread_t send_mesg_thread; ///< Background thread handle

public:

    ArduinoMessenger();
    ~ArduinoMessenger();

    /**
     * Translates a series of bytes into a properly formatted message to be sent
     * over the serial line. The message will be addressed to a certain mailbox
     * not to exceed 7 and must be shorter than 15 bytes. The motor speed is
     * controlled by messages arriving in mailbox 0 and the steering angle is
     * controlled by messages arriving in mailbox 1.
     *
     * Concerning PWM:
     * ---
     * On the Arduino side, the speeds and angles are controlled using a 16 bit
     * signed value which is simply the exact number of microseconds the high pulse
     * in the signal should last. 1000: Full back / Full left. 1500: Neutral.
     * 2000: Full forward / Full right.
     *
     * @param mesg: A serial line message struct to write into.
     * @param address: The mailbox the message is to be sent to. 0: motor speed;
     * 1: steering angle
     * @param buf: Byte stream to be transmitted. Must be a valid pointer.
     * @param size: The number of bytes in the buffer. Not to exceed 14.
     * @return Returns 0 on successfully creating the message. Returns -1 if the
     * address is out of range or the size is too large.
     */
    int buf_to_mesg(struct PWMMesg& mesg, int address, const void* buf, int size); ///< Translates a byte stream into a serial line message

    /**
     * Sends a byte stream over serial. Uses buf_to_mesg() on the back end; see
     * its documentation for the behavior of this function.
     *
     * This function shall wake the background thread if it is sleeping due to
     * lack of data.
     *
     * @param address: The mailbox the message is to be sent to.
     * @param buf: By stream to be transmitted.
     * @param size: The number of bytes in the buffer.
     * @return Returns 0 on success. Returns -1 on failure.
     */
    int send_mesg(int address, const void* buf, int size); ///< Sends the given byte stream over serial

    /**
     * Sends a serial message struct over serial. This struct shall have been
     * created with buf_to_mesg(); see its documentation for the behavior of
     * this function.
     *
     * This function shall wake the background thread if it is sleeping due to
     * lack of data.
     *
     * @param mesg: serial message.
     * @return Returns 0 on success. Returns -1 on failure.
     */
    int send_mesg(const struct PWMMesg& mesg); ///< Sends the given message over serial
};

#endif
