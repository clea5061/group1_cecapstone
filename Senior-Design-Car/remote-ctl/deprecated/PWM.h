#ifndef __PWM__
#define __PWM__

#include <cstdio>
#include "jetsontx2-gpio-ctl.h"
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>

/**
 * THIS CLASS IS DEPRECATED! DO NOT USE!
 *
 * Please refer to and use the ArduinoPWM class.
 *
 * The PWMGen class is used for generating PWM signals on the GPIO pins of the
 * Jetson. By default, this class generates a servo PWM signal with a period of
 * 10.0 ms and a pulse width of 1.5 ms on the number 29 pin of the Jetson's
 * expansion header. To generate a PWM signal with this class, first you must
 * instantiate an object and then also call the start() method. If you do not
 * call the start, method the PWM signal will not be generated.
 *
 * To stop the PWM signal from being generated, call the stop() method. This is
 * necessary to adjust certain parameters such as which pin the signal is being
 * generated on. The PWM signal will automatically stop being generated when
 * this object is destroyed.
 *
 * If no GPIO control object is supplied upon object creation, this class will
 * generate its own. It is stronly advised that if your application requires
 * multiple accessors of the GPIO pins that you create the GPIO control outside
 * this class and pass it in on object construction.
 */
class PWMGen
{
    /**
     * This is a static non-member function for use with the pthread library
     * that generates the PWM signal in a background thread, thus freeing the
     * user from regular polling or blocking timers. It is considered a friend
     * of the PWMGen class which allows it to access private members,
     * essentially making it a member function that is also usable with pthread.
     *
     * @param generator: Pointer to the PWMGen object which houses the PWM signal
     * paramters
     *
     * @return Always returns NULL
     */
    friend void* pwm_loop(void* generator);

private:
    unsigned long period;      ///< Period of this PWM signal in microseconds
    unsigned long pulse_width; ///< Pulse width of this PWM signal in microseconds

    bool running;              ///< Used for checking if the background thread is running and for terminating it
    jetsongpio::GPIOCtl gpio;  ///< GPIO control. See gpio-lib docs.

    jetsongpio::HeaderPin_t pin; ///< Which pin the signal is being created on
    pthread_mutex_t write_lock;  ///< Mutex to prevent data races when changing PWM parameters

    /**
     * This is the average overshoot of the usleep() system call that is used to
     * generate the low pulse on the GPIO pin. Because Linux is a general
     * purpose operating system, the thread may oversleep and thus the PWM signal
     * will be fowled. This offset is used to correct the oversleep by averaging
     * it over time. Given in microseconds.
     */
    long period_offset;

    /**
     * This is the average overshoot of the usleep() system call that is used to
     * generate the high pulse on the GPIO pin. Because Linux is a general
     * purpose operating system, the thread may oversleep and thus the PWM signal
     * will be fowled. This offset is used to correct the oversleep by averaging
     * it over time. Given in microseconds.
     */
    long pulse_width_offset;
    long pulse_count;        ///< Number of pulses generated. Used for averaging oversleep. See *_offset members

    pthread_t pwm_thread; ///< pthread handle of for the background thread generating the signal

public:
    /**
     * Default constructor for a PWMGen object. This will instantiate its own
     * GPIO control object and sets default signal parameters of 10.0 ms period
     * and 1.5 ms pulse width.
     */
    PWMGen();

    /**
     * Constructor for when a GPIO control object already exists, such as when
     * an application (likely) has more than one part that needs GPIO access.
     * Sets default signal parameters of 10.0 ms period and 1.5 ms pulse width.
     *
     * @param gpio: reference to a master GPIO object
     */
    PWMGen(jetsongpio::GPIOCtl& gpio);

    /**
     * Destructor. Ensures that the PWM signal is no longer generated if it is
     * and that the pin is left at low voltage and disabled.
     */
    ~PWMGen();

    void start(); ///< Starts the PWM generation thread. MUST be called for PWM signal to be generated.
    void stop();  ///< Stops the PWM generation thread, sets pin to low, and disables it. Must be called before changing pin.

    long get_period();                 ///< Get the period in microseconds of this PWM signal
    long get_pulse_width();            ///< Get the pulse width in microseconds of this PWM signal
    jetsongpio::HeaderPin_t get_pin(); ///< Get the pin number that this PWM signal is generated on
    bool is_running();                 ///< Returns true if the signal is currently being generated

    jetsongpio::GPIOCtl& get_gpio(); ///< Get the GPIO control object of this PWM generator

    void set_period(long period);              ///< Set the period in microseconds of this PWM signal. May be done on the fly
    void set_pulse_width(long pulse_width);    ///< Set the pulse width in microseconds of this PWM signal May be done on the fly
    void set_pin(jetsongpio::HeaderPin_t pin); ///< Change the pin this signal is generated on. Signal generation must NOT be active.
};

#endif
