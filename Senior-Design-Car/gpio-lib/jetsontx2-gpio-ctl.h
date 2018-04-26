#ifndef __JETSONTX2_GPIO_CTL__
#define __JETSONTX2_GPIO_CTL__

#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <map>
#include <unistd.h>
#include <pthread.h>

namespace jetsongpio
{

#define TOTAL_GPIO_PINS 5                  ///< Reference value. How many pins total
#define SYSFS_GPIO_PATH "/sys/class/gpio"  ///< Reference and convenience value. Used to easily define GPIO file paths

/**
 * This maps the the pins by their number on the header to their actual pin number
 * on the Tegra chip. They are intended to disambiguate and ease the location of
 * GPIO pins by allowing to address them simply by their position on the header.
 * These are the values that should be used when making calls to enable,
 * disable, set, etc the pins on the board through the GPIO class.
 */
enum HeaderPin_t
{
    GPIO_29 = 398,
    GPIO_31 = 298,
    GPIO_33 = 389,
    GPIO_37 = 388,
    GPIO_18 = 481
};

/**
 * This is used to define whether a pin is an input or output pin when making
 * calls to GPIOCtl.pin_type().
 */
enum PinType_t
{
    OUT,
    IN
};

#define PIN_ON  true  ///< Reference value for use with GPIOCtl.set_pin(). Pins are set to ON / HIGH with a value of true.
#define PIN_OFF false ///< Reference value for use with GPIOCtl.set_pin(). Pins are set to OFF / LOW with a value of false.

/**
 * The PinState struct is used by the GPIOCtl class to keep track of a pin's
 * basic parameters to eliminate, for instance, attempts to write to a pin that
 * is actually an input.
 */
struct PinState;

/**
 * This class acts as a hardware abstraction layer of sorts for dealing with the
 * GPIO pins onboard the Jetson TX2. It should be noted that GPIO control requires
 * root access meaning that any process that makes use of this class to control
 * GPIO must be run as root either directly or with 'sudo'. Also, since there is
 * only one set of header pins, most of this class's functionality is actually
 * implemented using static members. Actual instances of this class are used to
 * to demarcate ownership of pins and prevent I/O clobbering within an application.
 *
 * To use this class, one must first make sure to initialize it using the static
 * initialize() method. This generates the initial ownership map; no pins are
 * considered owned in the initial map. Any instances of this class that attempt
 * to enable or otherwise modify pins before initialize() has been called will
 * explicity fail.
 *
 * After the class has been statically initialized, instances are allowed to
 * modify pins. In order to enable a pin, an instance shall call the enable()
 * method. If the pin is not enabled or is already enabled due to previous
 * applications incorrectly or failing to shut down the GPIO, then the requesting instance
 * becomes the owner of that pin meaning all other instances are not allowed to
 * access it. It should be noted that each instance is allowed to own multiple
 * pins.
 *
 * Next, after enabling and gaining ownership, a instance shall call the
 * pin_direction() method to set the pin's overall behavior. This must be done
 * for safety reasons before it is allowed to read or set the pin.
 *
 * After an instance is finished using a pin, it should call the disable() method
 * which will release ownership of the pin and ensure that it is safely disabled.
 * Either that or the instance should be deleted / go out of scope as the
 * destructor will automatically disable any owned pins.
 *
 * When an application is finished, it should call the static cleanup() function
 * which will disable all pins that were not previously disabled.
 *
 * This class is thread safe after static initialization. For as long as
 * there is an active operation on a pin, this class cannot be statically
 * cleaned up. Nor can any operations modify the same interior data structures
 * at the same time meaning concurrent calls to in particular enable() and
 * disable() will be mutually exclusive and no pin may be dual initialized
 * disabled.
 *
 * IMPORTANT CAVEATS TO THREAD AND ERROR SAFETY:
 *
 * 1. The safety features provided by this class only exist within the scope of
 * the current PROCESS. This class makes NO GUARANTEE that other process will be
 * prevented from accessing the GPIO.
 *
 * 2. This class only makes the assurance that GPIO will not be shut down while
 * there is an active operation but does NOT guarantee that cleanup() cannot
 * occur while instances still exist and intend to access the GPIO. Perhaps
 * counterintuitively, this is for safety to allow the current process to unconditionally
 * release control of the GPIO on termination.
 */
class GPIOCtl
{
private:
    static std::map<HeaderPin_t, struct PinState> pin_states; ///< Used for tracking the state and ownership of each pin
    static pthread_mutex_t write_lock; ///< Mutex to prevent data races when modifying pins
    static bool initialized; ///< True when this class has been statically initialized

    /**
     * This method is the static backend for enabling pins. For description of
     * its function, see the documentation for the public instance-bound method
     * of the same name.
     *
     * @param pin_num: number of the pin to be enabled
     * @param requestor: address of the object requesting the pin to be enabled
     * as given by the 'this' keyword in C++. This address will be used as the
     * identifier for ownership.
     *
     * @return Returns 0 on success or if the pin is already enabled. Returns -1
     * on failure due to safety violations defined by this class. Returns POSIX
     * error code for failure due to errors generated by the open() and write()
     * system calls used to implement its function.
     */
    static int enable_pin(HeaderPin_t pin_num, GPIOCtl* requestor); ///< Static backend for enabling pins

    /**
     * This method is the static backend for disabling pins. For description of
     * its function, see the documentation for the public instance-bound method
     * of the same name.
     *
     * @param pin_num: number of the pin to be enabled
     * @param requestor: address of the object requesting the pin to be disabled
     * as given by the 'this' keyword in C++. Only the pin's owner or the static
     * cleanup() method - which uses NULL as its ownership identifier - may disable
     * pins.
     * @return Returns 0 on success or if the pin is already disabled. Returns
     * -1 on failure to due to safety violations defined by this class. Returns
     * POSIX error code for failures generated by the open() and write() system
     * calls used to implement its function.
     */
    static int disable_pin(HeaderPin_t pin_num, GPIOCtl* requestor); ///< Static backend for disabling pins

    /**
     * This method is the static backend for changing a pin's type. For description
     * of its function see, the documentation for public instance-bound method
     * of the same name.
     *
     * @param pin_num: number of the pin to set type on
     * @param pin_type: input or output pin. uses PinType_t enum
     * @param requestor: address of the object requesting the pin to change type of
     * as given by the 'this' keyword in C++. Only the pin's owner or the static
     * cleanup() method - which uses NULL as its ownership identifier - may change
     * the type pins.
     * @return Returns 0 on success. Reutnrs -1 on failure due to safety
     * violations defined by this class. Returns POSIX error code for failures
     * generated by the open() and write() system calls used to implement its
     * function.
     */
    static int pin_direction(HeaderPin_t pin_num, PinType_t pin_type, GPIOCtl* requestor); ///< Static backend for changing pin types

    /**
     * This method is the static backend for chaning the state of output pins.
     * For a description of its function, see the documentation for the public
     * instance-bound method of the same name.
     *
     * @param pin_num: number of the pin to set the state of
     * @param on: true if the output is on. false if off
     * @param requestor: address of the object requesting the pin to have output set
     * as given by the 'this' keyword in C++. Only the pin's owner or the static
     * cleanup() method - which uses NULL as its ownership identifier - may change
     * the output of a pin.
     * @return Returns 0 on success. Returns -1 on failure due to safety violations
     * defined by this class. Returns POSIX error code for the failures generated
     * by the open() and write() system calls used to implement its function.
     */
    static int set_pin(HeaderPin_t pin_num, bool on, GPIOCtl* requestor); ///< Static backend for setting state of output pins

public:
    static const char* PIN_TYPE_OUT; ///< Text that is written to the system FS to set pin as an output
    static const char* PIN_TYPE_IN;  ///< Text that is written to the system FS to set pin as an input

    /**
     * On instance construction, the object is considered to own no pins. Calls
     * to enable() are required to gain control of pins.
     */
    GPIOCtl();

    /**
     * The instance destructor will automatically disable all pins that the
     * instance owns.
     */
    ~GPIOCtl();

    /**
     * The initialize() method sets up the pin_states map which internally
     * tracks the state and ownership of each pin. It must be called before GPIO
     * can be accessed using this class. If the process that uses this class does
     * not posses root privelege, a C++ runtime_error exception will be thrown
     * as this class cannot function without root privelege to access the /sys/
     * filesystem. Initially, all pins are considered unowned. The static
     * ownership identifier is NULL which has unlimited access to pins.
     */
    static void initialize(); ///< Static initializer. Must be called for GPIO to be used.

    /**
     * The cleanup() method disables every pin that is not yet disabled thus
     * releasing control of the GPIO and leaving the /sys/ filesystem in a clean
     * state for use by other applications. This method can be called at any
     * time as long as no GPIO operation is in progress. It makes NO GUARANTEE
     * that instances will not still attempt to access GPIO after cleanup.
     */
    static void cleanup();    ///< Static destructor. Should be called before application closes to fully release GPIO.

    /**
     * This method attempts to enable the given pin and make the invoking instance
     * its owner if possible. If the pin is already enabled and ownerless, the
     * invoking instance will take ownership of the pin. If the pin is already
     * enabled and the invoking instance is the owner of the pin, nothing will
     * be done. If the pin is already enabled and the invoking instance is NOT
     * the owner of the pin, then this function will explicitly fail. If errors
     * arise during the access of the /sys/ filesystem for controlling the pin,
     * this function will explicitly fail.
     *
     * @param pin_num: Pin's number on the Tegra chip. Use HeaderPin_t enum to
     * map the header pin to the Tegra pin.
     */
    void enable_pin(HeaderPin_t pin_num); ///< Enables the given pin if possible

    /**
     * This method attempts to disable the given pin and release ownership of the
     * pin from the invoking instance if possible. If the pin is already disabled,
     * nothing will be done. If the invoking instance is NOT the owner of the pin
     * and is not the static class, then this function will explicitly fail. If errors
     * arise during the access of the /sys/ filesystem for controlling the pin,
     * this function will explicitly fail.
     *
     * @param pin_num: Pin's number on the Tegra chip. Use HeaderPin_t enum to
     * map the header pin to the Tegra pin.
     */
    void disable_pin(HeaderPin_t pin_num); ///< Disables the given pin if possible

    /**
     * This method attempts to change the given pin's type if possible. If the
     * the pin is already the same type as the type given, this function will have
     * no effect. If the invoking instance is not the owner of the pin and is not
     * the static class, the this function will explicitly fail. If errors arise
     * during the access of the /sys/ filesystem for controlling the pin, this
     * function will explicity fail.
     *
     * @param pin_num: Pin's number on the Tegra chip. Use HeaderPin_t enum to
     * map the header pin to the Tegra pin.
     * @param pin_type: Input or output pin. Use PinType_t enum to correctly
     * signal the pin's requested type.
     */
    void pin_direction(HeaderPin_t pin_num, PinType_t pin_type); ///< Changes the pin's type if possible

    /**
     * This method attempts to set the given output pin to the given state. If
     * the pin is not an output pin, then this method has no effect. If the
     * invoking instance is not the owner of the pin and is not the static class,
     * then this function will explicitly fail. If erros arise during access of
     * the /sys/ filesystem for controlling the pin, this function will explicitly
     * fail.
     *
     * @param pin_num: Pin's number on the Tegra chip. Use HeaderPin_t enum to
     * map the header pin to the Tegra pin.
     * @param on: True if the pin is to be on. False if off.
     */
    void set_pin(HeaderPin_t pin_num, bool on); ///< Sets the given output to to the given state if possible
};

struct PinState
{
    bool enabled; ///< True when a pin has been exported / enabled successfully
    int pin_type; ///< Tracks the pin type. Uses the values directly from PinType_t enum
    GPIOCtl* owner; ///< Tracks which object owns this pin to prevent clobbering and unauthroized access
};

} // end namespace

#endif
