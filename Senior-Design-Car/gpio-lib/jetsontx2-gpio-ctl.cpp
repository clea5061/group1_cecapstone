#include "jetsontx2-gpio-ctl.h"
#include <string>
#include <errno.h>

using namespace jetsongpio;
using namespace std;

const char* GPIOCtl::PIN_TYPE_IN  = "in";
const char* GPIOCtl::PIN_TYPE_OUT = "out";

GPIOCtl::GPIOCtl()
{

}

GPIOCtl::~GPIOCtl()
{
    // TODO: disable all pins on object destruction
}

bool is_owner(struct PinState* pin_state, GPIOCtl* requestor, HeaderPin_t pin_num)
{
    if ((requestor != NULL) && (requestor != pin_state->owner))
    {
        printf("Error: GPIOCtl: Object at %p is not allowed to access pin %d", (void*)requestor, (int)pin_num);
        return false;
    }

    return true;
}

int GPIOCtl::enable_pin(HeaderPin_t pin_num, GPIOCtl* requestor)
{
    struct PinState* pin_state = NULL;

    try
    {
        pin_state = &pin_states.at(pin_num);
    }
    catch (exception& exc)
    {
        printf("Error: GPIOCtl.enable(): invalid pin number: %d\n", (int)pin_num);
        return -1;
    }

    string enable_pin(SYSFS_GPIO_PATH"/gpio");
    enable_pin += to_string((int)pin_num);

    if (pin_state->enabled || (access(enable_pin.c_str(), F_OK) == 0))
    {
        printf("Info: GPIOCtl.enable(): pin %d is already enabled. Nothing to do\n", (int)pin_num);
        pin_state->enabled = true;
        if (pin_state->owner == NULL)
        {
            pin_state->owner = requestor;
        }
        return 0;
    }

    int enable_file = open(SYSFS_GPIO_PATH"/export", O_WRONLY);
    if (enable_file == -1)
    {
        printf("Error: GPIOCtl.enable(): open(): failed to enable pin %d: %d\n", (int)pin_num, errno);
        return errno;
    }

    enable_pin = to_string((int)pin_num);

    if (write(enable_file, enable_pin.c_str(), enable_pin.length()) == -1)
    {
        printf("Error: GPIOCtl.enable(): write(): failed to enable pin %d: %d\n", (int)pin_num, errno);
        return errno;
    }
    else
    {
        pin_state->enabled = true;
        pin_state->pin_type = PinType_t::IN; // assume all pins are input. TODO: check what they are or force them to be input for safety
        pin_state->owner = requestor;
    }

    close(enable_file);

    return 0;
}

int GPIOCtl::pin_direction(HeaderPin_t pin_num, PinType_t pin_type, GPIOCtl* requestor)
{
    struct PinState* pin_state = NULL;

    try
    {
        pin_state = &pin_states.at(pin_num);
    }
    catch (exception& exc)
    {
        printf("Error: GPIOCtl.pin_direction(): invalid pin number: %d\n", (int)pin_type);
        return -1;
    }

    if (!pin_state->enabled)
    {
        printf("Error: GPIOCtl.pin_direction(): pin %d is not enabled\n", (int)pin_num);
        return -1;
    }

    if (!is_owner(pin_state, requestor, pin_num))
    {
        return -1;
    }

    string pin_path(SYSFS_GPIO_PATH"/gpio");
    pin_path += to_string((int)pin_num) + "/direction";

    int dir_file = open(pin_path.c_str(), O_WRONLY);
    if (dir_file == -1)
    {
        printf("Error: GPIOCtl.pin_direction(): open(): failed to set pin %d's type: %d\n", (int)pin_type, errno);
        return errno;
    }

    int status = 0;

    if (pin_type == PinType_t::IN)
    {
        status = write(dir_file, (void*)GPIOCtl::PIN_TYPE_IN, 2);
    }
    else
    {
        status = write(dir_file, (void*)GPIOCtl::PIN_TYPE_OUT, 3);
    }

    if (status == -1)
    {
        printf("Error: GPIOCtl.pin_direction(): write(): failed to set pin %d's type: %d\n", (int)pin_type, errno);
        return errno;
    }
    else
    {
        pin_state->pin_type = (int)pin_type;
    }

    close(dir_file);

    return 0;
}

int GPIOCtl::set_pin(HeaderPin_t pin_num, bool on, GPIOCtl* requestor)
{
    try
    {
        struct PinState& pin_state = pin_states.at(pin_num);

        if (pin_state.pin_type != PinType_t::OUT)
        {
            printf("Error: GPIOCtl.set_pin(): %d is not an output pin\n", (int)pin_num);
            return -1;
        }

        if (!pin_state.enabled)
        {
            printf("Error: GPIOCtl.set_pin(): pin %d is not enabled\n", (int)pin_num);
            return -1;
        }

        if (!is_owner(&pin_state, requestor, pin_num))
        {
            return -1;
        }
    }
    catch (exception& exc)
    {
        printf("Error: GPIOCtl.set_pin(): invalid pin number: %d\n", (int)pin_num);
        return -1;
    }

    string pin_path(SYSFS_GPIO_PATH"/gpio");
    pin_path += to_string((int)pin_num) + "/value";

    int set_file = open(pin_path.c_str(), O_WRONLY);
    if (set_file == -1)
    {
        printf("Error: GPIOCtl.set_pin(): open(): failed to set pin %d's state: %d\n", (int)pin_num, errno);
        return errno;
    }

    char state = '0';
    if (on)
    {
        state = '1';
    }

    if (write(set_file, (void*)&state, sizeof(char)) == -1)
    {
        printf("Error: GPIOCtl.set_pin(): write(): failed to set pin %d's state: %d\n", (int)pin_num, errno);
        return errno;
    }

    close(set_file);

    return 0;
}

int GPIOCtl::disable_pin(HeaderPin_t pin_num, GPIOCtl* requestor)
{
    struct PinState* pin_state = NULL;

    try
    {
        pin_state = &pin_states.at(pin_num);
    }
    catch (exception& exc)
    {
        printf("Error: GPIOCtl.disable(): invalid pin number: %d\n", (int)pin_num);
        return -1;
    }

    if (!pin_state->enabled)
    {
        printf("Info: GPIOCtl.disable(): pin %d is already disabled. Nothing to do\n", (int)pin_num);
        return 0;
    }

    if (!is_owner(pin_state, requestor, pin_num))
    {
        return -1;
    }

    int disable_file = open(SYSFS_GPIO_PATH"/unexport", O_WRONLY);
    if (disable_file == -1)
    {
        printf("Error: GPIOCtl.disable(): open(): failed to disable pin %d: %d\n", (int)pin_num, errno);
        return errno;
    }

    string pin_num_str = to_string(pin_num);

    if (write(disable_file, (void*)pin_num_str.c_str(), pin_num_str.length()) == -1)
    {
        printf("Error: GPIOCtl.disable(): write(): failed to disable pin %d: %d\n", (int)pin_num, errno);
        return errno;
    }
    else
    {
        pin_state->enabled = false;
        pin_state->pin_type = PinType_t::IN;
        pin_state->owner = NULL;
    }

    close(disable_file);

    return 0;
}

void GPIOCtl::enable_pin(HeaderPin_t pin_num)
{
    if (initialized)
    {
        pthread_mutex_lock(&write_lock);
        enable_pin(pin_num, this);
        pthread_mutex_unlock(&write_lock);
    }
    else
    {
        printf("Error: GPIOCtt.enable_pin(): GPIO is not initialized\n");
    }
}

void GPIOCtl::pin_direction(HeaderPin_t pin_num, PinType_t pin_type)
{
    if (initialized)
    {
        pthread_mutex_lock(&write_lock);
        pin_direction(pin_num, pin_type, this);
        pthread_mutex_unlock(&write_lock);
    }
    else
    {
        printf("Error: GPIOCtl.pin_direction(): GPIO is not initialized\n");
    }
}

void GPIOCtl::set_pin(HeaderPin_t pin_num, bool on)
{
    if (initialized)
    {
        pthread_mutex_lock(&write_lock);
        set_pin(pin_num, on, this);
        pthread_mutex_unlock(&write_lock);
    }
    else
    {
        printf("Error: GPIOCtl.set_pin(): GPIO is not initialized\n");
    }
}

void GPIOCtl::disable_pin(HeaderPin_t pin_num)
{
    if (initialized)
    {
        pthread_mutex_lock(&write_lock);
        disable_pin(pin_num, this);
        pthread_mutex_unlock(&write_lock);
    }
    else
    {
        printf("Error: GPIOCtl.disable_pin(): GPIO is not initialized\n");
    }
}

void GPIOCtl::initialize()
{
    if (!initialized)
    {
        if (access(SYSFS_GPIO_PATH"/export", R_OK | W_OK) == -1)
        {
            throw runtime_error(SYSFS_GPIO_PATH"/export: access denied. GPIO requires root privelege");
        }

        struct PinState pin_init_state;
        pin_init_state.enabled = false;
        pin_init_state.pin_type = PinType_t::IN;
        pin_init_state.owner = (GPIOCtl*)NULL;

        // all pins are initially in an unknown state with no owner
        pin_states[HeaderPin_t::GPIO_18] = pin_init_state;
        pin_states[HeaderPin_t::GPIO_29] = pin_init_state;
        pin_states[HeaderPin_t::GPIO_31] = pin_init_state;
        pin_states[HeaderPin_t::GPIO_33] = pin_init_state;
        pin_states[HeaderPin_t::GPIO_37] = pin_init_state;

        initialized = true;

        pthread_mutex_init(&write_lock, NULL);
    }
    else
    {
        printf("Info: GPIOCtl.initialize(): GPIO is already initialized. Nothing to do\n");
    }
}

void GPIOCtl::cleanup()
{
    if (initialized)
    {
        pthread_mutex_lock(&write_lock);

        for (auto& pin : pin_states)
        {
            if (pin.second.enabled)
            {
                if (pin.second.pin_type == PinType_t::OUT)
                {
                    set_pin(pin.first, false, NULL);
                }

                disable_pin(pin.first, NULL);
            }
        }

        initialized = false;

        pthread_mutex_unlock(&write_lock);
        pthread_mutex_destroy(&write_lock);
    }
    else
    {
        printf("Info: GPIOCtl.cleanup(): GPIO is not initialized. Nothing to do\n");
    }
}
