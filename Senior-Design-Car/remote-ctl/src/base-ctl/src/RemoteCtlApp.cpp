#include "RemoteCtlApp.h"

using namespace std;

void* connect_handler(void* app_ptr)
{
    RemoteCtlApp* app = (RemoteCtlApp*)app_ptr;

    pthread_mutex_lock(&app->write_lock);
    bool running = app->running;
    pthread_mutex_unlock(&app->write_lock);

    while (running)
    {
        struct sockaddr_in connection_addr;
        memset((void*)&connection_addr, 0, sizeof(struct sockaddr_in));

        socklen_t addr_bytes = sizeof(struct sockaddr_in);
        int connection_sock = accept(app->robot_socket, (struct sockaddr*)&connection_addr, &addr_bytes);

        if (connection_sock < 0)
        {
            printf("Error establishing new connection. accept() error: %d", errno);
        }
        else
        {
            printf("New connection from %s. ", inet_ntoa(connection_addr.sin_addr));

            pthread_mutex_lock(&app->write_lock);

            if (app->connection) // if already connected
            {
                printf("Already connected. Refusing\n");
                close(connection_sock);
            }
            else
            {
                printf("\n");
                app->connection = new pair<int, struct sockaddr_in>(connection_sock, connection_addr);
            }

            running = app->running;
            pthread_mutex_unlock(&app->write_lock);
        }
    }

    return NULL;
}

char gen_checksum(const char* buf, int size)
{
    char checksum = 0;

    for (int index = 0; index < size; index++)
    {
        checksum += buf[index];
    }

    return -checksum;
}

void* tx_handler(void* app_ptr)
{
    RemoteCtlApp* app = (RemoteCtlApp*)app_ptr;

    bool running = app->running;

    struct timeval systime;
    gettimeofday(&systime, NULL);
    long start_time = systime.tv_sec * 1000000 + systime.tv_usec;

    while (running)
    {
        if (app->connection)
        {
            char mesg[DEFAULT_MESG_SIZE];
            memset((void*)mesg, 0, DEFAULT_MESG_SIZE);

            pthread_mutex_lock(&app->write_lock);

            memcpy((void*)mesg, (void*)&app->gamepad_state, sizeof(struct GamepadState));
            mesg[sizeof(struct GamepadState)] = gen_checksum((char*)&app->gamepad_state,
                                                              sizeof(struct GamepadState));

            pthread_mutex_unlock(&app->write_lock);

            int bytes_sent = send(app->connection->first, (void*)mesg, DEFAULT_MESG_SIZE, MSG_NOSIGNAL);

            if (bytes_sent == -1)
            {
                if (errno == ECONNRESET)
                {
                    printf("Error sending gamepad state to %s. Connection reset by peer.\n",
                           inet_ntoa(app->connection->second.sin_addr));
                }
                else if (errno == EPIPE || errno == ENOTCONN)
                {
                    printf("Error sending gamepad state to %s. Connection dropped.\n",
                           inet_ntoa(app->connection->second.sin_addr));
                }
                else
                {
                    printf("Error sending gamepad state to %s. Unhandled tx error: %d. Dropping connection.\n",
                           inet_ntoa(app->connection->second.sin_addr), errno);
                }

                close(app->connection->first);
                delete app->connection;
                app->connection = NULL;
            }
        }

        gettimeofday(&systime, NULL);
        long current_time = systime.tv_sec * 1000000 + systime.tv_usec;
        long sleep_time = 10 * 1000 - (current_time - start_time);

        if (sleep_time > 0) // usleep takes an unsigned long. negative values will cause sleeping for thousands of years
        {
            usleep(sleep_time); // send at 100 Hz
        }

        gettimeofday(&systime, NULL);
        start_time = systime.tv_sec * 1000000 + systime.tv_usec;

        pthread_mutex_lock(&app->write_lock);
        running = app->running;
        pthread_mutex_unlock(&app->write_lock);
    }

    return NULL;
}

RemoteCtlApp::RemoteCtlApp()
{
    connection = NULL;

    int status = 0;
    status = SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);

    if (status < 0)
    {
        throw runtime_error(string("Failed to initialize SDL input library. SDL error: ") + to_string(status));
    }

    int joysticks_num = SDL_NumJoysticks();
    printf("Joysticks detected: %d\n", joysticks_num);

    if (joysticks_num >= 1)
    {
        gamepad = 0;
        printf("Attempting to use joystick %d\n", gamepad);
        sdl_gamepad = SDL_JoystickOpen(gamepad);
        SDL_JoystickEventState(SDL_ENABLE);

        if (sdl_gamepad)
        {
            printf("Error: %s\n"
                   "\n"
                   "*** Joystick control may be non-functional. Proceed with caution! ***\n"
                   "\n",
                   SDL_GetError());
        }
        else
        {
            printf("Success!\n");
        }
    }
    else
    {
        throw runtime_error("No usable input devices detected.");
    }

    memset((void*)&gamepad_state, 0, sizeof(struct GamepadState));

    printf("Setting up TCP socket for robots...\n");

    // create new internet socket, TCP protocol, infer correct protocol
    robot_socket = socket(AF_INET, SOCK_STREAM, 0);

    if (robot_socket < 0)
    {
        robot_socket = 0;
        throw runtime_error(string("Failed to create TCP socket. Error: ") + to_string(errno));
    }

    struct sockaddr_in socket_addr;
    socket_addr.sin_family = AF_INET;         // internet type socket
    socket_addr.sin_port = htons(5309);       // port 5309. need to use correct byte order
    socket_addr.sin_addr.s_addr = INADDR_ANY; // use this machine's IP address

    printf("TCP socket created. Binding to port 5309...\n");

    int binopt_on = 1;
    if (setsockopt(robot_socket, SOL_SOCKET, SO_REUSEADDR, &binopt_on, sizeof(int)) == -1)
    {
        close(robot_socket);
        robot_socket = 0;
        throw runtime_error(string("Failed to set socket options. setsockopt(): SO_REUSEADDR error: ") + to_string(errno));
    }

    // will not compile without that wierd looking type cast
    if (bind(robot_socket, (struct sockaddr*)&socket_addr, sizeof(struct sockaddr_in)) == -1)
    {
        close(robot_socket);
        robot_socket = 0;
        throw runtime_error(string("Failed to bind TCP socket to port. Error: ") + to_string(errno));
    }

    printf("TCP socket bound to port. Opening for connections...\n");

    // open the socket for at least one connection
    if (listen(robot_socket, 1) == -1)
    {
        close(robot_socket);
        robot_socket = 0;
        throw runtime_error(string("Failed to open TCP socket for connections. Error: ") + to_string(errno));
    }

    int timeout = 1500; // 1500 milliseconds
    if (setsockopt(robot_socket, IPPROTO_TCP, TCP_USER_TIMEOUT, &timeout, sizeof(int)) == -1)
    {
        close(robot_socket);
        robot_socket = 0;
        throw runtime_error(string("Failed to set socket options. setsockopt(): TCP_USER_TIMEOUT error: ") + to_string(errno));
    }

    if (pthread_mutex_init(&write_lock, NULL) == -1)
    {
        close(robot_socket);
        robot_socket = 0;
        throw runtime_error(string("Failed to create write protection mutex. Error: ") + to_string(errno));
    }

    running = true;

    if (pthread_create(&connect_thread, NULL, &connect_handler, (void*)this) == -1)
    {
        close(robot_socket);
        robot_socket = 0;
        throw runtime_error(string("Failed to create connection handler thread. Error: ") + to_string(errno));
    }

    if (pthread_create(&tx_thread, NULL, &tx_handler, (void*)this) == -1)
    {
        pthread_mutex_lock(&write_lock);
        running = false;
        pthread_mutex_unlock(&write_lock);

        pthread_cancel(connect_thread);
        pthread_join(connect_thread, NULL);

        pthread_mutex_destroy(&write_lock);

        close(robot_socket);
        robot_socket = 0;
        throw runtime_error(string("Failed to create tx handler thread. Error: ") + to_string(errno));
    }
}

RemoteCtlApp::~RemoteCtlApp()
{
    pthread_mutex_lock(&write_lock);
    running = false;
    pthread_mutex_unlock(&write_lock);

    pthread_cancel(connect_thread);
    pthread_join(connect_thread, NULL);
    pthread_join(tx_thread, NULL);

    pthread_mutex_destroy(&write_lock);
    shutdown(robot_socket, SHUT_RDWR);
    close(robot_socket);

    SDL_Quit();

    if (connection)
    {
        shutdown(connection->first, SHUT_RDWR);
        close(connection->first);

        delete connection;
    }
}

void RemoteCtlApp::_event(SDL_Event* event)
{
    switch (event->type)
    {
    case SDL_QUIT:
        printf("\nUser requested exit. Closing...\n");
        pthread_mutex_lock(&write_lock);
        running = false;
        pthread_mutex_unlock(&write_lock);
        break;

    case SDL_APP_TERMINATING:
        printf("\nInterrupt signal recieved. Terminating...\n");
        pthread_mutex_unlock(&write_lock);
        running = false;
        pthread_mutex_unlock(&write_lock);
        break;

    case SDL_JOYAXISMOTION:
        if (event->jaxis.which == gamepad)
        {
            int value = event->jaxis.value;
            if (value > 0) { value += 1; } // limit of short is 32,767. increment to make division by 32768 equal 1

            double axis_val = (double)value / 32768.0;
            pthread_mutex_lock(&write_lock);

            if (event->jaxis.axis == 0)
            {
                gamepad_state.axis_lx = axis_val;
            }
            else if (event->jaxis.axis == 1)
            {
                gamepad_state.axis_ly = axis_val;
            }
            else if (event->jaxis.axis == 2)
            {
                gamepad_state.axis_lt = axis_val;
            }
            else if (event->jaxis.axis == 3)
            {
                gamepad_state.axis_rx = axis_val;
            }
            else if (event->jaxis.axis == 4)
            {
                gamepad_state.axis_ry = axis_val;
            }
            else if (event->jaxis.axis == 5)
            {
                gamepad_state.axis_rt = axis_val;
            }

            pthread_mutex_unlock(&write_lock);
        }
        break;

    case SDL_JOYBUTTONDOWN:
        if (event->jbutton.which == gamepad)
        {
            if (event->jbutton.button < 32)
            {
                pthread_mutex_lock(&write_lock);
                gamepad_state.button[event->jbutton.button] = (char)0x01;
                pthread_mutex_unlock(&write_lock);
            }
        }

        break;

    case SDL_JOYBUTTONUP:
        if (event->jbutton.which == gamepad)
        {
            if (event->jbutton.button < 32)
            {
                pthread_mutex_lock(&write_lock);
                gamepad_state.button[event->jbutton.button] = (char)0x00;
                pthread_mutex_unlock(&write_lock);
            }
        }
        break;

    default:
        break;
    }
}

int RemoteCtlApp::execute()
{
    SDL_Event event;

    // while the user has no closed the application
    while (running)
    {
        int success = SDL_WaitEvent(&event);

        if (success == 1)
        {
            _event(&event);
            fflush(stdout);
        }
        else
        {
            pthread_mutex_lock(&write_lock);
            running = false;
            pthread_mutex_unlock(&write_lock);

            printf("SDL input error: %s\n"
                   "Critical error. Exiting...\n",
                   SDL_GetError());
        }
    }

    return EXIT_SUCCESS;
}
