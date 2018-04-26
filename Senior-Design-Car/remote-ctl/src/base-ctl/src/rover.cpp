#include "ros/ros.h"
#include "RemoteCtlApp.h"
#include "RoverApp.h"
#include <string>
#include <cstring>
#include <iostream>
#include "ArduinoMessenger.h"
#include <cstdlib>
#include <cmath>

using namespace std;

int main(int argc, char* argv[])
{
    string addr_str;
    struct in_addr host_addr;
    RoverApp* rover = NULL;
    bool debug_out = false;

    if (argc > 1)
    {
        if (strcmp(argv[1], "--help") == 0)
        {
            printf("Usage:\n"
                   "  base-ctl_node [ip_address] [ROS_opts] ...\n"
                   "\n"
                   "Description:\n"
                   "  Runs the base control node. If an IPv4 address is given as the first argument,\n"
                   "  the node will attempt to connect to a remote control host at that address. All\n"
                   "  options after this are considered ROS launch options. If the first option is\n"
                   "  not a valid IPv4 address, the program will immediately exit.\n"
                   "\n"
                   "  When no options are given, the user will be prompted for the address of the\n"
                   "  control host.\n"
                   "\n"
                   "Options:\n"
                   "  -d      - include debug output\n"
                   "  --help  - displays this help message and exits\n");

             return EXIT_SUCCESS;
        }
        if (strcmp(argv[1], "-d") == 0)
        {
            debug_out = true;
            argc--;
            argv += sizeof(char**);
        }
        if (!inet_aton(argv[1], &host_addr))
        {
            printf("%s is not a valid IP address. Exiting.\n", argv[1]);
            return EXIT_FAILURE;
        }

        argc--;
        argv += sizeof(char**);
    }
    else
    {
        do
        {
            printf("Enter IPv4 address of control host (e.g. \"192.168.1.1\") > ");
            memset((void*)&host_addr, 0, sizeof(struct in_addr));
            cin >> addr_str;

        } while (!inet_aton(addr_str.c_str(), &host_addr));
    }

    try
    {
        ros::init(argc, argv, "base_ctl");
        rover = new RoverApp(host_addr, debug_out);
    }
    catch (exception& exc)
    {
        printf("%s\nFatal error. Exiting.\n", exc.what());
        return EXIT_FAILURE;
    }

    rover->recieve_cmds();
    delete rover;

    return EXIT_SUCCESS;
}
