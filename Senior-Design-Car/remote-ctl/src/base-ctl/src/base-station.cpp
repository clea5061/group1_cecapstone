#include "RemoteCtlApp.h"

int main(int argc, char* argv[])
{
    RemoteCtlApp* ctl_app = NULL;
    try
    {
        ctl_app = new RemoteCtlApp();
    }
    catch (std::exception& exc)
    {
        printf("Fatal error: %s\nExiting...\n", exc.what());
        return EXIT_FAILURE;
    }

    int status = ctl_app->execute();

    delete ctl_app;
    return status;
}
