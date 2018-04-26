#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <sys/time.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    LaneDetector* detector = NULL;

    try
    {
        ros::init(argc, argv);
        detector = new LaneDetector();
    }
    catch (exception& exc)
    {
        printf("%s\nFatal error. Exiting\n", exc.what());
        return EXIT_FAILURE;
    }

    printf("Lane detector. Now publishing output\n");

    detector->lane_guidance();
    delete detector;

    return EXIT_SUCCESS;
}
