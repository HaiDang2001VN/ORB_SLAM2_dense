/**
Author: Hariharan Ramshankar
test Application for loading an NVM file from Visual SFM.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cstdlib> //For system calls

#include <opencv2/core/core.hpp>
#include <System.h>
//#include<Optimizer.h>
using namespace std;

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./mono_nvm_load path_to_vocabulary path_to_settings path_to_nvm" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    cv::Mat frame=cv::imread("frame0000.jpg");
    double tframe=0;
    while(tframe<10)
    {
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, tframe);
        tframe++;
    }
    SLAM.Shutdown();
    return 0;
}
