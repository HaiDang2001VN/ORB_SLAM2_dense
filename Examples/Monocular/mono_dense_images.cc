/**
Author: Hariharan Ramshankar
Application for producing an NVM file for use with Visual SFM and MVE.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<cstdlib> //For system calls

#include<opencv2/core/core.hpp>
#include<System.h>
//#include<Optimizer.h>
using namespace std;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_dense path_to_vocabulary path_to_settings path_to_images" << endl;
        return 1;
    }

    vector<cv::String> fn;
    cv::glob(string(argv[3])+"/*.jpg", fn, false);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Number of images"<<fn.size()<<endl;
    // Main loop
    cv::Mat frame,rframe;
    double tframe=0;
    for(int ni=0; ni<fn.size(); ni++)
    {
        // Read image from file
        frame = cv::imread(fn[ni],CV_LOAD_IMAGE_UNCHANGED);
        //double tframe = ni;
        //cv::resize(frame,rframe,cv::Size(3840,2160),0,0,CV_INTER_LANCZOS4);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        //auto t1 = std::chrono::system_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame,tframe);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        //auto t2 = std::chrono::system_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        std::chrono::duration<double, std::milli> fp_ms1 = t2 - t1;
        if(int(tframe)%50==0)
        //cout<<"Frame: "<<tframe<<" finished in "<<fp_ms1.count() << " ms" << std::endl;
        tframe+=0.04;
        cv::waitKey(400);
        //usleep(1000);
    }
    //Done with the Video
    cout<<"Done with Images processing"<<endl;
    //cap.release();
    cout<<"Stopping all threads"<<endl;
    // Stop all threads
    SLAM.Shutdown();

    //cout<<"Saving Trajectory to file"<<endl;
    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    //SLAM.CreatePCD("pointcloudimage.pcd");

    //SLAM.CreateNVM("ORB_SLAM2image.nvm");

    //SLAM.DisplayKF(0);
    //Now for calling MVE

    return 0;
}

