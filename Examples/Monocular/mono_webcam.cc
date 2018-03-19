/**
Author: Hariharan Ramshankar
*/

#include <unistd.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

//void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_webcam path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

/*
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();
*/
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    //vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    //cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat frame,rframe;
    //cout<<argv[3]<<endl;
    //std::string path_to_video="/home/hariharan/Desktop/Projects/ORB_SLAM2_dense/Video/DJI_0128.MOV";
    //cv::VideoCapture cap(argv[3]);
    cv::VideoCapture cap(0);
    if(!cap.isOpened())
    {
        cout<<"Unable to read video!"<<endl;
        exit(1);
    }
    double tframe=0;
    while(true)
    {
        //cout<<tframe<<endl;
        if(tframe>50.0)
        {
            break;
        }
        cap>>frame;
        //cv::imshow("video",frame);
        //cv::resize(frame,rframe,cv::Size(1920,1080),0,0,CV_INTER_LANCZOS4);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
       //cout<<(t2-t1).count()<<endl;
        tframe++;
        //cv::waitKey(30);
        //usleep(30);
    }
    cap.release();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
