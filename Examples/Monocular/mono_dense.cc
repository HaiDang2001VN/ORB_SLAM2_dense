/**
Author: Hariharan Ramshankar
Application for producing an NVM file for use with Visual SFM and MVE.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
//#include<cstdlib> //For system calls

#include<opencv2/core/core.hpp>
#include<System.h>
//#include<Optimizer.h>
using namespace std;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_dense path_to_vocabulary path_to_settings path_to_video" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    // Main loop
    cv::Mat frame,rframe;
    cout<<argv[3]<<endl;
    //std::string path_to_video="/home/hariharan/Desktop/Projects/ORB_SLAM2_dense/Video/DJI_0128.MOV";
    cv::VideoCapture cap(argv[3]);
    //cv::VideoCapture cap(0);
    if(!cap.isOpened())
    {
        cout<<"Unable to read video!"<<endl;
        exit(1);
    }
    double tframe=0;
    while(cap.read(frame))
    {
        //cout<<tframe<<endl;
        //cap>>frame;
        //cv::imshow("video",frame);
        //cv::rotate(frame, frame, 0);
        //cv::resize(frame,rframe,cv::Size(640,480),0,0,CV_INTER_LANCZOS4);
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
        if(int(tframe*15.0)%50==0)
        {cout<<"Frame: "<<tframe*15.0<<" finished in "<<fp_ms1.count() << " ms" << std::endl;}
        tframe+=(1.0/15);
        /*if (cv::waitKey(1) == 'q')
        {
                break;
        }*/
        usleep(1000);
    }
    //Done with the Video
    cout<<"Done with Video processing"<<endl;
    //cap.release();
    cout<<"Stopping all threads"<<endl;
    // Stop all threads
    SLAM.Shutdown();

    //cout<<"Saving Trajectory to file"<<endl;
    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("idrisKeyFrameTrajectory.txt");

   SLAM.CreatePCD("testingdaniel2000.pcd");

   SLAM.SaveNVM("testingdaniel2000.nvm");

    //SLAM.DisplayKF(0);
    //Now for calling MVE

    return 0;
}
