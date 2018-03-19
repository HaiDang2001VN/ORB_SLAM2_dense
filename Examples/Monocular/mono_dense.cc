/**
Author: Hariharan Ramshankar
Application for producing an NVM file for use with Visual SFM and MVE.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<System.h>
#include<Optimizer.h>
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
        //cap>>frame;
        //cv::imshow("video",frame);
        cv::resize(frame,rframe,cv::Size(1920,1080),0,0,CV_INTER_LANCZOS4);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(rframe,tframe);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
       //cout<<(t2-t1).count()<<endl;
        tframe++;
        //cv::waitKey(1);
        //usleep(1000);
    }
    //Done with the Video
    cout<<"Done with Video processing"<<endl;
    cout<<"Stopping all threads"<<endl;
    // Stop all threads
    SLAM.Shutdown();

    cout<<"Saving Trajectory to file"<<endl;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    /////////////////////////////////
        ///////-----------------------
        ////   save all the keyframe images
        ///////------------------------
        /*
        string kfstrFile = ros::package::getPath("ORB_SLAM")+"/"+"testframe.jpg"; //set path for saving
        f.open(kfstrFile.c_str()); //open the file handle
        cv::Mat temp=cv::imread("/home/hari/Desktop/my_photo-24.jpg");
        cv::imshow("image",temp);
        cv::imwrite(kfstrFile.c_str(),temp);
        cout<<"Saving image";
        f.close();//close the file handle
        */
        //////////

        //--------------
        //  Export the Poses and the Features to a NVM file
        //--------------
        //adjust the scene first via global BA

        //cout << "Starting GlobalBA! This can take some minutes. \n";
        //ORB_SLAM2::Map* World=SLAM.GetMap();
        //ORB_SLAM2::Optimizer optim;
        //optim.GlobalBundleAdjustemnt(World,50);
        //ORB_SLAM::Optimizer::GlobalBundleAdjustemnt(&World,40);
        //cout<<"BA Done"<<endl;
        /*
        cout << endl << "Saving NVM to ORB_SLAM.nvm" << endl;
        //See http://ccwu.me/vsfm/doc.html#nvm for details about the file structure
        string nvmStrFile = ros::package::getPath("ORB_SLAM")+"/"+"ORB_SLAM.nvm";
        f.open(nvmStrFile.c_str());
        // fx cx fy cy;
        f << "NVM_V3 \n";
        //used for fixed cameras, but not supported yet by vsfm
        //"_KFIXED " << (double)fsSettings["Camera.fx"] << " " << (double)fsSettings["Camera.fy"] << " " <<
        //    (double)fsSettings["Camera.cx"] << " " << (double)fsSettings["Camera.cy"] << "\n";

        //Now: the model:
        //<Number of cameras>   <List of cameras>
        //<Number of 3D points> <List of points>
            <Camera> = <Image File name> <focal length> <quaternion WXYZ> <camera center> <radial distortion> 0
            <Point>  = <XYZ> <RGB> <number of measurements> <List of Measurements>
            with:
            <Measurement> = <Image index> <Feature Index> <xy>

        //1.------ Exort the cameras
        //1.1 count the amount of key frames
        int count_good_KF=0;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM::KeyFrame* pKF = vpKFs[i];

            if(pKF->isBad())
                continue;
            count_good_KF+=1;
        }
        cout<<count_good_KF<<"Good KFs";

        f << count_good_KF << "\n"; //now, the list of cameras follows

        //1.2 export the camera parameters itself
        //indexing of key frames by its consecutive number
        std::map<int,int> kf_index;
        int inc_frame_counter=-1;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM::KeyFrame* pKF = vpKFs[i];

            if(pKF->isBad())
                continue;
            inc_frame_counter+=1;

            cv::Mat R = pKF->GetRotation();//.t();
            vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            kf_index[pKF->mnFrameId]=inc_frame_counter;
            //
            cv::Mat temp= pKF->GetImage();
            string kfstrFile = ros::package::getPath("ORB_SLAM")+"/"; //set path for saving
            ostringstream kfstrFilen;
            kfstrFilen<<kfstrFile<<"frame"<<  formatInt(pKF->mnFrameId, 4) << ".jpg " ;
            cout<<kfstrFilen.str()<<endl;
            cv::imwrite(kfstrFilen.str(),temp);
            cout<<"Saving image"<<i<<endl;
            //
            f << "frame"<<  formatInt(pKF->mnFrameId, 4) << ".jpg " << (double)fsSettings["Camera.fx"] << " " <<
                q[3] << " " <<  q[0] << " " << q[1] << " " << q[2] << " " << //WXYZ
                t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " <<
                (double)fsSettings["Camera.k1"] << " " << (double)fsSettings["Camera.k2"] << "\n";
        }
        //f<< "\n";
        using namespace ORB_SLAM;
        //2. Export the 3D feature observations
        //<Number of 3D points> <List of points>
        //<Point>  = <XYZ> <RGB> <number of measurements> <List of Measurements>
        //<Measurement> = <Image index> <Feature Index> <xy>

        std::vector<MapPoint*> all_points=World.GetAllMapPoints();
        int count_good_map_points=0;
        for(size_t i=0, iend=all_points.size(); i<iend;i++)
        {
            if (!(all_points[i]->isBad()))
                count_good_map_points+=1;
        }
        cout<<"Good map points"<<count_good_map_points<<endl;
        f << count_good_map_points << "\n";
        for(size_t i=0, iend=all_points.size(); i<iend;i++)
        {
            if (all_points[i]->isBad())
                continue;

            MapPoint* pMP = all_points[i];
            cv::Mat pos=pMP->GetWorldPos();
            f << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << " " <<
            //rgb
            "0 0 0 ";
            //now all the observations/measurements
            std::map<KeyFrame*,size_t> observations=pMP->GetObservations();
            //count good observations:
            int num_good_observations=0;
            for (std::map<KeyFrame*,size_t>::iterator ob_it=observations.begin(); ob_it!=observations.end(); ob_it++)
            {
                if (!(*ob_it).first->isBad())
                    num_good_observations+=1;
            }

            f << num_good_observations << " ";
            for (std::map<KeyFrame*,size_t>::iterator ob_it=observations.begin(); ob_it!=observations.end(); ob_it++)
            {
                //skip if the key frame is "bad"
                if ((*ob_it).first->isBad())
                    continue;
                //<Measurement> = <Image index> <Feature Index> <xy>
                std::vector<cv::KeyPoint> key_points=(*ob_it).first->GetKeyPoints();
                f << kf_index[(*ob_it).first->mnFrameId] << " " << (*ob_it).second << " " <<
                key_points[ob_it->second].pt.x << " " <<
                key_points[ob_it->second].pt.y << " ";
            }
            f << "\n";

        }
        f.close();

    */
    /////////////////////////////////
    

    return 0;
}
