// SLAM based on recorded video

#include <opencv2/opencv.hpp>

// ORB-SLAM interface
#include "System.h"

#include <string>
#include <chrono>   // for time stamp
#include <iostream>

using namespace std;

string parameterFile = "/home/xinkaiz/projects/ato_slam_dev/Examples/ATO/ato.yaml";
string vocFile = "/home/xinkaiz/projects/ato_slam_dev/Vocabulary/ORBvoc.txt";

// video file
// string videoFile = "/home/xinkaiz/projects/ATO/data/circuit-20210813/mp4/2021-08-13-19-52-48.mp4";
string videoFile = "/home/xinkaiz/projects/ATO/data/circuit-20210813/mp4/2021-08-13-21-04-13.mp4";

int main(int argc, char **argv) {

    ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::MONOCULAR, true, 0, "circuit2");

    // get image from video
    cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.

    auto fps = cap.get(cv::CAP_PROP_FPS);
    auto totalframes = cap.get(cv::CAP_PROP_FRAME_COUNT);

    // std::cout << "VIDEO INFO: fps: " << fps << ", total frames: " << totalframes << std::endl;

    // recrd time
    auto start = chrono::system_clock::now();

    while (1) {
        cv::Mat frame;
        cap >> frame;
        if ( frame.data == nullptr ){
            std::cout << "FAILED TO LOAD VIDEO!" << std::endl;
            break;
        }
        // auto timestamp_mp4 = cap.get((cv::CAP_PROP_POS_MSEC));

        auto now = chrono::system_clock::now();

        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame, double(timestamp.count())/1000.0); //frame_resized
        // std:cout << "Time stamp now: mp4: " << timestamp_mp4 << ", chrono: " << double(timestamp.count())/1000.0 << std::endl;
        cv::waitKey(10);
    }

    // cout << "ttrack_tot = " << ttrack_tot << std::endl;
    // Stop all threads
    std::vector<ORB_SLAM3::MapPoint*> mapStuff = SLAM.GetAtlas()->GetCurrentMap()->GetAllMapPoints();
        // Map* GetCurrentMap();
        // mapStuff = SLAM.GetTrackedMapPoints();
        cout << "Start to write PCD with datapoints: " << endl;
        cout << mapStuff.size() << endl;
        // std::cout << "# x,y,z" << std::endl;
        string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append("ATO");
        pathSaveFileName.append(".pcd");
        std::remove(pathSaveFileName.c_str());
        std::ofstream ofs(pathSaveFileName, std::ios::binary);
        // boost::archive::text_oarchive oa(ofs);
        ofs  << "VERSION .7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4\n"
            << "TYPE F F F\n"
            << "COUNT 1 1 1\n"
            << "WIDTH "
            << mapStuff.size()
            << "\n"
            << "HEIGHT " << 1 << "\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS "
            << mapStuff.size()
            << "\n"
            << "DATA ascii\n";
	for (auto p : mapStuff) {
		Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();//ORB_SLAM3::Converter::toVector3d(p->GetWorldPos());
		// std::cout << v.x() << "," << v.y() << "," << v.z() << std::endl;
        ofs << v.x()  << " " << v.y()  << " " << v.z()  << "\n";
	}
    ofs.close();
    cout << "End to write PCD" << endl;
    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    cout << "Yo Shutting" << endl;

    // SLAM.Shutdown();
    return 0;
}
