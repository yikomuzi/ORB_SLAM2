/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    // Retrieve paths to images
//    string vstrImageFilenamesRGB;
//    string vstrImageFilenamesD;
//    vector<double> vTimestamps;
//    string strAssociationFilename = "/home/ubuntu/Desktop/ORB_SLAM2_study/ORB_SLAM2/Examples/RGB-D/associations/fr1_xyz.txt";
//    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
//    int nImages = vstrImageFilenamesRGB.size();
//    if (vstrImageFilenamesRGB.empty()) {
//        cerr << endl << "No images found in provided path." << endl;
//        return 1;
//    } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
//        cerr << endl << "Different number of images for rgb and depth." << endl;
//        return 1;
//    }

    string voc_path = "/home/ubuntu/Desktop/ORB_SLAM2_study/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    string config_path = "/home/ubuntu/Desktop/ORB_SLAM2_study/ORB_SLAM2/Examples/RGB-D/igibson.yaml";
    string datasets_path = "/home/ubuntu/Desktop/iGibson_study/igibson_dataset/01/";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voc_path, config_path, ORB_SLAM2::System::RGBD, true);


    // Main loop
    cv::Mat imRGB, imD;
    for (int ni = 1; ni <= 200; ni++) {
        // Read image and depthmap from file
        imRGB = cv::imread(string(datasets_path) + "/rgb/" + to_string(ni) + ".png", CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(datasets_path) + "/depth/" + to_string(ni) + ".png", CV_LOAD_IMAGE_UNCHANGED);
        cout << imRGB.channels() << endl;
        cout << imD.channels() << endl;
//        double tframe = vTimestamps[ni];

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, ni);
        // 如果要运行局部建图线程，需要把跟踪部分mCurrentFrame.mpReferenceKF->ComputeBoW(); 代码注释掉
//        SLAM.LocalMapping_run();
        SLAM.Viewer_run();

    }

    // Save camera trajectory
//    SLAM.SaveTrajectoryTUM("CameraTrajectory01.txt");
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory01.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory01_t.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory01_t.txt");

    // Stop all threads
    SLAM.Shutdown();


    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps) {
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while (!fAssociation.eof()) {
        string s;
        getline(fAssociation, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
