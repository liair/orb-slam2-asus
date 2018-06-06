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
#include<OpenNI.h>

#include<System.h>

using namespace std;
using namespace openni;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void CheckOpenNIError(Status rc, string status) {
    if (rc != STATUS_OK)
        cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_asus path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    OpenNI::initialize();
    Status rc = STATUS_OK;
    CheckOpenNIError(rc, "Initialize");

    Device xtion;
    const char *deviceURL = openni::ANY_DEVICE;
    rc = xtion.open(deviceURL);
    CheckOpenNIError(rc, "Open device");

    VideoStream streamDepth;
    rc = streamDepth.create(xtion, SENSOR_DEPTH);
    CheckOpenNIError(rc, "Creat depth stream");
    VideoMode ModeDepth;
    ModeDepth.setResolution(320, 240);
    ModeDepth.setFps(30);
    ModeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
    streamDepth.setVideoMode(ModeDepth);
    streamDepth.setMirroringEnabled(false);
    rc = streamDepth.start();
    CheckOpenNIError(rc, "Start depth steam...");

    VideoStream streamColor;
    rc = streamColor.create(xtion, SENSOR_COLOR);
    CheckOpenNIError(rc, "Creat color stream");
    VideoMode ModeColor;
    ModeColor.setResolution(320, 240);
    ModeColor.setFps(30);
    ModeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
    streamColor.setVideoMode(ModeColor);
    streamColor.setMirroringEnabled(false);
    rc = streamColor.start();
    CheckOpenNIError(rc, "Start color stream...");

    if (!streamColor.isValid() || !streamDepth.isValid()) {
        cerr << "...Error..." << endl;
        OpenNI::shutdown();
        return 1;
    }

    if (xtion.isImageRegistrationModeSupported(
            IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
        xtion.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    VideoFrameRef frameDepth;
    VideoFrameRef frameColor;
    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0;; ni++)
    {
        streamDepth.readFrame(&frameDepth);
        cv::Mat imD(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void *) frameDepth.getData());

        streamColor.readFrame(&frameColor);
        cv::Mat ImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void *) frameColor.getData());
        cv::Mat imRGB;
        cv::cvtColor(ImageRGB, imRGB, CV_RGB2BGR);
        if (imRGB.data == nullptr || imD.data == nullptr)
            continue;
        //#ifdef COMPILEDWITHC11
        //        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        //#else
        //        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        //#endif
        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,ni);

        //#ifdef COMPILEDWITHC11
        //        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        //#else
        //        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        //#endif
//
        if (cv::waitKey(1) == 27)
            break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
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
