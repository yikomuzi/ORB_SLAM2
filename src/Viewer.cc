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

#include "Viewer.h"
#include <pangolin/pangolin.h>

//#include <mutex>

namespace ORB_SLAM2 {

    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
                   const string &strSettingPath) :
            mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
            mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
//            menuFollowCamera("menu.Follow Camera", true, true),
//            menuShowPoints("menu.Show Points", true, true),
//            menuShowKeyFrames("menu.Show KeyFrames", true, true),
//            menuShowGraph("menu.Show Graph", true, true),
//            menuLocalizationMode("menu.Localization Mode", false, true),
//            menuReset("menu.Reset", false, false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
//        if (mImageWidth < 1 || mImageHeight < 1) {
//            mImageWidth = 640;
//            mImageHeight = 480;
//        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];


        // 原本在run函数里的初始化代码
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 500, 500);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

        // Define Camera Render Object (for view / scene browsing)
        s_cam = pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(500, 500, mViewpointF, mViewpointF, 250, 250, 0.0001, 10000000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, 0, 1)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0, 1.0, -500.0f / 500.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        Twc.SetIdentity();

        cv::namedWindow("ORB-SLAM2: Current Frame");

        bFollow = true;
        bLocalizationMode = false;

    }

    void Viewer::Run_once() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

//        if (menuFollowCamera && bFollow) {
//            s_cam.Follow(Twc);
//        } else if (menuFollowCamera && !bFollow) {
//            s_cam.SetModelViewMatrix(
//                    pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
//            s_cam.Follow(Twc);
//            bFollow = true;
//        } else if (!menuFollowCamera && bFollow) {
//            bFollow = false;
//        }

//        if (menuLocalizationMode && !bLocalizationMode) {
//            mpSystem->ActivateLocalizationMode();
//            bLocalizationMode = true;
//        } else if (!menuLocalizationMode && bLocalizationMode) {
//            mpSystem->DeactivateLocalizationMode();
//            bLocalizationMode = false;
//        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
//        if (menuShowKeyFrames || menuShowGraph)
        mpMapDrawer->DrawKeyFrames(true, false);
//        if (menuShowPoints)
        mpMapDrawer->DrawMapPoints();
        mpMapDrawer->DrawTruth_poses();


        // 绘制原点坐标系
        glLineWidth(5);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();


        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame", im);
        cv::waitKey(mT);

//        if (menuReset) {
//            menuShowGraph = true;
//            menuShowKeyFrames = true;
//            menuShowPoints = true;
//            menuLocalizationMode = false;
//            if (bLocalizationMode)
//                mpSystem->DeactivateLocalizationMode();
//            bLocalizationMode = false;
//            bFollow = true;
//            menuFollowCamera = true;
//            mpSystem->Reset();
//            menuReset = false;
//        }

    }

    void Viewer::RequestFinish() {
//        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish() {
//        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop() {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested) {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release() {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}
