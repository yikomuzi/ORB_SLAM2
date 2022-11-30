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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
//#include <mutex>

namespace ORB_SLAM2 {

    template<class T>
    bool read_all_number_txt(const std::string txt_file_name,
                             Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat) {
        if (!std::ifstream(txt_file_name)) {
            std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
            return false;
        }
        std::ifstream filetxt(txt_file_name.c_str());
        int row_counter = 0;
        std::string line;
        if (read_number_mat.rows() == 0)
            read_number_mat.resize(100, 10);

        while (getline(filetxt, line)) {
            T t;
            if (!line.empty()) {
                std::stringstream ss(line);
                int colu = 0;
                while (ss >> t) {
                    read_number_mat(row_counter, colu) = t;
                    colu++;
                }
                row_counter++;
                if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                    read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
            }
        }
        filetxt.close();

        read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows

        return true;
    }

    MapDrawer::MapDrawer(Map *pMap, const string &strSettingPath) : mpMap(pMap) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

        // 读取真实的相机位姿路径
        std::string truth_pose_txts = "/home/ubuntu/Desktop/cube_slam_study/dataset/seq_07/pose_truth.txt";
        Eigen::MatrixXd truth_cam_poses(5, 8);
        if (read_all_number_txt(truth_pose_txts, truth_cam_poses)) {
            truth_poses.resize(truth_cam_poses.rows() / 10, 3);
            for (int i = 0; i < truth_poses.rows(); i++) {
                truth_poses.row(i) = truth_cam_poses.row(i * 10).segment(1, 3);
                if (true) {
                    Eigen::Quaterniond pose_quat(truth_cam_poses(i * 10, 7), truth_cam_poses(i * 10, 4),
                                                 truth_cam_poses(i * 10, 5), truth_cam_poses(i * 10, 6));
                    Eigen::Matrix4d pose_to_init;
                    pose_to_init.setIdentity();
                    pose_to_init.block(0, 0, 3, 3) = pose_quat.toRotationMatrix();
                    pose_to_init.col(3).head<3>() = Eigen::Vector3d(truth_cam_poses(i * 10, 1),
                                                                    truth_cam_poses(i * 10, 2),
                                                                    truth_cam_poses(i * 10, 3));
                    Eigen::Matrix4d pose_to_ground = pose_to_init;
                    truth_poses.row(i) = pose_to_ground.col(3).head<3>();
                }
            }
            cout << "Read sampled truth pose size for visualization:  " << truth_poses.rows() << endl;
        }

    }

    void MapDrawer::DrawMapPoints() {
        const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = mpMap->GetReferenceMapPoints();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if (vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 0.0);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

        }

        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

        if (bDrawKF) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f, 0.0f, 1.0f);
                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }

        if (bDrawGraph) {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            for (size_t i = 0; i < vpKFs.size(); i++) {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if (!vCovKFs.empty()) {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++) {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent) {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                }
            }

            glEnd();
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }


    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) {
//        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
        if (!mCameraPose.empty()) {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
//                unique_lock<mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        } else
            M.SetIdentity();
    }

    void MapDrawer::DrawTruth_poses() {
        glLineWidth(2);
        glBegin(GL_LINE_STRIP); // line strip connects adjacent points
        glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
        for (int pt_id = 0; pt_id < truth_poses.rows(); pt_id++) {
            glVertex3f(truth_poses(pt_id, 0), truth_poses(pt_id, 1), truth_poses(pt_id, 2));
        }
        glEnd();
    }


} //namespace ORB_SLAM
