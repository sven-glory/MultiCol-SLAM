/**
* This file is part of MultiCol-SLAM
*
* Copyright (C) 2015-2016 Steffen Urban <urbste at googlemail.com>
* For more information see <https://github.com/urbste/MultiCol-SLAM>
*
* MultiCol-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* MultiCol-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with MultiCol-SLAM . If not, see <http://www.gnu.org/licenses/>.
*/

/*
* MultiCol-SLAM is based on ORB-SLAM2 which was also released under GPLv3
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
*/

#include "cViewer.h"
#ifndef _DEBUG
#include <pangolin/pangolin.h>
#endif
#include <mutex>

namespace MultiColSLAM
{

    cViewer::cViewer(
        cSystem* pSystem,
        cMultiFramePublisher* pFrameDrawer,
        cMapPublisher* pMapDrawer,
        cTracking *pTracking,
        const std::string &strSettingPath) :
        mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
        mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 25;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if (mImageWidth < 1 || mImageHeight < 1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];

        drawNrCams = (int)fSettings["Viewer.DrawNrCams"];

        int nrCamsTracker = mpTracker->GetNrCams();
        if (drawNrCams > nrCamsTracker)
            drawNrCams = nrCamsTracker;
        showWidth = 1280.0;
        showHeight = 720.0;
        // showWidth = 754.0;
        // showHeight = 480.0;



    }

    void cViewer::Run()
    {

        mbFinished = false;
#ifndef _DEBUG
        pangolin::CreateWindowAndBind("FusionMapping: Map Viewer", 1024, 768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        int iBntPix = 176;
        pangolin::CreatePanel("menu").SetBounds(0.35, 1.0, 0.0, pangolin::Attach::Pix(iBntPix));
        // pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175)); //[Debug]-->delete
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        // pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true); //[Debug]-->delete
        pangolin::Var<bool> menuShowObstacle("menu.Show Obstacle", false, true);
        pangolin::Var<bool> menuShowUS("menu.Show US", false, true); //show ultrasonic waves
        pangolin::Var<bool> menuShow3DMap("menu.Obstacle 3D", true, true);
        pangolin::Var<int>  menuMapScale("menu.Map scale", 1, 1, 5); //int slider
        pangolin::Var<bool> menuShowCmpKFM("menu.Show CmpKeyFrames", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            // pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0) //[Debug]-->delete
            pangolin::ModelViewLookAt(-5.0, -5.0, 5.0, 0, 0, 0, 0.0, 0.0, 1.0) //[Debug]add
            );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::View& cv_img_1 = pangolin::Display("image_1").SetBounds(0,0.35,0.0,0.4,showWidth/showHeight).SetLock(pangolin::LockLeft, pangolin::LockBottom);
        pangolin::View& cv_img_2 = pangolin::Display("image_2").SetBounds(0,0.3,0.4,0.6,showWidth/showHeight).SetLock(pangolin::LockLeft, pangolin::LockBottom);
        pangolin::View& cv_img_3 = pangolin::Display("image_3").SetBounds(0,0.3,0.6,0.8,showWidth/showHeight).SetLock(pangolin::LockLeft, pangolin::LockBottom);
        pangolin::View& cv_img_4 = pangolin::Display("image_4").SetBounds(0,0.3,0.8,1.0,showWidth/showHeight).SetLock(pangolin::LockLeft, pangolin::LockBottom);
        pangolin::GlTexture imgTexture1(showWidth, showHeight, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
        pangolin::GlTexture imgTexture2(showWidth, showHeight, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
        pangolin::GlTexture imgTexture3(showWidth, showHeight, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
        pangolin::GlTexture imgTexture4(showWidth, showHeight, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();
#endif
        // int x[] ={1200,1200,1200};
        // int y[] ={0,400,800};
        // for (int c = 0; c < drawNrCams; ++c)
        // {
        //     cv::namedWindow("MultiCol-SLAM: Current Frame: "+to_string(c));
        //     cv::moveWindow("MultiCol-SLAM: Current Frame: "+to_string(c), x[c], y[c]);
        // }


        bool bFollow = true;
        bool bLocalizationMode = false;

        while (1)
        {
#ifndef _DEBUG
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLMCSPose(Twc);

            if (menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if (menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            mpMapDrawer->PublishCurrentCamera(Twc);
            if (menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->PublishMultiKeyFrames(menuShowKeyFrames, menuShowGraph);
            if (menuShowPoints)
                mpMapDrawer->PublishMapPoints();
            if (menuShowCmpKFM)
                mpMapDrawer->PublishCompareKeyFrames();

            // draw obstcale map
            // d_cam2.Activate(s_cam2);
            // glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            if (menuShowObstacle)
            {
                mpMapDrawer->PublishBuildMap(Twc, menuShow3DMap, menuMapScale, menuShowUS);
            }

            std::vector<cv::Mat> imgs;
            mpFrameDrawer->DrawMultiFrame(imgs);

            // 向GPU装载图像
            imgTexture1.Upload(imgs[0].data, GL_BGR, GL_UNSIGNED_BYTE);
            imgTexture2.Upload(imgs[1].data, GL_BGR, GL_UNSIGNED_BYTE);
            imgTexture3.Upload(imgs[2].data, GL_BGR, GL_UNSIGNED_BYTE);
            imgTexture4.Upload(imgs[3].data, GL_BGR, GL_UNSIGNED_BYTE);
            // 显示图像
            cv_img_1.Activate();
            glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
            imgTexture1.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的

            cv_img_2.Activate();
            glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
            imgTexture2.RenderToViewportFlipY();

            cv_img_3.Activate();
            glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
            imgTexture3.RenderToViewportFlipY();

            cv_img_4.Activate();
            glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
            imgTexture4.RenderToViewportFlipY();

            pangolin::FinishFrame();
#endif
            //std::vector<cv::Mat> imgs;
            // mpFrameDrawer->DrawMultiFrame(imgs);
            // for (int c = 0; c < drawNrCams; ++c)
            // {
            //     cv::imshow("MultiCol-SLAM: Current Frame: " + to_string(c), imgs[c]);
            //     //cv::moveWindow("trans:MultiCol-SLAM: Current Frame: " + to_string(c),0,0);
            //     cv::waitKey(1);
            // }
            // cv::waitKey(mT);
#ifndef _DEBUG
            if (menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                // menuLocalizationMode = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
                menuShowObstacle = false;
                menuShow3DMap = true;
                menuMapScale  = 1;
                menuShowCmpKFM = true;
                menuShowUS = false;
            }
#endif
            if (Stop())
                while (isStopped())
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            if (CheckFinish())
                break;
        }

        SetFinish();
#ifndef _DEBUG
        pangolin::BindToContext("MultiCam SLAM: Map Viewer");
        pangolin::Quit();
#endif
    }

    void cViewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool cViewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void cViewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool cViewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void cViewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool cViewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool cViewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void cViewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}