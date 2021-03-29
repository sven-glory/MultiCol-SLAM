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
* Ra�l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
*/

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include "cMap.h"
#include "cMapPoint.h"
#include "cMultiKeyFrame.h"
#include "cMultiFramePublisher.h"

#ifndef _DEBUG
#include <pangolin/pangolin.h>
#include <cQuadMap.h>
#endif

namespace MultiColSLAM
{
	class cMultiFramePublisher;

	class cMapPublisher
	{
	public:
		cMapPublisher(cMap* pMap,
			const string &strSettingPath);

		cMap* mpMap;

		void SetInitMC(const int Mc); // convert world coordinate for quadmap

		// for draw obstcale map
        MapFactory mpQuadMap;
		void PublishBuildMap(const pangolin::OpenGlMatrix& Tcw, bool Is3D, int scale, bool bShowUS);

		void setCmpKFM(const std::vector<cv::Matx44d>& cmpKFMs);

		void setUltrasonicData(const std::vector<cv::Vec3d>& USData, const std::vector<int>& DataTypes);

		void PublishMapPoints();
		void PublishMultiKeyFrames(const bool bDrawKF = true,
			const bool bDrawGraph = true);
		void PublishCompareKeyFrames();

#ifndef _DEBUG
		void PublishCurrentCamera(
			const pangolin::OpenGlMatrix& Tcw);

		void GetCurrentOpenGLMCSPose(
			pangolin::OpenGlMatrix& _Tcw);
#endif

		void SetCurrentCameraPose(const cv::Matx44d Tcw);
		void SetMCS(const std::vector<cv::Matx44d> M_c_s);

		void SetFinish();
		void RequestFinish();
		bool CheckFinish();
		bool isFinished();
		void RequestStop();
		bool isStopped();
		void Release();
		bool Stop();
	private:

		cv::Matx44d GetCurrentCameraPose();
		bool isCamUpdated();
		void ResetCamFlag();

		void DrawAxis(GLfloat width, GLfloat unit);
		void DrawCamera(float w, float h, float z);

		float mPointSize;
		float mCameraSize;
		float mCameraLineWidth;
		float mMultiKeyFrameSize;
		float mGraphLineWidth;
		float mMultiKeyFrameLineWidth;

		cv::Matx44d mCameraPose;
		std::vector<cv::Matx44d> mM_c_s;

		// for draw obstcale map
		int mleadingCam; // init camera No:0~3
		std::vector<cv::Vec3d> mRealWdPts; // World coordinate system: leading cam --> world
		std::vector<cv::Vec3d> mUSPts;
		std::vector<int> mUSPtTypes;
		std::vector<cv::Matx44d> mCmpKFMs; // History Key Frame

		bool mbCameraUpdated;
		bool modelPointsSet;
		int cnter;
		int laufSave;

		std::mutex mMutexCamera;
		std::mutex mMutexFinish;
		std::mutex mMutexStop;

		bool mbFinishRequested;
		bool mbFinished;
		bool mbStopped;
		bool mbStopRequested;
	};
}
#endif // MAPPUBLISHER_H
