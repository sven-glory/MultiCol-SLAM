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
* Ra鷏 Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
*/

#include "cMap.h"

namespace MultiColSLAM
{
	cMap::cMap()
	{
		mbMapUpdated = false;
		mnMaxKFid = 0;
	}

	void cMap::AddKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspKeyFrames.insert(pKF);
		if (pKF->mnId > mnMaxKFid)
			mnMaxKFid = pKF->mnId;
		mbMapUpdated = true;
	}

	void cMap::AddMapPoint(cMapPoint *pMP)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspMapPoints.insert(pMP);
		mbMapUpdated = true;
	}

	void cMap::EraseMapPoint(cMapPoint *pMP)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspMapPoints.erase(pMP);
		mbMapUpdated = true;
	}

	void cMap::EraseKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspKeyFrames.erase(pKF);
		mbMapUpdated = true;
	}

	void cMap::SetReferenceMapPoints(const std::vector<cMapPoint*> &vpMPs)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mvpReferenceMapPoints = vpMPs;
		mbMapUpdated = true;
	}

	std::vector<cMultiKeyFrame*> cMap::GetAllKeyFrames()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return std::vector<cMultiKeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
	}

	std::vector<cMapPoint*> cMap::GetAllMapPoints()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return std::vector<cMapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
	}

	int cMap::MapPointsInMap()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mspMapPoints.size();
	}

	int cMap::KeyFramesInMap()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mspKeyFrames.size();
	}

	std::vector<cMapPoint*> cMap::GetReferenceMapPoints()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mvpReferenceMapPoints;
	}

	bool cMap::isMapUpdated()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mbMapUpdated;
	}

	void cMap::SetFlagAfterBA()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mbMapUpdated = true;
	}

	void cMap::ResetUpdated()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mbMapUpdated = false;
	}

	unsigned int cMap::GetMaxKFid()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mnMaxKFid;
	}

	void cMap::clear()
	{
		for (std::set<cMapPoint*>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end();
			sit != send; ++sit)
			delete *sit;

		for (std::set<cMultiKeyFrame*>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end();
			sit != send; ++sit)
			delete *sit;

		mspMapPoints.clear();
		mspKeyFrames.clear();
		mnMaxKFid = 0;
		mvpReferenceMapPoints.clear();
	}

    void cMap::SaveMapPoint(ofstream& f, cMapPoint *mp)
    {
        //Save ID and the x,y,z coordinates of the current MapPoint
        f.write((char*)&mp->mnId, sizeof(mp->mnId));
        cv::Vec3d mpWorldPos = mp->GetWorldPos();
        f.write((char*)& mpWorldPos[0], sizeof(float));
        f.write((char*)& mpWorldPos[1], sizeof(float));
        f.write((char*)& mpWorldPos[2], sizeof(float));
    }

    void cMap::SaveKeyFrame(ofstream &f, cMultiKeyFrame *kf)
    {
        //保存当前关键帧的ID和时间戳
        f.write((char*)&kf->mnId, sizeof(kf->mnId));

        f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));
        //保存当前关键帧的位姿
        cv::Matx<double, 4, 4> Tcw = kf->GetPose();
        cv::Matx33d Rcw = cv::Matx<double, 3, 3>(Tcw(0, 0), Tcw(0, 1), Tcw(0, 2),
                                                 Tcw(1, 0), Tcw(1, 1), Tcw(1, 2),
                                                 Tcw(2, 0), Tcw(2, 1), Tcw(2, 2));
        std::vector<double> Quat = cConverter::toQuaternion(Rcw);
           for(int i = 0; i < 4; i ++)
               f.write((char*)&Quat[i], sizeof(double));
           for(int i = 0; i < 3; i++)
               f.write((char*)&Tcw(i, 3), sizeof(double));

           //保存ORB特征
           vector<cv::KeyPoint> vkp = kf->GetKeyPoints();
           int n = vkp.size();
           f.write((char*)&n, sizeof(vkp.size()));
           for(int i = 0; i < vkp.size(); i++)
           {
               cv::KeyPoint kp = vkp[i];
               f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
               f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
               f.write((char*)&kp.size, sizeof(kp.size));
               f.write((char*)&kp.angle, sizeof(kp.angle));
               f.write((char*)&kp.response, sizeof(kp.response));
               f.write((char*)&kp.octave, sizeof(kp.octave));

               vector<cv::Mat> vdes = kf->GetAllDescriptors();
               //f.write((char*)&vdes[i].cols, sizeof(vdes[i],cols));
               //for(int j = 0; j < vdes[i],cols; j++)
                  // f.write((char*)&vdes[i].at<j>)

           }
    }
}
