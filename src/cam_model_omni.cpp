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
#include "cam_model_omni.h"
// own includes
#include "misc.h"

namespace MultiColSLAM
{
	using namespace cv;
	using namespace std;

	void cCamModelGeneral_::ImgToWorld(cv::Point3_<double>& X,						// 3D scene point
		const cv::Point_<double>& m) 			            // 2D image point
	{
		//double invAff = c - d*e;
		const double u_t = m.x - u0;
		const double v_t = m.y - v0;
		// inverse affine matrix image to sensor plane conversion
		X.x = (1 * u_t - d * v_t) / this->invAffine;
		X.y = (-e * u_t + c * v_t) / this->invAffine;
		const double X2 = X.x * X.x;
		const double Y2 = X.y * X.y;
		X.z = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

		// normalize vectors spherically
		const double norm = sqrt(X2 + Y2 + X.z*X.z);
		X.x /= norm;
		X.y /= norm;
		X.z /= norm;
	}


// static void Cam2WorldCoordinate(Point2D32f src, Point2D32f* dst, calibSVFisheyeParam *params)
// {
// 	Point2D32f center = {params->center_x, params->center_y};
// 	double c = params->c;
// 	double src_x = (src.x-center.x)/c;
// 	double src_y = src.y-center.y;
// 	double r = sqrt(src_x*src_x+src_y*src_y);
// 	double zp = params->pol[0];
// 	double r_i = 1;
// 	int i;

// 	for (i = 1; i < params->pol_length; i++)
// 	{
// 		r_i *= r;
// 		zp += r_i*params->pol[i];
// 	}

// 	dst->x = (float)(src_x/zp*(-1));
// 	dst->y = (float)(src_y/zp*(-1));
// }

// static void Vec2CamCoordinate(Point2D32f vecPt, Point2D32f* camPt, calibAdasFisheyeParam *param)
// {
// 	float x_ = vecPt.x;
// 	float y_ = vecPt.y;
// 	float cx_ = param->cx;
// 	float cy_ = param->cy;
// 	float r0 = sqrtf((x_*x_) + (y_*y_));
// 	float t = atanf(r0);
// 	float r =
// 		(param->k1 * powf(t, 1.f)) +
// 		(param->k3 * powf(t, 3.f)) +
// 		(param->k5 * powf(t, 5.f)) +
// 		(param->k7 * powf(t, 7.f)) +
// 		(param->k9 * powf(t, 9.f));
// 	float phi = 0.f;
	
// 	if ((fabs(x_)<FLT_MIN*100) && (fabs(y_)<FLT_MIN*100))
// 	{
// 		camPt->x = cx_;
// 		camPt->y = cy_;
// 	}
// 	else
// 	{
// 		phi = (float)atan2(y_, x_);
// 		camPt->x = ((r * (float)cos(phi)) / param->asp) + cx_;
// 		camPt->y = (r * (float)sin(phi)) + cy_;
// 	}
// }


	void cCamModelGeneral_::ImgToWorld(double& x, double& y, double& z,						// 3D scene point
		const double& u, const double& v) 			    // 2D image point
	{
		//double invAff = c - d*e;
		const double u_t = u - u0;
		const double v_t = v - v0;
		// inverse affine matrix image to sensor plane conversion
		x = (u_t - d * v_t) / this->invAffine;
		y = (-e * u_t + c * v_t) / this->invAffine;
		const double X2 = x * x;
		const double Y2 = y * y;
		z = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

		// normalize vectors spherically
		double norm = sqrt(X2 + Y2 + z*z);
		x /= norm;
		y /= norm;
		z /= norm;
	}

	void cCamModelGeneral_::ImgToWorld(cv::Vec3d& X,						// 3D scene point
		const cv::Vec2d& m) 			            // 2D image point
	{
		//double invAff = c - d*e;
		const double u_t = m(0) - u0;
		const double v_t = m(1) - v0;
		// inverse affine matrix image to sensor plane conversion
		X(0) = (u_t - d * v_t) / this->invAffine;
		X(1) = (-e * u_t + c * v_t) / this->invAffine;
		const double X2 = X(0) * X(0);
		const double Y2 = X(1) * X(1);
		X(2) = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

		// normalize vectors spherically
		double norm = sqrt(X2 + Y2 + X(2)*X(2));
		X(0) /= norm;
		X(1) /= norm;
		X(2) /= norm;
	}


	void cCamModelGeneral_::WorldToImg(const cv::Point3_<double>& X,			// 3D scene point
		cv::Point_<double>& m)			// 2D image point
	{
		double norm = sqrt(X.x*X.x + X.y*X.y);

		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-X.z / norm);
		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = X.x / norm * rho;
		const double vv = X.y / norm * rho;

		m.x = uu*c + vv*d + u0;
		m.y = uu*e + vv + v0;
	}

	void cCamModelGeneral_::WorldToImg(const cv::Vec3d& X,			// 3D scene point
		cv::Vec2d& m)			// 2D image point
	{

		double norm = cv::sqrt(X(0)*X(0) + X(1)*X(1));

		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-X(2) / norm);
		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = X(0) / norm * rho;
		const double vv = X(1) / norm * rho;

		m(0) = uu*c + vv*d + u0;
		m(1) = uu*e + vv + v0;
	}

	void cCamModelGeneral_::WorldToImg(const cv::Vec3d& X,			// 3D scene point
		cv::Vec2f& m)			// 2D image point
	{
		double norm = cv::sqrt(X(0)*X(0) + X(1)*X(1));

		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-X(2) / norm);

		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = X(0) / norm * rho;
		const double vv = X(1) / norm * rho;

		m(0) = uu*c + vv*d + u0;
		m(1) = uu*e + vv + v0;
	}

	void cCamModelGeneral_::WorldToImg(const double& x, const double& y, const double& z,    // 3D scene point
		double& u, double& v) const							 // 2D image point
	{
		double norm = sqrt(x*x + y*y);
		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-z / norm);
		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = x / norm * rho;
		const double vv = y / norm * rho;

		u = uu*c + vv*d + u0;
		v = uu*e + vv + v0;
	}



// /**
// 原始图像坐标到畸变校正图坐标
// */
// static void Cam2VecCoordinate(Point2D32f camPt, Point2D32f* vecPt, calibAdasFisheyeParam *param)
// {
// 	float cx_ = param->cx;
// 	float cy_ = param->cy;
// 	float rx_ = (camPt.x - cx_)*param->asp;
// 	float ry_ = camPt.y - cy_;
// 	float r = sqrtf((rx_*rx_) + (ry_*ry_));
// 	float t = Fastfsolve(r, param);
// 	float r0 = tanf(t) * 1;
// 	float phi = 0.f;
// 	if ((fabs(rx_)<FLT_MIN*100) && (fabs(ry_)<FLT_MIN*100))
// 	{
// 		vecPt->x = 0.f;
// 		vecPt->y = 0.f;
// 	}
// 	else
// 	{
// 		phi = (float)atan2(ry_, rx_);
// 		vecPt->x = r0 * (float)cos(phi);
// 		vecPt->y = r0 * (float)sin(phi);
// 	}
// 	vecPt->z = 1.f;
// }

	bool cCamModelGeneral_::isPointInMirrorMask(
		const double& u,
		const double& v,
		int pyr)
	{
		const int ur = cvRound(u);
		const int vr = cvRound(v);
		// check image bounds
		if (ur >= mirrorMasks[pyr].cols || ur <= 0 ||
			vr >= mirrorMasks[pyr].rows || vr <= 0)
			return false;
		// check mirror
		if (mirrorMasks[pyr].ptr<uchar>(vr)[ur] > 0)
			return true;
		else return false;
	}


	void CreateMirrorMask(cCamModelGeneral_ camera,
		int c,
		int pyrLevel,
		vector<Mat>& mirror_masks)
	{
		int w = (int)camera.GetWidth();
		int h = (int)camera.GetHeight();
		float u0 = (float)camera.Get_v0();
		float v0 = (float)camera.Get_u0();
		Mat sizeDef = Mat::zeros(h, w, CV_8UC1);
		vector<Mat> sizeDefvec;
		buildPyramid(sizeDef, sizeDefvec, pyrLevel);
		// Mirror mask for pyramid
		//float offset[4] = { 22.0f, 10.0f, 5.0f, 1.0f };
		float offset[4] = { 50.0f, 25.0f, 10.0f, 5.0f };
		//float offset[4] = { 50.0f, 50.0f, 50.0f, 50.0f};
		for (int mIdx = 0; mIdx < pyrLevel; mIdx++)
		{
			if (mIdx != 0)
			{
				w = sizeDefvec[mIdx].cols;
				h = sizeDefvec[mIdx].rows;

				u0 = ceil(u0 / 2.0f);
				v0 = ceil(v0 / 2.0f);
			}
			Mat tempMask = Mat::zeros(h, w, CV_8UC1);

			// for (int i = 0; i < h; ++i)
			// {
			// 	for (int j = 0; j < w; ++j)
			// 	{
			// 		float ans = sqrt((float)pow(i - u0, 2) + (float)pow(j - v0, 2));
			// 		if (ans < (u0 + offset[mIdx]))
			// 			tempMask.at<uchar>(i, j) = 255;
			// 		else
			// 			tempMask.at<uchar>(i, j) = 0;
			// 	}

			// }

			//if (c == 2)
			// if(1)
			// {
			// 	float offset[4] = { 150.0f, 75.0f, 30.0f, 10.0f };
			// 	for (int i = 0; i < h; ++i)
			// 	{
			// 		for (int j = 0; j < w; ++j)
			// 		{
			// 			float ans = sqrt((float)pow(i - u0, 2) + (float)pow(j - v0, 2));
			// 			if (ans < (u0 + offset[mIdx]) && ((i < h/3*2) && (i > h/6)) )
			// 			{
			// 				tempMask.at<uchar>(i, j) = 255;
			// 			}
			// 			else
			// 			{
			// 				tempMask.at<uchar>(i, j) = 0;
			// 			}					
			// 		}
			// 	}
			// }
			// else
			// {


				//float offset[4] = { 150.0f, 75.0f, 30.0f, 10.0f };
				for (int i = 0; i < h; ++i)
				{
					for (int j = 0; j < w; ++j)
					{
						float ans = sqrt((float)pow(i - u0, 2) + (float)pow(j - v0, 2));

						if (ans < (u0 + offset[mIdx]) && (i < h/2))
						{
							//cout<< "ans:"<< ans << " u0:"<< u0 << " i:"<< i << " j:"<< j <<endl;
							tempMask.at<uchar>(i, j) = 255;
						}
						else
						{
							tempMask.at<uchar>(i, j) = 0;
						}					
					}
				}



				// for (int i = 0; i < h; ++i)
				// {
				// 	for (int j = 0; j < w; ++j)
				// 	{
				// 		float ans = sqrt((float)pow(i - u0, 2) + (float)pow(j - v0, 2));
				// 		if ((i < h/2))
				// 		{
				// 			tempMask.at<uchar>(i, j) = 255;
				// 		}
				// 		else
				// 		{
				// 			tempMask.at<uchar>(i, j) = 0;
				// 		}					
				// 	}
				// }


			//}
			

			// for (int i = 0; i < h; ++i)
			// {
			// 	for (int j = 0; j < w; ++j)
			// 	{
			// 		if (i > h/2)
			// 		{
			// 			tempMask.at<uchar>(i, j) = 0;
			// 		}
			// 		else
			// 		{
			// 			tempMask.at<uchar>(i, j) = 255;
			// 		}
			// 	}

			// }



			// cv::imshow("Mask", tempMask);

			// cv::waitKey(10000000000);
			mirror_masks.push_back(tempMask);
		}
	}

}