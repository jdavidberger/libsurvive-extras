#include "opencv_solver.h"

#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <libsurvive/poser.h>
#include <libsurvive/survive.h>

extern "C" {
#include "../redist/linmath.h"
};

struct sensor_data_t {
	uint32_t timecode = 0;
	double angle = 0;
	double length = 0;

	sensor_data_t() {}

	sensor_data_t(uint32_t timecode, double length, double angle)
		: timecode(timecode), angle(angle), length(length) {}
};

// LH, sensor_id, acode

std::vector<std::vector<cv::Point3f>> listOfobjectPoints[2];
std::vector<std::vector<cv::Point2f>> listOfimagePoints[2];

double scale = 1.;
double fov = 120. / 180. * M_PI;
double fov_2 = fov / 2.;
size_t fakeImageSize = fov * scale;
double f = 1;  // scale * cos(fov_2) / sin(fov_2) * fov_2;
double cx = 0; // scale * fov_2,
double cy = cx;

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);

/*
void generateCameraMatrix(int lh) {
	cv::Mat dist, rvecs, tvecs;

	cv::calibrateCamera(listOfobjectPoints[lh], listOfimagePoints[lh],
cv::Size(fakeImageSize, fakeImageSize), cameraMatrix[lh], dist, rvecs, tvecs,
CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_ASPECT_RATIO |
CV_CALIB_FIX_PRINCIPAL_POINT);
	std::cerr << cameraMatrix[lh] << std::endl << std::endl;
	std::cerr << dist << std::endl << std::endl;
	std::cerr << rvecs<< std::endl << std::endl;
	std::cerr << tvecs<< std::endl << std::endl;

}
*/

void quatfrommatrix33(cv::Mat_<double> &m, double *q) {
	auto m00 = m(0, 0), m11 = m(1, 1), m22 = m(2, 2), m21 = m(2, 1), m12 = m(1, 2), m02 = m(0, 2), m20 = m(2, 0),
		 m10 = m(1, 0), m01 = m(0, 1);

	auto tr = m00 + m11 + m22;

	auto &qw = q[0];
	auto &qx = q[1];
	auto &qy = q[2];
	auto &qz = q[3];

	if (tr > 0) {
		auto S = sqrt(tr + 1.0) * 2; // S=4*qw
		qw = 0.25 * S;
		qx = (m21 - m12) / S;
		qy = (m02 - m20) / S;
		qz = (m10 - m01) / S;
	} else if ((m00 > m11) & (m00 > m22)) {
		auto S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
		qw = (m21 - m12) / S;
		qx = 0.25 * S;
		qy = (m01 + m10) / S;
		qz = (m02 + m20) / S;
	} else if (m11 > m22) {
		auto S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		qw = (m02 - m20) / S;
		qx = (m01 + m10) / S;
		qy = 0.25 * S;
		qz = (m12 + m21) / S;
	} else {
		auto S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		qw = (m10 - m01) / S;
		qx = (m02 + m20) / S;
		qy = (m12 + m21) / S;
		qz = 0.25 * S;
	}
}

static int opencv_solver_fullscene(SurviveObject *so, PoserDataFullScene *pdfs) {
	for (int lh = 0; lh < 2; lh++) {
		std::vector<cv::Point3f> cal_objectPoints;
		std::vector<cv::Point2f> cal_imagePoints;

		for (size_t i = 0; i < so->nr_locations; i++) {
			auto &lengths = pdfs->lengths[i][lh];
			auto &pt = pdfs->angles[i][lh];
			if (lengths[0] < 0 || lengths[1] < 0)
				continue;

			cal_imagePoints.emplace_back(tan(pt[0]) * scale, tan(pt[1]) * scale);

			cal_objectPoints.emplace_back(so->sensor_locations[i * 3 + 0], so->sensor_locations[i * 3 + 1],
										  so->sensor_locations[i * 3 + 2]);
		}

		std::cerr << "Solving for " << cal_imagePoints.size() << " correspondents" << std::endl;
		if (cal_imagePoints.size() <= 4) {
			auto ctx = so->ctx;
			SV_INFO("Can't solve for only %lu points on lh %d\n", cal_imagePoints.size(), lh);
			continue;
		}
		cv::Mat_<double> dist, rvec, tvec;
		std::vector<int> inliers;
		/*
						cv::solvePnPRansac(cal_objectPoints,
		   cal_imagePoints, cameraMatrix, dist, rvec, tvec,
										   false, 100, 1., .99, inliers);
		*/
		auto err = cv::solvePnP(cal_objectPoints, cal_imagePoints, cameraMatrix, dist, rvec, tvec, false, CV_EPNP);
		cv::Mat_<double> R;
		cv::Rodrigues(rvec, R); // R is 3x3

		for (size_t i = 0; i < cal_objectPoints.size(); i++) {
			auto &obj = cal_objectPoints[i];
			cv::Vec3d pt = {cal_imagePoints[i].x, cal_imagePoints[i].y, 1};

			cv::Vec3d v = cv::Mat(cameraMatrix * (R * cv::Mat_<double>(obj) + tvec));
			std::cerr << cv::norm(pt - (v / v[2])) << std::endl;
		}

		R = R.t();		  // rotation of inverse
		tvec = -R * tvec; // translation of inverse

		// Rotate to be vivian -- coordinates. This means towards the object is
		// backwards to Z
		/*R = R * (cv::Mat_<double>(3, 3) <<
							   -1, 0, 0,
							   0, 1, 0,
							   0, 0, -1);
*/
		cv::Vec3d pt = {0, 0, 1};

		so->ctx->bsd[lh].PositionSet = 1;
		so->ctx->bsd[lh].Pose.Pos[0] = tvec[0][0];
		so->ctx->bsd[lh].Pose.Pos[1] = tvec[0][1];
		so->ctx->bsd[lh].Pose.Pos[2] = tvec[0][2];

		FLT tmp[4];
		quatfrommatrix33(R, tmp);

		const FLT rt[4] = {0, 0, 1, 0};
		quatrotateabout(so->ctx->bsd[lh].Pose.Rot, tmp, rt);
	}

	return 0;
}

std::map<size_t, PoserDataLight> stored_data[NUM_LIGHTHOUSES];

void add_stored_data(PoserDataLight *lightData) {
	// stored_data[lightData->lh][lightData->sensor_id];
}

int opencv_solver_poser_cb(SurviveObject *so, PoserData *pd) {
	switch (pd->pt) {
	case POSERDATA_IMU: {
		// Really should use this...
		auto imuData = (PoserDataIMU *)pd;
		break;
	}
	case POSERDATA_LIGHT: {
		auto lightData = (PoserDataLight *)pd;
		add_stored_data(lightData);

		break;
	}
	case POSERDATA_FULL_SCENE: {
		return opencv_solver_fullscene(so, (PoserDataFullScene *)(pd));
	}
	}
	return -1;
}
