#include "opencv_solver.h"

#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <poser.h>
#include <survive.h>

extern "C" {
#include "../redist/linmath.h"
};

static void quatfrommatrix33(cv::Mat_<double> &m, double *q) {
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

			cal_imagePoints.emplace_back(tan(pt[0]), tan(pt[1]));

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

		static cv::Mat_<double> identity = cv::Mat_<double>::eye(3, 3);
		cv::solvePnP(cal_objectPoints, cal_imagePoints, identity, dist, rvec, tvec, false, CV_EPNP);
		/*
				std::vector<int> inliers;

							cv::solvePnPRansac(cal_objectPoints,
			   cal_imagePoints, cameraMatrix, dist, rvec, tvec,
											   false, 100, 1., .99, inliers);
			*/

		cv::Mat_<double> R;
		cv::Rodrigues(rvec, R); // R is 3x3

		R = R.t();
		tvec = -R * tvec;

		so->ctx->bsd[lh].PositionSet = 1;

		for (size_t i = 0; i < 3; i++) {
			so->ctx->bsd[lh].Pose.Pos[i] = tvec[0][i];
		}

		// Typical camera applications have Z facing forward; the vive is contrarian
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
