#include "opencv_solver.h"

#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <libsurvive/poser.h>
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

static SurvivePose solve_correspondence(SurviveObject *so, const std::vector<cv::Point3f> &cal_objectPoints,
										const std::vector<cv::Point2f> &cal_imagePoints, bool cameraToWorld) {

	// std::cerr << "Solving for " << cal_imagePoints.size() << " correspondents" << std::endl;
	if (cal_imagePoints.size() <= 4) {
		auto ctx = so->ctx;
		SV_INFO("Can't solve for only %lu points\n", cal_imagePoints.size());

		return {};
	}
	cv::Mat_<double> dist, rvec, tvec, R;

	static cv::Mat_<double> identity = cv::Mat_<double>::eye(3, 3);
	cv::solvePnP(cal_objectPoints, cal_imagePoints, identity, dist, rvec, tvec, false, CV_EPNP);
	/*
			std::vector<int> inliers;
			cv::solvePnPRansac(cal_objectPoints, cal_imagePoints, cameraMatrix, dist, rvec, tvec, false, 100, 1., .99,
	   inliers);
	 */
	cv::Rodrigues(rvec, R); // R is 3x3

	// Requested output is camera -> world, so invert
	if (cameraToWorld) {
		R = R.t();
		tvec = -R * tvec;
	}

	SurvivePose rtn = {};
	for (size_t i = 0; i < 3; i++) {
		rtn.Pos[i] = tvec[0][i];
	}

	FLT tmp[4];
	quatfrommatrix33(R, tmp);

	// Typical camera applications have Z facing forward; the vive is contrarian and has Z going out of the
	// back of the lighthouse. Think of this as a rotation on the Y axis a full 180 degrees -- the quat for that is
	// [0 0x 1y 0z]
	const FLT rt[4] = {0, 0, 1, 0};
	quatrotateabout(rtn.Rot, tmp, rt);
	if (!cameraToWorld) {
		// We have to pre-multiply the rt transform here, which means we have to also offset our position by
		quatrotateabout(rtn.Rot, rt, tmp);
		rtn.Pos[0] = -rtn.Pos[0];
		rtn.Pos[2] = -rtn.Pos[2];
	}

	return rtn;
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

		so->ctx->bsd[lh].Pose = solve_correspondence(so, cal_objectPoints, cal_imagePoints, true);

		so->ctx->bsd[lh].PositionSet = 1;
	}
	return 0;
}

struct PersistentScene {
	uint32_t tolerance = 1500000;

	// If "lengths[...]" < 0, means not a valid piece of sweep information.
	FLT angles[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2]; // 2 Axes  (Angles in LH space)
	uint32_t timecode[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2] = {};

	PoserDataIMU lastimu;

	uint32_t currentPoseTime;
	SurvivePose currentPose;

	void integratePose(const SurvivePose &newPose, uint32_t timecode_now) {
		if (timecode_now - currentPoseTime > tolerance) {
			currentPose = newPose;
		} else {
			for (int i = 0; i < 3; i++) {
				currentPose.Pos[i] = currentPose.Pos[i] * .9 + newPose.Pos[i] * .1;
			}

			FLT tmp[4];

			for (int i = 0; i < 4; i++) {
				tmp[i] = currentPose.Rot[i] * .9 + newPose.Rot[i] * .1;
			}

			quatnormalize(currentPose.Rot, tmp);
		}
	}

	void add(PoserDataLight *lightData) {
		int axis = (lightData->acode & 1);
		auto &data_timecode = timecode[lightData->sensor_id][lightData->lh][axis];
		auto &angle = angles[lightData->sensor_id][lightData->lh][axis];

		// if(lightData->timecode - data_timecode > 100000000) {
		angle = lightData->angle;
		/*} else {
			angle = lightData->angle * .1 + angle * .9;
		}*/
		data_timecode = lightData->timecode;
	}

	void fill_correspondence(SurviveObject *so, int lh, std::vector<cv::Point3_<float>> &cal_objectPoints,
							 std::vector<cv::Point_<float>> &cal_imagePoints, uint32_t timecode_now) {
		cal_objectPoints.clear();
		cal_imagePoints.clear();

		for (size_t i = 0; i < so->nr_locations; i++) {
			auto &data_timecode = timecode[i][lh];
			auto &pt = angles[i][lh];
			if (timecode_now - data_timecode[0] > tolerance || timecode_now - data_timecode[1] > tolerance)
				continue;

			cal_imagePoints.emplace_back(tan(pt[0]), tan(pt[1]));

			cal_objectPoints.emplace_back(so->sensor_locations[i * 3 + 0], so->sensor_locations[i * 3 + 1],
										  so->sensor_locations[i * 3 + 2]);
		}
	}
};

int opencv_solver_poser_cb(SurviveObject *so, PoserData *pd) {
	PersistentScene *scene = static_cast<PersistentScene *>(so->PoserData);
	if (!scene) {
		so->PoserData = scene = new PersistentScene();
	}

	switch (pd->pt) {
	case POSERDATA_IMU: {
		// Really should use this...
		auto imuData = (PoserDataIMU *)pd;
		return 0;
	}
	case POSERDATA_LIGHT: {
		auto lightData = (PoserDataLight *)pd;
		scene->add(lightData);
		std::vector<cv::Point3_<float>> cal_objectPoints;
		std::vector<cv::Point_<float>> cal_imagePoints;
		int lh = lightData->lh;
		scene->fill_correspondence(so, lh, cal_objectPoints, cal_imagePoints, lightData->timecode);
		if (cal_imagePoints.size() > 4 && so->ctx->bsd[lh].PositionSet && lh == 1 && (lightData->acode & 1) == 0) {
			auto pose = solve_correspondence(so, cal_objectPoints, cal_imagePoints, false);

			SurvivePose txPose = {};
			quatrotatevector(txPose.Pos, so->ctx->bsd[lh].Pose.Rot, pose.Pos);

			for (int i = 0; i < 3; i++) {
				txPose.Pos[i] += so->ctx->bsd[lh].Pose.Pos[i];
			}
			quatrotateabout(txPose.Rot, so->ctx->bsd[lh].Pose.Rot, pose.Rot);

			// scene->integratePose(txPose, lightData->timecode);
			// txPose = scene->currentPose;
			if (so->ctx->rawposeproc) {
				so->ctx->rawposeproc(so, lh, &txPose.Pos[0]);
			}
		}
		return 0;
	}
	case POSERDATA_FULL_SCENE: {
		return opencv_solver_fullscene(so, (PoserDataFullScene *)(pd));
	}
	}
	return -1;
}
