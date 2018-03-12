extern "C" {
#include "../redist/linmath.h"
}
#include "PersistentScene.h"
#include "opencv_solver.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <survive.h>

void PersistentScene::add(SurviveObject *so, PoserDataLight *lightData) {
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

void PersistentScene::fill_correspondence(SurviveObject *so, int lh, std::vector<cv::Point3_<float>> &cal_objectPoints,
										  std::vector<cv::Point_<float>> &cal_imagePoints, uint32_t timecode_now) {
	cal_objectPoints.clear();
	cal_imagePoints.clear();

	for (size_t i = 0; i < so->nr_locations; i++) {
		auto &data_timecode = timecode[i][lh];
		auto &pt = angles[i][lh];
		if (isStillValid(timecode_now, i, lh)) {
			cal_imagePoints.emplace_back(tan(pt[0]), tan(pt[1]));

			cal_objectPoints.emplace_back(so->sensor_locations[i * 3 + 0], so->sensor_locations[i * 3 + 1],
										  so->sensor_locations[i * 3 + 2]);
		}
	}
}

void PersistentScene::integratePose(const SurvivePose &newPose, uint32_t timecode_now) {
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

std::vector<PersistentScene::TwoSensorCorrespondence> PersistentScene::fill_correspondence(SurviveObject *so,
																						   uint32_t timecode_now) {
	std::vector<PersistentScene::TwoSensorCorrespondence> correspondence;
	for (size_t i = 0; i < so->nr_locations; i++) {

		if (isStillValid(timecode_now, i, 0) && isStillValid(timecode_now, i, 1)) {
			correspondence.emplace_back(i, angles[i][0][0], angles[i][0][1], angles[i][1][0], angles[i][1][1]);
		}
	}
	return correspondence;
}

PersistentScene &GetSceneForSO(const SurviveObject &so) {
	static std::map<const SurviveObject *, PersistentScene> scenes;
	return scenes[&so];
}

PersistentScene::TwoSensorCorrespondence::TwoSensorCorrespondence(size_t sensor_idx, const cv::Point2d &ang0,
																  const cv::Point2d &ang1)
	: sensor_idx(sensor_idx), ang0(ang0), ang1(ang1) {}
