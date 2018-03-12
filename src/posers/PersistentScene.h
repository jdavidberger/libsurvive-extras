#pragma once
#include "PersistentScene.h"
#include "opencv_solver.h"
#include <vector>

#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <survive.h>

extern "C" {
#include "../redist/linmath.h"
};

struct PersistentScene {
	uint32_t tolerance = 1500000;

	// If "lengths[...]" < 0, means not a valid piece of sweep information.
	FLT angles[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2]; // 2 Axes  (Angles in LH space)
	uint32_t timecode[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2] = {};

	PoserDataIMU lastimu;

	uint32_t currentPoseTime;
	SurvivePose currentPose;

	void integratePose(const SurvivePose &newPose, uint32_t timecode_now);

	void add(SurviveObject *lightData, PoserDataLight *ptr);

	bool isStillValid(uint32_t timecode_now, size_t idx, int lh) const {
		auto &data_timecode = timecode[idx][lh];
		return !(timecode_now - data_timecode[0] > tolerance || timecode_now - data_timecode[1] > tolerance);
	}

	struct TwoSensorCorrespondence {
		size_t sensor_idx;
		cv::Point2d ang0, ang1;

		TwoSensorCorrespondence(size_t sensor_idx, const cv::Point2d &ang0, const cv::Point2d &ang1);
		TwoSensorCorrespondence(size_t sensor_idx, double x1, double y1, double x2, double y2)
			: sensor_idx(sensor_idx), ang0(x1, y1), ang1(x2, y2) {}
	};

	std::vector<TwoSensorCorrespondence> fill_correspondence(SurviveObject *so, uint32_t timecode_now);

	void fill_correspondence(SurviveObject *so, int lh, std::vector<cv::Point3_<float>> &cal_objectPoints,
							 std::vector<cv::Point_<float>> &cal_imagePoints, uint32_t timecode_now);
};

PersistentScene &GetSceneForSO(const SurviveObject &so);