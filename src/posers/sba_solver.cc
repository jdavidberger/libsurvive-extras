#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include "sba_solver.h"
#include "opencv_solver.h"
#include "sba.h"
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "PersistentScene.h"
#include "libsurvive/poser.h"
#include <libsurvive/poser.h>
#include <survive.h>

extern "C" {
#include "../redist/linmath.h"
#include "../src/survive_config.h"
}

extern "C" int PoserCharlesSlow(SurviveObject *so, PoserData *pd);

std::ostream &operator<<(std::ostream &o,
						 const survive_calibration_options_config &self) {
	o << "\t";
	if (!self.enable[0] && !self.enable[1]) {
		o << "disabled";
		return o;
	}

	o << "swap: " << self.swap << std::endl;
	for (int i = 0; i < 2; i++) {
		if (self.enable[i]) {
			o << "\tinvert[" << i << "]: " << self.invert[i];
		} else {
			o << "\t" << i << ": disabled";
		}
	}
	return o;
}

std::ostream &operator<<(std::ostream &o,
						 const survive_calibration_config &self) {
	o << "Index: " << survive_calibration_config_index(&self) << std::endl;
	o << "Phase: " << std::endl << self.phase << std::endl;
	o << "Tilt: " << std::endl << self.tilt << std::endl;
	o << "Curve: " << std::endl << self.curve << std::endl;
	o << "gibPhase: " << std::endl << self.gibPhase << std::endl;
	o << "gibMag: " << std::endl << self.gibMag << std::endl;
	o << "gibUseSin: " << self.gibUseSin << std::endl;
	return o;
}

struct sba_context {
	survive_calibration_config calibration_config;
	PoserData *pdfs;
	SurviveObject *so;
};
void metric_function(int j, int i, double *aj, double *xij, void *adata) {
	sba_context *ctx = static_cast<sba_context *>(adata);
	auto so = ctx->so;
	auto &camera = *(SurvivePose *)aj;
	survive_reproject_from_pose_with_config(so->ctx, &ctx->calibration_config,
											j, &camera,
											&so->sensor_locations[i * 3], xij);
}

void construct_input(const SurviveObject *so, PoserDataFullScene *pdfs,
					 std::vector<char> &vmask, std::vector<double> &meas) {
	auto size = so->nr_locations * NUM_LIGHTHOUSES; // One set per lighthouse
	vmask.resize(size, 0);

	for (size_t sensor = 0; sensor < so->nr_locations; sensor++) {
		for (size_t lh = 0; lh < 2; lh++) {
			auto &l = pdfs->lengths[sensor][lh];
			if (l[0] < 0 || l[1] < 0)
				continue;

			auto &a = pdfs->angles[sensor][lh];
			vmask[sensor * NUM_LIGHTHOUSES + lh] = 1;
			meas.push_back((a[0]));
			meas.push_back((a[1]));
		}
	}
}
void construct_input(const SurviveObject *so, PoserDataLight *pdl, std::vector<char> &vmask,
					 std::vector<double> &meas) {
	auto size = so->nr_locations * NUM_LIGHTHOUSES; // One set per lighthouse
	vmask.resize(size, 0);

	auto &scene = GetSceneForSO(*so);

	for (size_t sensor = 0; sensor < so->nr_locations; sensor++) {
		for (size_t lh = 0; lh < 2; lh++) {
			if (scene.isStillValid(pdl->timecode, sensor, lh)) {
				auto &a = scene.angles[sensor][lh];
				vmask[sensor * NUM_LIGHTHOUSES + lh] = 1;
				meas.push_back((a[0]));
				meas.push_back((a[1]));
			}
		}
	}
}

void sba_set_cameras(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose, void *user) {
	SurvivePose *poses = (SurvivePose *)(user);
	poses[lighthouse] = *pose;
}
void sba_set_position(SurviveObject *so, uint8_t lighthouse, SurvivePose *new_pose, void *user) {
	auto *poses = (std::vector<SurvivePose> *)(user);
	poses->push_back(*new_pose);
}
extern "C" void *GetDriver(const char *name);

void str_metric_function(int j, int i, double *bi, double *xij, void *adata) {
	SurvivePose obj = *(SurvivePose *)bi;
	auto sensor_idx = j >> 1;
	auto lh = j & 1;

	sba_context *ctx = static_cast<sba_context *>(adata);
	auto so = ctx->so;

	assert(lh < 2);
	assert(sensor_idx < so->nr_locations);

	quatnormalize(obj.Rot, obj.Rot);
	FLT xyz[3];
	ApplyPoseToPoint(xyz, obj.Pos, &so->sensor_locations[sensor_idx * 3]);

	// std::cerr << "Processing " << sensor_idx << ", " << lh << std::endl;
	auto &camera = so->ctx->bsd[lh].Pose;
	survive_reproject_from_pose_with_config(so->ctx, &ctx->calibration_config, lh, &camera, xyz, xij);
}
static double run_sba_find_3d_structure(survive_calibration_config options, PoserDataLight *pdl, SurviveObject *so,
										int max_iterations = 50, double max_reproj_error = 0.005) {
	double *covx = nullptr;

	std::vector<char> vmask;
	std::vector<double> meas;
	construct_input(so, pdl, vmask, meas);

	if (so->ctx->bsd[0].PositionSet == 0 || so->ctx->bsd[1].PositionSet == 0) {
		return -1;
	}

	SurvivePose soLocation = so->OutPose;
	if (quatmagnitude(&soLocation.Rot[0]))
		soLocation.Rot[0] = 1;

	{
		auto subposer = config_read_str(so->ctx->global_config_values, "SBASeedPoser", "PoserOpenCV");
		auto driver = (PoserCB)GetDriver(subposer);
		auto ctx = so->ctx;
		if (driver) {
			PoserData hdr = pdl->hdr;
			pdl->hdr = {}; // Clear callback functions
			pdl->hdr.pt = hdr.pt;
			pdl->hdr.rawposeproc = sba_set_position;
			std::vector<SurvivePose> locations;
			pdl->hdr.userdata = &locations;
			driver(so, &pdl->hdr);
			pdl->hdr = hdr;

			if (locations.empty()) {
				return -1;
			} else if (false && locations.size() == 1) {
				PoserData_poser_raw_pose_func(&pdl->hdr, so, pdl->lh,
							      &locations[0]);
				return -1;
			} else {
				for (auto &p : locations) {
					for (int i = 0; i < 7; i++) {
						soLocation.Pos[i] += p.Pos[i];
					}
				}

				for (int i = 0; i < 7; i++) {
					soLocation.Pos[i] = soLocation.Pos[i] / locations.size();
				}
			}
		} else {
			SV_INFO("Not using a seed poser for SBA; results will likely be way off");
			for (int i = 0; i < 2; i++) {
				so->ctx->bsd[i].Pose = SurvivePose();
				so->ctx->bsd[i].Pose.Rot[0] = 1.;
			}
		}
		// opencv_solver_poser_cb(so, (PoserData *)pdl);
		// PoserCharlesSlow(so, (PoserData *)pdl);
	}

	double opts[SBA_OPTSSZ] = {};
	double info[SBA_INFOSZ] = {};

	sba_context ctx = {options, &pdl->hdr, so};

	opts[0] = SBA_INIT_MU;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH; // max_reproj_error * meas.size();
	opts[4] = 0.0;

	/*
	 * sba_str_levmar(const int n, const int ncon, const int m, char *vmask, double *p, const int pnp,
		   double *x, double *covx, const int mnp,
		   void (*proj)(int j, int i, double *bi, double *xij, void *adata),
		   void (*projac)(int j, int i, double *bi, double *Bij, void *adata),
		   void *adata, const int itmax, const int verbose, const double opts[SBA_OPTSSZ], double info[SBA_INFOSZ]);
	 */
	sba_str_levmar(1, // Number of 3d points
				   0, // Number of 3d points to fix in spot
				   NUM_LIGHTHOUSES * so->nr_locations, vmask.data(),
				   soLocation.Pos, // Reads as the full pose though
				   7,			   // pnp -- SurvivePose
				   meas.data(),
				   nullptr, // cov data
				   2,		// mnp -- 2 points per image
				   str_metric_function,
				   nullptr,		   // jacobia of metric_func
				   &ctx,		   // user data
				   max_iterations, // Max iterations
				   0,			   // verbosity
				   opts,		   // options
				   info);		   // info

	quatnormalize(soLocation.Rot, soLocation.Rot);
	PoserData_poser_raw_pose_func(&pdl->hdr, so, 1, &soLocation);

	// Docs say info[0] should be divided by meas; I don't buy it really...
	// std::cerr << info[0] / meas.size() * 2 << " original reproj error" << std::endl;

	return info[1] / meas.size() * 2;
}

static double run_sba(survive_calibration_config options,
					  PoserDataFullScene *pdfs, SurviveObject *so,
					  int max_iterations = 50,
					  double max_reproj_error = 0.005) {
	double *covx = nullptr;

	std::vector<char> vmask;
	std::vector<double> meas;
	construct_input(so, pdfs, vmask, meas);

	std::vector<SurvivePose> camera_params;
	camera_params.emplace_back(so->ctx->bsd[0].Pose);
	camera_params.emplace_back(so->ctx->bsd[1].Pose);

	if (true || so->ctx->bsd[0].PositionSet == 0 || so->ctx->bsd[1].PositionSet == 0) {
		auto subposer = config_read_str(so->ctx->global_config_values, "SBASeedPoser", "PoserOpenCV");
		auto driver = (PoserCB)GetDriver(subposer);
		auto ctx = so->ctx;
		if (driver) {
			SV_INFO("Using %s seed poser for SBA", subposer);
			PoserData hdr = pdfs->hdr;
			pdfs->hdr = {}; // Clear callback functions
			pdfs->hdr.pt = hdr.pt;
			pdfs->hdr.lighthouseposeproc = sba_set_cameras;
			pdfs->hdr.userdata = camera_params.data();
			driver(so, &pdfs->hdr);
			pdfs->hdr = hdr;
		} else {
			SV_INFO("Not using a seed poser for SBA; results will likely be way off");
			for (int i = 0; i < 2; i++) {
				so->ctx->bsd[i].Pose = SurvivePose();
				so->ctx->bsd[i].Pose.Rot[0] = 1.;
			}
		}
		// opencv_solver_poser_cb(so, (PoserData *)pdfs);
		// PoserCharlesSlow(so, (PoserData *)pdfs);
	}

	double opts[SBA_OPTSSZ] = {};
	double info[SBA_INFOSZ] = {};

	sba_context ctx = {options, &pdfs->hdr, so};

	opts[0] = SBA_INIT_MU;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH; // max_reproj_error * meas.size();
	opts[4] = 0.0;

	sba_mot_levmar(
		so->nr_locations,			 // number of 3d points
		NUM_LIGHTHOUSES,			 // Number of cameras -- 2 lighthouses
		0,							 // Number of cameras to not modify
		vmask.data(),				 // boolean vis mask
		(double *)&camera_params[0], // camera parameters
		sizeof(SurvivePose) /
			sizeof(double), // The number of floats that are in a camera param
		meas.data(),		// 2d points for 3d objs
		covx,				// covariance of measurement. Null sets to identity
		2,					// 2 points per image
		metric_function,
		nullptr,		// jacobia of metric_func
		&ctx,			// user data
		max_iterations, // Max iterations
		0,				// verbosity
		opts,			// options
		info);			// info

	PoserData_lighthouse_pose_func(&pdfs->hdr, so, 0, &camera_params[0]);
	PoserData_lighthouse_pose_func(&pdfs->hdr, so, 1, &camera_params[1]);

	// Docs say info[0] should be divided by meas; I don't buy it really...
	std::cerr << info[0] / meas.size() * 2 << " original reproj error" << std::endl;

	return info[1] / meas.size() * 2;
}

int sba_bruteforce_config_solver_cb(SurviveObject *so, PoserData *pd) {
	switch (pd->pt) {
	case POSERDATA_FULL_SCENE: {
		auto pdfs = (PoserDataFullScene *)(pd);

		double bestError = INFINITY;
		survive_calibration_config bestConfigOptions;
		size_t bestIdx = 0;
		size_t total = survive_calibration_config_max_idx() + 1;
		size_t unique_configs = 0;
		std::map<size_t, double> errorMap;

		for (size_t i = 0; i < total; i++) {
			if (i % 1000000 == 0) {
				std::cerr << ((double)i / total * 100) << "% complete"
						  << std::endl;
			}

			auto config = survive_calibration_config_create_from_idx(i);
			if (i != survive_calibration_config_index(&config))
				continue;
			unique_configs++;
			auto error = run_sba(config, pdfs, so, 5);
			errorMap[i] = error;
			if (error < bestError) {
				bestError = error;
				bestConfigOptions = config;
				bestIdx = i;
				std::cerr << i << " has " << bestError << std::endl;
			}
		}

		std::cerr << "Ran " << unique_configs << std::endl;
		std::cerr << "Best configuration setting is " << bestIdx << std::endl;
		std::cerr << bestConfigOptions << std::endl;

		FILE *f = fopen("error_map.csv", "w");
		for (auto &m : errorMap) {
			fprintf(f, "%lu, %.17g\n", m.first, m.second);
		}
		fclose(f);

		return 0;
	}
	}
	return -1;
}

int sba_solver_poser_cb(SurviveObject *so, PoserData *pd) {
	switch (pd->pt) {
	case POSERDATA_LIGHT: {
		auto &scene = GetSceneForSO(*so);
		auto lightData = (PoserDataLight *)pd;
		scene.add(so, lightData);

		auto config = *survive_calibration_default_config();
		auto error = run_sba_find_3d_structure(config, lightData, so);
		return 0;
	}
	case POSERDATA_FULL_SCENE: {
		auto pdfs = (PoserDataFullScene *)(pd);
		auto config = *survive_calibration_default_config();
		std::cerr << "Running sba with " << config << std::endl;
		auto error = run_sba(config, pdfs, so);
		std::cerr << "Average reproj error: " << error << std::endl;
		return 0;
	}
	}
	return -1;
}
