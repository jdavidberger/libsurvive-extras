#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <poser.h>
#include "survive_reproject.h"
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include "sba_solver.h"
#include "sba.h"
#include "opencv_solver.h"
#include "../redist/linmath.h"

std::ostream& operator<<(std::ostream& o, const   survive_calibration_options_config& self)  {
  o << "\t";
  if(!self.enable[0] && !self.enable[1]) {
    o << "disabled";
    return o;
  }

  o << "swap: " << self.swap << std::endl;
  for(int i = 0;i < 2;i++) {
    if(self.enable[i]) {
      o << "\tinvert[" << i << "]: " << self.invert[i];
    } else {
      o << "\t" << i << ": disabled";
    }
  }
  return o;
}

std::ostream& operator<<(std::ostream& o, const survive_calibration_config& self) {
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
  PoserDataFullScene * pdfs;
  SurviveObject *so; 
};

void metric_function(int j, int i, double *aj, double *xij, void *adata) {
    sba_context* ctx = static_cast<sba_context *>(adata);
    auto so = ctx->so;
    auto& camera = *(SurvivePose*)aj;
    survive_reproject_from_pose_with_config(so->ctx, &ctx->calibration_config, j, &camera, &so->sensor_locations[i * 3], xij);
}

void construct_input(const SurviveObject* so, PoserDataFullScene *pdfs,
                     std::vector<char>& vmask,
                     std::vector<double>& meas) {
    auto size = so->nr_locations * NUM_LIGHTHOUSES; // One set per lighthouse
    vmask.resize(size, 0);

    for(size_t sensor = 0; sensor < so->nr_locations;sensor++) {
        for(size_t lh = 0;lh < 2;lh++) {
            auto& l = pdfs->lengths[sensor][lh];
            if(l[0] < 0 || l[1] < 0) continue;

            auto& a = pdfs->angles[sensor][lh];
            vmask[sensor * NUM_LIGHTHOUSES + lh] = 1;
            meas.push_back((a[0]));
            meas.push_back((a[1]));
        }
    }
}

static double run_sba(survive_calibration_config options, PoserDataFullScene * pdfs, SurviveObject *so, int max_iterations = 50, double max_reproj_error = 0.005) {
  double *covx = nullptr;

  std::vector<char> vmask;
  std::vector<double> meas;
  construct_input(so, pdfs, vmask, meas);

  if(so->ctx->bsd[0].PositionSet == 0 ||
     so->ctx->bsd[1].PositionSet == 0) {
    opencv_solver_poser_cb(so, (PoserData*)pdfs);
  }

  std::vector<SurvivePose> camera_params;
  camera_params.emplace_back(so->ctx->bsd[0].Pose);
  camera_params.emplace_back(so->ctx->bsd[1].Pose);

  double opts[SBA_OPTSSZ] = {};
  double info[SBA_INFOSZ] = {};

  sba_context ctx = {
    options, 
    pdfs,
    so
  };
  
  opts[0] = SBA_INIT_MU;
  opts[1] = SBA_STOP_THRESH;
  opts[2] = SBA_STOP_THRESH;
  opts[3] = SBA_STOP_THRESH;
  opts[3] = max_reproj_error * meas.size();
  opts[4] = 0.0;

  sba_mot_levmar(so->nr_locations, // number of 3d points
		 NUM_LIGHTHOUSES, // Number of cameras -- 2 lighthouses
		 0, // Number of cameras to not modify
		 vmask.data(), // boolean vis mask
		 (double *) &camera_params[0], // camera parameters
		 sizeof(SurvivePose) /
		 sizeof(double), // The number of floats that are in a camera param
		 meas.data(), // 2d points for 3d objs
		 covx, // covariance of measurement. Null sets to identity
		 2, // 2 points per image
		 metric_function,
		 nullptr, // jacobia of metric_func
		 &ctx, // user data
		 max_iterations, // Max iterations
		 0, // verbosity
		 opts, // options
		 info); // info

  so->ctx->bsd[0].PositionSet = 1;
  so->ctx->bsd[1].PositionSet = 1;
  so->ctx->bsd[0].Pose = camera_params[0];
  so->ctx->bsd[1].Pose = camera_params[1];

  return info[1] / meas.size() * 2;
}

int sba_bruteforce_config_solver_cb(SurviveObject *so, PoserData *pd) {
  switch (pd->pt) {
  case POSERDATA_FULL_SCENE: {
    auto pdfs = (PoserDataFullScene *) (pd);

    double bestError = INFINITY;
    survive_calibration_config bestConfigOptions;
    size_t bestIdx = 0;
    size_t total = 1 << sizeof(survive_calibration_config);
    size_t unique_configs = 0;
    std::map<size_t, double> errorMap;
    
    for(size_t i = 0;i < total;i++) {
      if(i % 1000000 == 0) {
	std::cerr << ((double)i / total * 100) << "% complete" << std::endl;
      }
      
      auto config = survive_calibration_config_create_from_idx(i);
      if(i != survive_calibration_config_index(&config))
	continue;
      unique_configs++;
      auto error = run_sba(config, pdfs, so, 5);
      errorMap[i] = error;
      if(error < bestError) {
	bestError = error;
	bestConfigOptions = config;
	bestIdx = i;
	std::cerr << i << " has " << bestError << std::endl;
      }
    }

    std::cerr << "Ran " << unique_configs << std::endl;
    std::cerr << "Best configuration setting is " << bestIdx << std::endl;
    std::cerr << bestConfigOptions << std::endl;

    FILE* f = fopen("error_map.csv", "w");
    for(auto& m : errorMap) {
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
        case POSERDATA_FULL_SCENE: {
            auto pdfs = (PoserDataFullScene *) (pd);
            auto config = *survive_calibration_default_config();
	    std::cerr << "Running sba with " << config << std::endl;
	    auto error = run_sba(config, pdfs, so);
	    std::cerr << "Average reproj error: " << error << std::endl;
            return 0;
        }
    }
    return -1;
}
