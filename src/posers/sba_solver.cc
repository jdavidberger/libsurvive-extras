#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <libsurvive/survive.h>
#include <vector>
#include <libsurvive/poser.h>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include "sba_solver.h"
#include "sba.h"
#include "opencv_solver.h"
#include "../redist/linmath.h"

PoserDataFullScene * pdfs;

cv::Mat_<double> matFromSurvivePose(const SurvivePose& pose) {
    auto qx = pose.Rot[0], qy = pose.Rot[1], qz = pose.Rot[2], qw = pose.Rot[3];
    auto qx2 = qx * qx, qy2 = qy * qy, qz2 = qz*qz, qw2 = qw*qw;

    return (cv::Mat_<double>(4, 4)) <<
                                    1 - 2*qy2 - 2*qz2,	2*qx*qy - 2*qz*qw,	2*qx*qz + 2*qy*qw, pose.Pos[0],
            2*qx*qy + 2*qz*qw,	1 - 2*qx2 - 2*qz2,	2*qy*qz - 2*qx*qw, pose.Pos[1],
            2*qx*qz - 2*qy*qw,	2*qy*qz + 2*qx*qw,	1 - 2*qx2 - 2*qy2, pose.Pos[2],
    0, 0, 0, 1;
}


void quatnormalize(FLT* out, const FLT* qin) {
    auto mag = sqrt(qin[0]*qin[0] + qin[1]*qin[1] + qin[3]*qin[3] + qin[2]*qin[2]);
    out[0] = qin[0] / mag;
    out[1] = qin[1] / mag;
    out[2] = qin[2] / mag;
    out[3] = qin[3] / mag;
}
void quattomatrix(FLT * matrix44, const FLT * qin)
{
    FLT q[4];
    quatnormalize(q, qin);

    //Reduced calulation for speed
    FLT xx = 2 * q[1] * q[1];
    FLT xy = 2 * q[1] * q[2];
    FLT xz = 2 * q[1] * q[3];
    FLT xw = 2 * q[1] * q[0];

    FLT yy = 2 * q[2] * q[2];
    FLT yz = 2 * q[2] * q[3];
    FLT yw = 2 * q[2] * q[0];

    FLT zz = 2 * q[3] * q[3];
    FLT zw = 2 * q[3] * q[0];

    //opengl major
    matrix44[0] = 1 - yy - zz;
    matrix44[1] = xy - zw;
    matrix44[2] = xz + yw;
    matrix44[3] = 0;

    matrix44[4] = xy + zw;
    matrix44[5] = 1 - xx - zz;
    matrix44[6] = yz - xw;
    matrix44[7] = 0;

    matrix44[8] = xz - yw;
    matrix44[9] = yz + xw;
    matrix44[10] = 1 - xx - yy;
    matrix44[11] = 0;

    matrix44[12] = 0;
    matrix44[13] = 0;
    matrix44[14] = 0;
    matrix44[15] = 1;
}



cv::Mat_<double> rotMatFromPose(const SurvivePose& pose) {
    auto qx = pose.Rot[0], qy = pose.Rot[1], qz = pose.Rot[2], qw = pose.Rot[3];
    auto qx2 = qx * qx, qy2 = qy * qy, qz2 = qz*qz, qw2 = qw*qw;

    cv::Mat_<double> test(4, 4);
    quattomatrix(reinterpret_cast<double *>(test.data), pose.Rot);

    return test.rowRange(0, 3).colRange(0, 3);
}


struct ConfigOptions {
  struct pairOptions {
        bool swap = {};
        bool invert[2] = {};
        bool enable[2] = {};
      
        cv::Vec2d operator()(const double* values) const {
            return cv::Vec2d(enable[0] * (invert[0] ? -1 : 1) * values[swap],
                             enable[1] * (invert[1] ? -1 : 1) * values[!swap] );
        }

      void normalize() {
	if(!enable[0])
	  invert[0] = false;
	if(!enable[1])
	  invert[1] = false;
	if(!enable[0] && !enable[1])
	  swap = false;
      }
    };


    pairOptions tilt, phase, curve, gibMag, gibPhase;

    bool gibUseSin = false;

    ConfigOptions(size_t v = 0) {
        bool* _this = (bool*)this;
	size_t ov = v;
        for(size_t i = 0;i < sizeof(ConfigOptions);i++) {
            _this[i] = static_cast<bool>(v & 1);
            v = v >> 1;
        }
	
	tilt.normalize();
	phase.normalize();
	curve.normalize();
	gibMag.normalize();
	gibPhase.enable[0] = gibMag.enable[0];
	gibPhase.enable[1] = gibMag.enable[1];	
	gibPhase.normalize();

	if(!gibPhase.enable[0] && !gibPhase.enable[1])
	  gibUseSin = false;
    }

  size_t index() const {
        bool* _this = (bool*)this;
	size_t v = 0;
        for(size_t i = 0;i < sizeof(ConfigOptions);i++) {
	  v = (v | _this[sizeof(ConfigOptions) - i - 1]);
	  v = v << 1; 
        }
	    v = v >> 1;
	return v;
  }

};

std::ostream& operator<<(std::ostream& o, const ConfigOptions::pairOptions& self)  {
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

std::ostream& operator<<(std::ostream& o, const ConfigOptions& self) {
  o << "Index: " << self.index() << std::endl;
  o << "Phase: " << std::endl << self.phase << std::endl;  
  o << "Tilt: " << std::endl << self.tilt << std::endl;
  o << "Curve: " << std::endl << self.curve << std::endl;
  o << "gibPhase: " << std::endl << self.gibPhase << std::endl;
  o << "gibMag: " << std::endl << self.gibMag << std::endl;
  o << "gibUseSin: " << self.gibUseSin << std::endl;
  return o;
}

ConfigOptions config(8673213);

struct CameraParams {
    SurvivePose pose;
    //
    //
  CameraParams() {}
  CameraParams(const SurvivePose& pose) : pose(pose) {}
  
    cv::Vec2d projectAsAng(const BaseStationData& bsd, double *pt) {
        auto R = rotMatFromPose(pose);

        R = R.t();  // rotation of inverse
        cv::Mat tvec = -R * cv::Mat(cv::Vec3d(pose.Pos[0], pose.Pos[1], pose.Pos[2])); // translation of inverse
        cv::Mat obj_inCam = R * cv::Mat(cv::Vec3d(pt[0], pt[1], pt[2])) + tvec;
        cv::Vec3d v = (cv::Mat)(obj_inCam.rowRange(0, 3));
        auto x = v[0] / v[2];
        auto y = v[1] / v[2];

        double ang_x = atan(x);
        double ang_y = atan(y);

        auto phase = config.phase(bsd.fcalphase);
        auto tilt = config.tilt(bsd.fcaltilt);
        auto curve = config.curve(bsd.fcalcurve);
        auto gibPhase = config.gibPhase(bsd.fcalgibpha);
        auto gibMag = config.gibPhase(bsd.fcalgibmag);
        auto gibf = [](double v) {
            return config.gibUseSin ? sin(v) : cos(v);
        };

        /*
        double tilt[2] = {  bsd.fcaltilt[1], -bsd.fcaltilt[0]  };
        double phase[2] = {bsd.fcalphase[0], -bsd.fcalphase[1]};//bsd.fcalphase;
        double curve[2] = { -bsd.fcalcurve[1], -bsd.fcalcurve[0]};
        double gib_sign[2] = {1, -1};
*/
        return { ang_x + phase[0] + tan(tilt[0]) * y + curve[0] * y * y + gibf( gibPhase[0] + ang_x) * gibMag[0],
                 ang_y + phase[1] + tan(tilt[1]) * x + curve[1] * x * x + gibf( gibPhase[1] + ang_y) * gibMag[1]};
    }
};


void metric_function(int j, int i, double *aj, double *xij, void *adata) {
    SurviveObject* so = static_cast<SurviveObject *>(adata);
    auto& camera = *(CameraParams*)aj;
    auto newPt = camera.projectAsAng(so->ctx->bsd[j], &so->sensor_locations[i * 3]);
    xij[0] = newPt[0];
    xij[1] = newPt[1];
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

/*
extern "C" sba_mot_levmar(const int n, const int m, const int mcon, char *vmask, double *p, const int cnp,
               double *x, double *covx, const int mnp,
               void (*proj)(int j, int i, double *aj, double *xij, void *adata),
               void (*projac)(int j, int i, double *aj, double *Aij, void *adata),
               void *adata, const int itmax, const int verbose, const double opts[SBA_OPTSSZ], double info[SBA_INFOSZ]);
*/
extern "C" FLT PoserCharlesSlow_err( SurviveObject * so, PoserData * pd );

static double run_sba(ConfigOptions options, PoserDataFullScene * pdfs, SurviveObject *so, int max_iterations = 50, double max_reproj_error = 0.005) {
  double *covx = nullptr;

  std::vector<char> vmask;
  std::vector<double> meas;
  construct_input(so, pdfs, vmask, meas);

  if(so->ctx->bsd[0].PositionSet == 0 ||
     so->ctx->bsd[1].PositionSet == 0) {
    opencv_solver_poser_cb(so, (PoserData*)pdfs);
  }
  
  double bestError = INFINITY;
  ConfigOptions bestConfigOptions(0);
  size_t bestIdx = 0;

  std::vector<CameraParams> camera_params;
  camera_params.emplace_back(so->ctx->bsd[0].Pose);
  camera_params.emplace_back(so->ctx->bsd[1].Pose);

  double opts[SBA_OPTSSZ] = {};
  double info[SBA_INFOSZ] = {};

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
		 sizeof(CameraParams) /
		 sizeof(double), // The number of floats that are in a camera param
		 meas.data(), // 2d points for 3d objs
		 covx, // covariance of measurement. Null sets to identity
		 2, // 2 points per image
		 metric_function,
		 nullptr, // jacobia of metric_func
		 so, // user data
		 max_iterations, // Max iterations
		 0, // verbosity
		 opts, // options
		 info); // info

  so->ctx->bsd[0].PositionSet = 1;
  so->ctx->bsd[1].PositionSet = 1;
  so->ctx->bsd[0].Pose = camera_params[0].pose;
  so->ctx->bsd[1].Pose = camera_params[1].pose;

  return info[1] / meas.size() * 2;
}

int sba_bruteforce_config_solver_cb(SurviveObject *so, PoserData *pd) {
  switch (pd->pt) {
  case POSERDATA_FULL_SCENE: {
    pdfs = (PoserDataFullScene *) (pd);

    double bestError = INFINITY;
    ConfigOptions bestConfigOptions;
    size_t bestIdx = 0;
    size_t total = 1 << sizeof(ConfigOptions);
    size_t unique_configs = 0;
    std::map<size_t, double> errorMap;
    
    for(size_t i = 0;i < total;i++) {
      if(i % 1000000 == 0) {
	std::cerr << ((double)i / total * 100) << "% complete" << std::endl;
      }
      
      config = ConfigOptions(i);
      if(i != config.index())
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
            pdfs = (PoserDataFullScene *) (pd);
	    std::cerr << "Running sba with " << config << std::endl;
	    auto error = run_sba(config, pdfs, so);
	    std::cerr << "Average reproj error: " << error << std::endl;
            return 0;
        }
    }
    return -1;
}
