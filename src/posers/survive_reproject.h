#pragma once
#include "survive.h"
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" { 
#endif
  
  typedef struct {
  bool swap;
  bool invert[2];
  bool enable[2];
} survive_calibration_options_config; 

  
  typedef struct {
  
  survive_calibration_options_config tilt, phase, curve, gibMag, gibPhase;

  bool gibUseSin;
  
} survive_calibration_config;
  
  const survive_calibration_config* default_config();

  survive_calibration_config survive_calibration_config_create_from_idx(size_t v);  
  size_t survive_calibration_config_index(const survive_calibration_config* config);
  
  void survive_reproject(int lighthouse, FLT* point3d);
  void survive_reproject_from_pose(int lighthouse, const SurvivePose* pose, FLT* point3d);
  void survive_reproject_from_pose_with_config(const SurviveContext* ctx, const survive_calibration_config* config,
    int lighthouse, const SurvivePose* pose, const FLT* point3d, FLT* out);

#ifdef __cplusplus
}
#endif
  
