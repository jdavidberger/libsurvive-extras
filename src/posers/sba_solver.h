#pragma once

#pragma once
#include <stdint.h>
#define FLT double
#include <libsurvive/poser.h>
#include <ostream>
#include "survive_reproject.h"

struct SurviveObject;

int sba_bruteforce_config_solver_cb(SurviveObject *so, PoserData *pd);
int sba_solver_poser_cb(SurviveObject* so, PoserData* pd);


std::ostream& operator<<(std::ostream& o, const   survive_calibration_options_config& self);
std::ostream& operator<<(std::ostream& o, const survive_calibration_config& self);