#pragma once

#pragma once
#include <stdint.h>
#define FLT double
#include <libsurvive/poser.h>

struct SurviveObject;

int sba_bruteforce_config_solver_cb(SurviveObject *so, PoserData *pd);
int sba_solver_poser_cb(SurviveObject* so, PoserData* pd);
