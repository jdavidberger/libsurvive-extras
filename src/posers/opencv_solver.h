#pragma once
#include <stdint.h>
#define FLT double
#include <libsurvive/poser.h>

struct SurviveObject;

int opencv_solver_poser_cb(SurviveObject* so, PoserData* pd);