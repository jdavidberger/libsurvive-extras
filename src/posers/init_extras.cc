#include "init_extras.h"
#include "opencv_solver.h"
#include "sba_solver.h"

#include <libsurvive/survive.h>

void init_extras() {
	RegisterDriver("PoserOpenCV", (void *)opencv_solver_poser_cb);
	RegisterDriver("PoserSBA", (void *)sba_solver_poser_cb);
}
