#include "posers/init_extras.h"
#include "posers/sba_solver.h"
#include <survive.h>

// NOTE: For this tool to do what it needs to do, you need to manually edit your
// configuration
// with
// "ConfigPoser":"PoserSBA_SolveOptimalConfig"
// instead of some other poser
int main(int argc, char **argv) {
	init_extras();

	RegisterDriver("PoserSBA_SolveOptimalConfig",
				   (void *)sba_bruteforce_config_solver_cb);

	auto ctx = survive_init(0);

	survive_cal_install(ctx);
	// wait for CTRL-C or kill
	bool quit = false;
	while (survive_poll(ctx) == 0 && quit == false) {
	}
	survive_close(ctx);

	return 0;
}
