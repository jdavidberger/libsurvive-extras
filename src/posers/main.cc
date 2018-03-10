#define FLT double

#include "init_extras.h"
#include <iostream>
#include <libsurvive/survive.h>

int main(int argc, char **argv) {
	init_extras();
	auto ctx = survive_init(0);
	survive_cal_install(ctx);

	// wait for CTRL-C or kill
	bool quit = false;
	while (survive_poll(ctx) == 0 && quit == false) {
	}
	survive_close(ctx);

	return 0;
}
