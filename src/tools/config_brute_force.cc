#include <iostream>
#include <libsurvive/survive.h>
#include "posers/init_extras.h"
#include "posers/sba_solver.h"

int main(int argc, char** argv)
{
  init_extras();

  RegisterDriver("PoserSBA_SolveOptimalConfig", (void*)sba_bruteforce_config_solver_cb);
  
        auto ctx = survive_init(0);
	
        survive_cal_install(ctx);
        // wait for CTRL-C or kill
        bool quit = false;
        while (survive_poll(ctx) == 0 && quit == false) {

        }
        survive_close(ctx);

        return 0;
}
