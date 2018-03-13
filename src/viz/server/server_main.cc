#define FLT double

#include "SurviveServer.h"
#include <chrono>
#include <iostream>
#include <libsurvive/survive.h>
#include <posers/init_extras.h>

void send_angle_info(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode, double length, double angle,
					 uint32_t lh) {
	survive_default_angle_process(so, sensor_id, acode, timecode, length, angle, lh);
	((SurviveServer *)so->ctx->user_ptr)->send_angle_info(so, sensor_id, acode, timecode, length, angle, lh);
}

int count = 0;
unsigned long last_display = 0;
void send_pose_info(SurviveObject *so, uint8_t lighthouse, SurvivePose *pos) {
	unsigned long now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
	survive_default_raw_pose_process(so, lighthouse, pos);
	if (now > last_display + 100) {
		std::cerr << "Pose at: " << ((double)count * 10) << "hz" << std::endl;
		count = 0;
		last_display = now;
	}
	count++;
    ((SurviveServer *)so->ctx->user_ptr)->send_pose_info(so, lighthouse, &pos->Pos[0], &pos->Rot[0]);

}

void send_imu_info(SurviveObject *so, int mask, FLT *accelgyro, uint32_t timecode, int id) {
	survive_default_imu_process(so, mask, accelgyro, timecode, id);
	((SurviveServer *)so->ctx->user_ptr)->send_imu_info(so, mask, accelgyro, timecode, id);
}

int main(int argc, char **argv) {
	try {
		SurviveServer app;
		auto ret = app.run(argc, argv);

		if (ret != 0) {
			return ret;
		}

		init_extras();
		auto ctx = survive_init(0);
		ctx->bsd[0].PositionSet = ctx->bsd[1].PositionSet = 0;
		ctx->user_ptr = &app;
		survive_install_raw_pose_fn(ctx, send_pose_info);
		survive_install_angle_fn(ctx, send_angle_info);
		survive_install_imu_fn(ctx, send_imu_info);
		survive_cal_install(ctx);

		/*
		sleep(5);
		SurvivePose pose = {
				{1.2724328, 1.2279340, -0.5676212 + 1},
				{0.8470495, 0.0103849, 0.5313805, -0.0058371}
		};
		app.send_lh_info(0, &pose);

		SurvivePose pose2 = {
				{-0.6976100, 0.9608083, -1.1541746 + 1},
				{ 0.3482170, -0.0076827, -0.9373258, 0.0103104}
		};
		app.send_lh_info(1, &pose2);
*/
		// wait for CTRL-C or kill
		bool quit = false;
		while (survive_poll(ctx) == 0 && quit == false) {
		}

		while (quit == false) {
			sleep(1);
		}
		survive_close(ctx);

		return 0;
	} catch (Poco::Exception &exc) {
		std::cerr << exc.displayText() << std::endl;
		return Poco::Util::Application::EXIT_SOFTWARE;
	}
}
