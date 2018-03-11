#pragma once
#include "Poco/Net/HTTPRequestHandler.h"
#include "Poco/Net/HTTPServer.h"
#include "Poco/Net/HTTPServerRequest.h"
#include "Poco/Net/HTTPServerResponse.h"
#include "Poco/Net/NetException.h"
#include "Poco/Net/WebSocket.h"
#include <Poco/Util/ServerApplication.h>
#include <memory>

struct SurviveObject;
struct SurvivePose;

struct SurviveWSServer_p;
class SurviveServer : public Poco::Util::ServerApplication {
	std::shared_ptr<SurviveWSServer_p> p;

  public:
	SurviveServer();
	~SurviveServer() override;

	void send_angle_info(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode, double length,
						 double angle, uint32_t lh);
	void send_pose_info(SurviveObject *so, uint8_t lighthouse, FLT *pos, FLT *quat);

	void send_imu_info(SurviveObject *so, int mask, FLT *accelgyro, uint32_t timecode, int id);

	void send_lh_info(int lh, const SurvivePose *pose);

	void initialize(Application &self) override;

	bool hasConnections() const;

	int main(const std::vector<std::string> &args) override;
};