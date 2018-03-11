#define FLT double

#include "SurviveServer.h"
#include "PageRequestHandler.h"
#include "Poco/Util/HelpFormatter.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <libsurvive/survive.h>
#include <mutex>
#include <sstream>

void write_buffer(std::stringstream &ss, const FLT *buffer, size_t len, int dim = 1) {
	ss << "[ ";

	for (size_t i = 0; i < len; i++) {
		if (i != 0)
			ss << ", ";

		if (dim != 1)
			ss << "[ ";

		for (size_t j = 0; j < dim; j++) {
			if (j != 0)
				ss << ", ";
			if (std::isnan(buffer[i * dim + j]))
				ss << "null";
			else
				ss << buffer[i * dim + j];
		}

		if (dim != 1)
			ss << " ]";
	}

	ss << " ]";
}

class WebSocketRequestHandler : public Poco::Net::HTTPRequestHandler
/// Handle a WebSocket connection.
{
	std::weak_ptr<SurviveWSServer_p> server;

  public:
	WebSocketRequestHandler(const std::shared_ptr<SurviveWSServer_p> &server) : server(server) {}

	void handleRequest(Poco::Net::HTTPServerRequest &request, Poco::Net::HTTPServerResponse &response) override;
};

struct SurviveWSFactory : public Poco::Net::HTTPRequestHandlerFactory {
	std::string dir;
	std::shared_ptr<SurviveWSServer_p> p;

	Poco::Net::HTTPRequestHandler *createRequestHandler(const Poco::Net::HTTPServerRequest &request) override {
		if (request.find("Upgrade") != request.end() && Poco::icompare(request["Upgrade"], "websocket") == 0)
			return new WebSocketRequestHandler(p);
		else
			return new PageRequestHandler(dir);
	}

	SurviveWSFactory(const std::string &dir, const std::shared_ptr<SurviveWSServer_p> &p) : dir(dir), p(p) {}
};

struct SurviveWSServer_p : public std::enable_shared_from_this<SurviveWSServer_p> {
	struct Socket {
		Poco::Net::WebSocket socket;
		bool hasLHPosition[2] = {};
		bool hasStructure = false;
		bool isClosed = false;
		void send(const std::string &str);

		Socket(const Poco::Net::WebSocket &socket) : socket(socket) {}
	};
	std::mutex socket_mutex;
	std::vector<Socket> sockets;

	void broadcast(const std::string &msg) {
		std::lock_guard<std::mutex> g(socket_mutex);
		for (auto &socket : sockets) {
			if (socket.isClosed)
				continue;

			if (socket.socket.available() > 0) {
				int flags = 0;
				uint8_t buffer[128];
				socket.socket.receiveFrame(buffer, 128, flags);
			}
			socket.send(msg);
		}
	}

	SurviveContext *ctx;
	void AddSocket(Poco::Net::HTTPServerRequest &request, Poco::Net::HTTPServerResponse &response) {
		{
			std::lock_guard<std::mutex> g(socket_mutex);
			sockets.emplace_back(Poco::Net::WebSocket(request, response));
		}
		for (size_t i = 0; i < ctx->objs_ct; i++) {
			OnNewInfo(ctx->objs[i]);
		}
	}
	bool hasConnections() const { return !sockets.empty(); }
	void send_lh_info(int lh, const SurvivePose *pose) {
		std::stringstream ss;
		auto &p = pose->Pos;
		auto &q = pose->Rot;

		ss << R"({ "type": "lighthouse_pose",)"
		   << "\"lighthouse\": " << lh << ", "
		   << "\"position\": ";
		write_buffer(ss, p, 3);

		ss << ", \"quat\": ";
		write_buffer(ss, q, 4);

		ss << "}";

		std::lock_guard<std::mutex> g(socket_mutex);
		for (auto &socket : sockets) {
			socket.send(ss.str());
			socket.hasLHPosition[lh] = true;
		}
	}
	void OnNewInfo(SurviveObject *so) {
		ctx = so->ctx;
		std::lock_guard<std::mutex> g(socket_mutex);
		for (auto &socket : sockets) {
			if (!socket.hasStructure) {
				std::stringstream ss;
				ss <<
					R"({ "type": "tracker_calibration", "tracker": ")" << so->codename << "\","
				   << "\"points\":";
				write_buffer(ss, so->sensor_locations, so->nr_locations, 3);
				ss << ", "
				   << "\"normals\":";
				write_buffer(ss, so->sensor_normals, so->nr_locations, 3);
				ss << "}";
				socket.send(ss.str());

				socket.hasStructure = true;
			}

			for (size_t i = 0; i < 2; i++) {
				if (!socket.hasLHPosition[i] && so->ctx->bsd->PositionSet) {
					std::stringstream ss;
					auto &p = so->ctx->bsd[i].Pose.Pos;
					auto &q = so->ctx->bsd[i].Pose.Rot;
					ss << R"({ "type": "lighthouse_pose",)"
					   << "\"lighthouse\": " << i << ", "
					   << "\"position\": ";
					write_buffer(ss, p, 3);

					ss << ", \"quat\": ";
					write_buffer(ss, q, 4);

					ss << "}";
					socket.send(ss.str());
					socket.hasLHPosition[i] = true;
				}
			}
		}
	}
	Poco::Net::ServerSocket svs;
	// set-up a HTTPServer instance
	std::unique_ptr<Poco::Net::HTTPServer> srv;

	SurviveWSServer_p(const Poco::Util::LayeredConfiguration &config)
		: svs((unsigned short)config.getInt("WebSocketServer.port", 8080)) {}

	int start(const std::string &dir) {
		srv.reset(new Poco::Net::HTTPServer(new SurviveWSFactory(dir, shared_from_this()), svs,
											new Poco::Net::HTTPServerParams));
		srv->start();

		return Poco::Util::Application::EXIT_OK;
	}
};

void SurviveWSServer_p::Socket::send(const std::string &msg) {
	try {
		if (isClosed)
			return;

		socket.sendFrame(msg.c_str(), msg.size(), Poco::Net::WebSocket::FRAME_TEXT);
	} catch (const std::exception &e) {
		isClosed = true;
	}
}

void WebSocketRequestHandler::handleRequest(Poco::Net::HTTPServerRequest &request,
											Poco::Net::HTTPServerResponse &response) {
	auto &app = Poco::Util::Application::instance();
	try {
		if (auto s = server.lock()) {
			s->AddSocket(request, response);
		}
	} catch (Poco::Net::WebSocketException &exc) {
		app.logger().log(exc);
		switch (exc.code()) {
		case Poco::Net::WebSocket::WS_ERR_HANDSHAKE_UNSUPPORTED_VERSION:
			response.set("Sec-WebSocket-Version", Poco::Net::WebSocket::WEBSOCKET_VERSION);
		// fallthrough
		case Poco::Net::WebSocket::WS_ERR_NO_HANDSHAKE:
		case Poco::Net::WebSocket::WS_ERR_HANDSHAKE_NO_VERSION:
		case Poco::Net::WebSocket::WS_ERR_HANDSHAKE_NO_KEY:
			response.setStatusAndReason(Poco::Net::HTTPResponse::HTTP_BAD_REQUEST);
			response.setContentLength(0);
			response.send();
			break;
		}
	}
}

static void broadcast(const std::string &msg) {
	/*for (auto socket : sockets) {
		try{
	  if(socket->available() > 0) {
		  int flags = 0;
		  socket->receiveFrame(buffer, 128, flags);
		}
			socket->sendFrame(msg.c_str(), msg.size(), WebSocket::FRAME_TEXT);
		} catch(const std::exception& e) {
			hasData = false;
			sockets.clear();
			return;
		}

	}*/
}

SurviveServer::SurviveServer() {}

SurviveServer::~SurviveServer() {}

void SurviveServer::initialize(Application &self) {
	loadConfiguration();
	Poco::Util::Application::initialize(self);
}

int SurviveServer::main(const std::vector<std::string> &args) {
	if (args.size() < 1) {
		std::cerr << "Need a directory to point to for file path" << std::endl;
		return Poco::Util::Application::EXIT_USAGE;
	}

	p = std::make_shared<SurviveWSServer_p>(config());
	return p->start(args[0]);
}

void SurviveServer::send_angle_info(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode,
									double length, double angle, uint32_t lh) {
	p->OnNewInfo(so);

	std::stringstream ss;
	ss <<
		R"({ "type": "angle", "tracker":")" << so->codename << "\","
	   << "\"sensor_id\":" << sensor_id << ", "
	   << "\"acode\":" << acode << ", "
	   << "\"timecode\":" << timecode << ", "
	   << "\"length\":" << length << ", "
	   << "\"angle\":" << angle << ", "
	   << "\"lighthouse\":" << (int)lh << "}";

	p->broadcast(ss.str());
}
void SurviveServer::send_imu_info(SurviveObject *so, int mask, FLT *accelgyro, uint32_t timecode, int id) {
	p->OnNewInfo(so);

	std::stringstream ss;
	ss <<
		R"({ "type": "imu", "tracker": ")" << so->codename << "\","
	   << "\"id\": " << id << ", "
	   << "\"timecode\": " << timecode << ", "
	   << "\"accelgyro\": ";

	write_buffer(ss, accelgyro, 6);
	ss << "}";
	p->broadcast(ss.str());
}
void SurviveServer::send_pose_info(SurviveObject *so, uint8_t lighthouse, FLT *pos, FLT *quat) {
	p->OnNewInfo(so);

	std::stringstream ss;
	ss <<
		R"({ "type": "pose", "tracker": ")" << so->codename << "\","
	   << "\"lighthouse\": " << (int)lighthouse << ", "
	   << "\"position\": "; // [" << pos[0] << ", " << pos[1] << ", " << pos[2]
							// << "], " <<
	write_buffer(ss, pos, 3);
	ss << ", \"quat\": ";
	write_buffer(ss, quat, 4);
	ss << "}";

	p->broadcast(ss.str());
}

bool SurviveServer::hasConnections() const { return p->hasConnections(); }

void SurviveServer::send_lh_info(int lh, const SurvivePose *pose) { p->send_lh_info(lh, pose); }
