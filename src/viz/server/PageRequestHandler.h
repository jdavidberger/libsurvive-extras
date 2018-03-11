#pragma once

#include <Poco/Net/HTTPRequestHandler.h>
#include <Poco/Net/HTTPServerResponse.h>

class PageRequestHandler : public Poco::Net::HTTPRequestHandler
/// Return a HTML document with some JavaScript creating
/// a WebSocket connection.
{
	std::string dir;

  public:
	explicit PageRequestHandler(const std::string &dir);

	void handleRequest(Poco::Net::HTTPServerRequest &request, Poco::Net::HTTPServerResponse &response);
};
