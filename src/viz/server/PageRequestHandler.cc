#include "PageRequestHandler.h"
#include <Poco/Net/HTTPServerRequest.h>
#include <fstream>
#include <iterator>
PageRequestHandler::PageRequestHandler(const std::string &dir) : dir(dir) {}

void PageRequestHandler::handleRequest(Poco::Net::HTTPServerRequest &request, Poco::Net::HTTPServerResponse &response) {
	auto uri = request.getURI();
	if (uri.empty())
		uri = "/index.html";

	if (uri[uri.size() - 1] == '/')
		uri = uri + "index.html";

	std::ostream &ostr = response.send();
	response.setChunkedTransferEncoding(true);
	response.setContentType("text/html");

	{
		std::ifstream file(dir + uri);
		if (file) {
			file >> std::noskipws;
			std::copy(std::istream_iterator<char>(file), std::istream_iterator<char>(),
					  std::ostream_iterator<char>(ostr, ""));
			return;
		}
	}
}
