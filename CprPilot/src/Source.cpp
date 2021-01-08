#include <cpr/cpr.h>
#include <iostream>

int main(int argc, char** argv) {

	auto res = cpr::Get(cpr::Url{ "http://openresty.org" });

	std::cout << res.elapsed << std::endl;  // request cost time
	std::cout << res.url << std::endl;		// requested url
	std::cout << res.status_code << std::endl;	// response status code
	std::cout << res.text.length() << std::endl; // response body data

	for (auto& x: res.header)
	{
		std::cout << x.first << " => " << x.second << std::endl;
	}

    return 0;
}