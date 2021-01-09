#include <cpr/cpr.h>
#include <iostream>

int main(int argc, char** argv) {

	using namespace std::string_literals;

	const auto url = "http://openresty.org"s;
	auto res = cpr::Get(cpr::Url{ url });

	std::cout << res.elapsed << std::endl;  // request cost time
	std::cout << res.url << std::endl;		// requested url
	std::cout << res.status_code << std::endl;	// response status code
	std::cout << res.text.length() << std::endl; // response body data

	for (auto& x : res.header)
	{
		std::cout << x.first << " => " << x.second << std::endl;
	}

	auto res2 = cpr::Get(
		cpr::Url{ url },
		cpr::Parameters{
		{"a", "1"}, {"b", "2"} }
	);

	auto res3 = cpr::Post(
		cpr::Url{ url },
		cpr::Header{
		{"x", "xxx"},{"expect",""} },
		cpr::Body{ "post data" },
		cpr::Timeout{ 200 }  // 200ms
	);

	// Async call
	auto f = cpr::GetAsync(cpr::Url(url));
	auto res4 = f.get();
	std::cout << res4.elapsed << std::endl;

	return 0;
}