#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
#include <vector>
#include <map>
#include <string>
#include <iostream>

using json_h = nlohmann::json;

int main()
{
	json_h j2 = {
		{"pi", 3.141},
		{"happy", true},
		{"name", "Niels"},
		{"nothing", nullptr},
		{"answer", {
			{"everything", 42}
		}},
		{"list", {1, 0, 2}},
		{"object", {
			{"currency", "USD"},
			{"value", 42.99}
		}}
	};
	j2["foo"] = 23;
	j2["bar"] = false;
	j2.emplace("weather", "sunny");
	std::vector<int> v {10, 80, 100};
	j2["numbers"] = v;
	std::map<std::string, int> m = {{"one", 1}, {"two", 2}};
	j2["kv"] = m;
	j2["gear"]["suits"] = "2099";
	
	// write to JSON file
	std::ofstream out("test.json");
	out << std::setw(4) << j2 << std::endl;
	// dump() to finish serialization; with 2 indent
	std::cout << j2.dump(2) << std::endl;

	// de-serialize, e,g, raw string
	std::string str = R"(
		{
			"name": "wayne",
			"age": 23,
			"married": true
		}
	)";
	auto jr = json_h::parse(str);
	assert(jr["age"] == 23);
	assert(jr["name"] == "wayne");

	// read a JSON file
	std::ifstream in("test.json");
	json_h j;
	in >> j;


	return 0;
}