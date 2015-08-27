#pragma once

#include <boost/tokenizer.hpp>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>

class InstanceFileMap {
public:
	InstanceFileMap() {}

	InstanceFileMap(const std::string &instance) {
		append(instance);
	}

	void append(const std::string &instance) {
		std::fstream file;
		file.open(instance.c_str(), std::fstream::in);

		if(!file.is_open()) {
			fprintf(stderr, "can't open instance file: %s\n", instance.c_str());
			exit(1);
		}

		std::string line;
		while(!file.eof()) {
			std::getline(file, line);

			if(line.length() > 0 && line[0] == '#') continue;

			int delimeter = line.find("?");
			if(delimeter < 0) {
				map[line] = line;
			} else {

				std::string key = line.substr(0, delimeter);
				key = trim(key);
				std::string value = line.substr(delimeter+1);
				value = trim(value);
				map[key] = value;
			}
		}
		file.close();
	}

	bool exists(const std::string &key) const {
		return map.find(key) != map.end();
	}

	const std::string &value(const std::string &key) const {
		if(!exists(key)) {
			fprintf(stderr, "Key \"%s\" not bound\n", key.c_str());
			exit(1);
		}
		return map.at(key);
	}

	double doubleVal(const std::string &key) const {
		if(!exists(key)) {
			fprintf(stderr, "Key \"%s\" not bound\n", key.c_str());
			exit(1);
		}
		return stod(map.at(key));
	}

	std::vector<std::string> valueList(const std::string &key, const std::string delim = " ") const {
		if(!exists(key)) {
			fprintf(stderr, "Key \"%s\" not bound\n", key.c_str());
			exit(1);
		}
		std::vector<std::string> list;
		boost::char_separator<char> sep(delim.c_str());
		boost::tokenizer< boost::char_separator<char> > tokens(map.at(key), sep);
		for(auto token : tokens) {
			list.push_back(token);
		}
		return list;
	}

	std::vector<double> doubleList(const std::string &key, const std::string delim = " ") const {
		if(!exists(key)) {
			fprintf(stderr, "Key \"%s\" not bound\n", key.c_str());
			exit(1);
		}

		std::vector<double> values;
		boost::char_separator<char> sep(delim.c_str());
		boost::tokenizer< boost::char_separator<char> > tokens(map.at(key), sep);
		for(auto token : tokens) {
			values.push_back(std::stod(token));
		}
		return values;
	}

private:

	inline std::string &ltrim(std::string &s) {
		s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
		return s;
	}


	inline std::string &rtrim(std::string &s) {
		s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
		return s;
	}

	inline std::string &trim(std::string &s) {
		return ltrim(rtrim(s));
	}

	std::unordered_map<std::string, std::string> map;
};