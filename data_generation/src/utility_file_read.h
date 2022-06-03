#ifndef _UTILITY_FILES_H
#define _UTILITY_FILES_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <experimental/filesystem>
using namespace std;
namespace fs = std::experimental::filesystem;

bool iequals(const string& a, const string& b);
void getAllFormatFiles(string inputPath, vector<string>& filenames, string format);

#endif