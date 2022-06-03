#include "utility_file_read.h"


// case-insensiteive string comparison
// bool iequals(const string& a, const string& b)
// {
//     return std::equal(a.begin(), a.end(),
//                       b.begin(), b.end(),
//                       [](char a, char b) {
//                           return tolower(a) == tolower(b);
//                       });
// }

bool iequals(const string& a, const string& b)
{
    unsigned int sz = a.size();
    if (b.size() != sz)
        return false;
    for (unsigned int i = 0; i < sz; ++i)
        if (tolower(a[i]) != tolower(b[i]))
            return false;
    return true;
}

void getAllFormatFiles(string inputPath, vector<string>& filenames, string format)
{
    for(auto& p: fs::recursive_directory_iterator(inputPath))
    {
        if( iequals(p.path().extension(), format) )
            filenames.push_back(p.path());
    }
}