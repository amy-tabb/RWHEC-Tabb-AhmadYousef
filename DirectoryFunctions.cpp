/*
 * DirectoryFunctions.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: atabb
 */

#include "DirectoryFunctions.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

bool SortbyLength(string s0, string s1){
	if (s0.length() < s1.length()){
		return true;
	}	else {
		if (s0.length() > s1.length()){
			return false;
		}	else {
			if (s0.compare(s1) < 0){
				return true;
			}	else {
				return false;
			}
		}
	}
}

void ReadDirectory(string dir_name, vector<string>& content_names){

	struct dirent * dp;

	// Enter path to directory existing below

	DIR * dir = opendir (dir_name.c_str());
	string s;
	dp = readdir(dir);
	while ( dp != NULL) {

		if (dp->d_name[0] != '.'){
			s= dp->d_name;
			if (s.at(s.size() - 1) != '~'){
				content_names.push_back(dp->d_name);
				//cout << "Reading ..." << dp->d_name << endl;
			}
		}

		dp = readdir(dir);
	}
	closedir (dir);

	//cout << "Exit read dir" << endl;

	std::sort(content_names.begin(), content_names.end(), SortbyLength);

	for (int i = 0; i < int(content_names.size()); i++){
		cout << "Sorted order .... " << content_names[i]<< endl;
	}

}


