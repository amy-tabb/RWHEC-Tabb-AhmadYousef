/*
 * DirectoryFunctions.hpp
 *
 *  Created on: Dec 20, 2011
 *      Author: atabb
 */

#ifndef ROBOTWORLDHANDEYECALIDUALEXP0_SRC_DIRECTORYFUNCTIONS_HPP_
#define ROBOTWORLDHANDEYECALIDUALEXP0_SRC_DIRECTORYFUNCTIONS_HPP_


# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <dirent.h>
#include <vector>
#include <iostream>

using std::vector;
using std::string;

void ReadDirectory(string dir_name, vector<string>& content_names);


#endif /* ROBOTWORLDHANDEYECALIDUALEXP0_SRC_DIRECTORYFUNCTIONS_HPP_ */
