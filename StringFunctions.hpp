/*
 * StringFunctions.hpp
 *
 *  Created on: Dec 8, 2014
 *      Author: atabb
 */

#ifndef ROBOTWORLDHANDEYECALIDUALEXP0_SRC_STRINGFUNCTIONS_HPP_
#define ROBOTWORLDHANDEYECALIDUALEXP0_SRC_STRINGFUNCTIONS_HPP_

#include <iostream>
#include <fstream>
using std::string;


template<class T>
string ToString(T arg)
{
	std::ostringstream s;

	s << arg;

	return s.str();

}


template<class T>
T FromString(const std::string& s)
{
	std::istringstream stream (s);
	T t;
	stream >> t;
	return t;


}


#endif /* ROBOTWORLDHANDEYECALIDUALEXP0_SRC_STRINGFUNCTIONS_HPP_ */
