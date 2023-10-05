#include "NumCpp.hpp"
#include <cstdlib>
#include <iostream>

int main(){
	auto a = nc::random::randInt<int>({10,10},0,100);
	std::cout << a <<endl;
	return EXIT_SUCCESS;
}
