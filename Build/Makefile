CXX = g++
CXXFLAGS = -Wall -Wextra -Werror -std=c++11 -I"../Include"

# Target: fea
fea: fea.cpp fea_utilities.hpp
	$(CXX) $(CXXFLAGS) fea.cpp -o fea
	@echo "Done."

# Target: clean
clean:
	rm -f fea