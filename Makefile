default: solutionthreaded

solutionthreaded: solution.cpp ThreadPool.h generator.cpp generator.h bitutilities.h bitutilities.cpp
	g++ -pthread -lstdc++fs -std=c++17 -Wall -Wextra -Wshadow -march=native -O2 bitutilities.cpp generator.cpp solution.cpp -o solution

clean:
	rm solution