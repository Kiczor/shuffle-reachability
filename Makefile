default: solutionthreaded

solutionthreaded: solution.cpp ThreadPool.h generator.cpp generator.h bitutilities.h bitutilities.cpp
	g++ -pthread -lstdc++fs -std=c++23 -ftracer -flto -Wall -Wextra -Wshadow -march=native -Ofast bitutilities.cpp generator.cpp solution.cpp -o solution

clean:
	rm solution