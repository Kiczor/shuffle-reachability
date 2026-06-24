default: solutionthreaded

solutionthreaded: solution.cpp ThreadPool.h generator.cpp generator.h bitutilities.h bitutilities.cpp
	g++ -pthread -lstdc++fs -std=c++23 -ftracer -flto -Wall -Wextra -Wshadow -march=native -Ofast bitutilities.cpp generator.cpp solution.cpp -o solution

solutionclang:
	clang++ -pthread -std=c++23 -flto -Wall -Wextra -Wshadow -march=native -O3 bitutilities.cpp generator.cpp solution.cpp -o solution

clean:
	rm solution