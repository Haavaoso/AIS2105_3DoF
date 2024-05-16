#include <iostream>
#include <thread>
#include <vector>
#include <cstdlib>

void run_command(const std::string& command) {
    std::cout << "Executing: " << command << std::endl;
    std::system(command.c_str());
}

int main() {
    std::vector<std::string> commands = {
        "ros2 run node_hell preprocess",
        "ros2 run node_hell hough",
        "ros2 run node_hell regulator",
        "ros2 run node_hell contour",
        "ros2 run node_hell camera_kin",
	//"ros2 run node_hell yeeter",
	//"ros2 run node_hell tester"
    };

    std::vector<std::thread> threads;

    for (const auto& cmd : commands) {
        threads.emplace_back(run_command, cmd);
    }

    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    return 99;
}
