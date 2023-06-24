#include <gtest/gtest.h>
#include <fstream>
#include <thread>


#include "slam/mapping.hpp"

BlackBoard &m_blackboard = BlackBoard::getInstance();

std::vector<std::string> splitStringBySpace(const std::string& input) {
    std::vector<std::string> tokens;
    std::istringstream iss(input);
    std::string token;

    while (iss >> token) {
        tokens.push_back(token);
    }

    return tokens;
}

/**
 * @brief Create a fake ORB-SLAM thread which creates the coordinates
 */
int runFakeORBSLAM(const std::string& trajectory_file_name) {

    /* open trajectory file */
    auto trajectory_file = std::ifstream(trajectory_file_name);
    if (!trajectory_file.is_open()) {
        std::cout << "Error opening trajectory file" << std::endl;
        return -1;
    }

    std::string line;
    while (m_blackboard.localization_enabled.get() && std::getline(trajectory_file, line)) {
        //std::cout << "line: " << line << std::endl;
        auto values = splitStringBySpace(line);

        Coordinates coordinates = Coordinates(stod(values[0]), stod(values[1]), 0);
        

        m_blackboard.intersection_detected = strcasecmp("true",values[2].c_str()) == 0;

        std::array<bool, 3> exits_detected {strcasecmp("true",values[3].c_str()) == 0,
                                            strcasecmp("true",values[4].c_str()) == 0,
                                            strcasecmp("true",values[5].c_str()) == 0};

        m_blackboard.exits_detected.set(exits_detected);
        m_blackboard.coordinates.set(coordinates);

        if(m_blackboard.direction_changed.get()){
            std::cout << "direction passed: " << m_blackboard.direction.get() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "mapping state: finished " << m_blackboard.mapping_finished.get() << std::endl;
    return 0;
}

TEST(MappingTest, MappingThreadTest) {

    m_blackboard.localization_enabled.set(true);
    m_blackboard.mapping_enabled.set(true);

    /* start fake ORB-SLAM thread */
    std::string trajectory_file_name = "../test/testTrajectory.txt";
    std::thread fake_orb_slam_thread(runFakeORBSLAM, trajectory_file_name);

    /* start mapping thread */
    Mapping mapping = Mapping();
    std::thread mapping_thread(&Mapping::exec_thread, &mapping);

    /* wait for threads to finish */
    fake_orb_slam_thread.join();
    mapping_thread.join();
}
