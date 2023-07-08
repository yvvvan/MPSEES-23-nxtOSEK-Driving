#include <gtest/gtest.h>

#include <fstream>
#include <thread>

#include "helper.hpp"
#include "modules/slam/mapping.hpp"

BlackBoard& m_blackboard = BlackBoard::getInstance();

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
  while (m_blackboard.localization_enabled.get() &&
         std::getline(trajectory_file, line)) {
    // std::cout << "line: " << line << std::endl;
    auto values = splitStringBySpace(line);

    Coordinates coordinates = Coordinates(stod(values[0]), stod(values[1]), 0);

    m_blackboard.is_intersection = strcasecmp("true", values[2].c_str()) == 0;

    std::array<bool, 3> exits_detected{
        strcasecmp("true", values[3].c_str()) == 0,
        strcasecmp("true", values[4].c_str()) == 0,
        strcasecmp("true", values[5].c_str()) == 0};

    m_blackboard.exits_intersection.set(exits_detected);
    m_blackboard.coordinates.set(coordinates);

    if (m_blackboard.direction_changed.get()) {
      std::cout << "direction passed: " << m_blackboard.direction.get()
                << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "mapping state: finished " << m_blackboard.mapping_finished.get()
            << std::endl;

  auto exits_duration_matrix = m_blackboard.cost_exits_map.get();
  auto connection_matrix = m_blackboard.connection_map.get();
  auto duration_matrix = m_blackboard.duration_map.get();

  std::cout << "intersection No. | Connection at Exits S-W-N-E" << std::endl;
  for (const auto& elem : connection_matrix) {
    int id = elem.first;
    int south_exit = std::get<0>(elem.second);
    int left_exit = std::get<1>(elem.second);
    int north_exit = std::get<2>(elem.second);
    int right_exit = std::get<3>(elem.second);
    std::cout << id << " \t| " << south_exit << " \t" << left_exit << " \t"
              << north_exit << " \t" << right_exit << std::endl;
  }
  std::cout << "intersection No. | Costs Between Intersection No." << std::endl;
  for (const auto& elem : duration_matrix) {
    int id = elem.first;
    int i0 = std::get<0>(elem.second);
    int i1 = std::get<1>(elem.second);
    int i2 = std::get<2>(elem.second);
    int i3 = std::get<3>(elem.second);
    int i4 = std::get<4>(elem.second);
    int i5 = std::get<5>(elem.second);
    int i6 = std::get<6>(elem.second);
    int i7 = std::get<7>(elem.second);
    int i8 = std::get<8>(elem.second);
    int i9 = std::get<9>(elem.second);
    std::cout << id << " \t| " << i0 << " \t" << i1 << " \t" << i2 << " \t"
              << i3 << " \t" << i4 << " \t" << i5 << " \t" << i6 << " \t" << i7
              << " \t" << i8 << " \t" << i9 << std::endl;
  }
  std::cout << "intersection No. | Duration to next intersection S-W-N-E"
            << std::endl;
  for (const auto& elem : exits_duration_matrix) {
    int id = elem.first;
    int south_exit = std::get<0>(elem.second);
    int left_exit = std::get<1>(elem.second);
    int north_exit = std::get<2>(elem.second);
    int right_exit = std::get<3>(elem.second);
    std::cout << id << " \t| " << south_exit << " \t" << left_exit << " \t"
              << north_exit << " \t" << right_exit << std::endl;
  }

  return 0;
}

TEST(MappingTest, MappingThreadTest) {
  m_blackboard.localization_enabled.set(true);
  m_blackboard.mapping_enabled.set(true);

  /* start fake ORB-SLAM thread */
  std::string trajectory_file_name = "../test/testfiles/testTrajectory.txt";
  std::thread fake_orb_slam_thread(runFakeORBSLAM, trajectory_file_name);

  /* start mapping thread */
  Mapping mapping = Mapping();
  std::thread mapping_thread(&Mapping::exec_thread, &mapping);

  /* wait for threads to finish */
  fake_orb_slam_thread.join();
  mapping_thread.join();
}
