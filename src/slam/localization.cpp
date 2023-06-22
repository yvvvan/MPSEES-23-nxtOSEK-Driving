/**
 * This file should contain the mapping class for the SESE driving projekt.
 * Currently it only contains a test script to test ORB_SLAM on the raspberry pi.
*/
#include <signal.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <ctime>

#include <System.h>

#include "localization.hpp"


Localization::Localization(std::string vocabularyFile, std::string configFile) {
    this->vocabularyFile = vocabularyFile;
    this->configFile = configFile;

    /* create slam instance */
    this->slam = new ORB_SLAM3::System(vocabularyFile, configFile, ORB_SLAM3::System::MONOCULAR, false);
}

Localization::~Localization() {

    /* Stop ORB SLAM */
    if (slam->GetTrackingState() != ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY) {
        slam->Shutdown();
    }

    /* Delete ORB SLAM */
    if (this->slam != nullptr) {
        delete this->slam;
    }
}

int Localization::exec_thread() {

    /* Loop, which analyses the incoming camera frames */
    while(this->blackboard.localization_enabled == true && this->blackboard.camera_enabled == true) {

        /* If the frame was read successfully */
        if (!this->blackboard.frame.get().empty()) {

            // Get the current system time as a timestamp
            std::time_t timestamp = std::time(nullptr);

            Sophus::SE3f camera_pose = slam->TrackMonocular(this->blackboard.frame.get(), (double) timestamp);

            Eigen::Quaternionf q = camera_pose.unit_quaternion();
            Eigen::Vector3f twb = camera_pose.translation();
            // TODO translate to Coordinates Class correctly
            this->blackboard.coordinates = Coordinates(twb(0), twb(1), twb(2), q.x());

            std::cout << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

        } else {
            std::cerr << "No frame received" << std::endl;
            /* If the frame was not read successfully, exit the loop */
            return -1;
        }
    }

    return 0;
}