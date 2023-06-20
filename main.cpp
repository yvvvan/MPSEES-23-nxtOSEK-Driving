#include "camera/camera.hpp"
#include "slam/localization.hpp"
#include "blackboard/blackboard.hpp"

bool stop = false;

int main() {
    const std::string vocabularyFile = "/home/pi/mpsees-23-nxtosek-driving/libs/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string configFile = "/home/pi/mpsees-23-nxtosek-driving/src/mono_raspi_cam.yaml";

    /* handle keyboard interrupt */
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = [](int s) {
        std::cout << "Caught SIGINT, exiting..." << std::endl;
        stop = true;
    };

    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    /* instanciate Blackboard */
    Blackboard &blackboard = BlackBoard.getInstance();

    /* create Camera and ORB-SLAM Module */
    Localization localization(vocabularyFile, configFile);
    PiCamera camera(20);

    blackboard.camera_enabled = true;
    blackboard.localization_enabled = true;

    /* Start Camera */
    std::thread cameraThread(&PiCamera::run, &camera);

    /* Start ORB-SLAM */
    std::thread localizationThread(&Localization::run, &localization);

    while(!stop){};

    /* Stop Camera and ORB-SLAM */
    blackboard.localization_enabled = false;
    blackboard.camera_enabled = false;

    /* wait for threads to finish */
    cameraThread.join();
    localizationThread.join();
}