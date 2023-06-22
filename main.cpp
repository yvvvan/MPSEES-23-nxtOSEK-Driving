#include <signal.h>
#include <csignal>
#include "camera/camera.hpp"
#include "slam/localization.hpp"
#include "blackboard/BlackBoard.hpp"

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
    BlackBoard &blackboard = BlackBoard::getInstance();

    /* create Camera and ORB-SLAM Module */
    Localization localization(vocabularyFile, configFile);
    PiCamera camera(20);

    blackboard.camera_enabled = true;
    blackboard.localization_enabled = true;

    /* Start Camera */
    std::thread cameraThread(&PiCamera::run, &camera);

    /* Start ORB-SLAM */
    std::thread localizationThread(&Localization::exec_thread, &localization);

    while(!stop){};

    /* Stop Camera and ORB-SLAM */
    blackboard.localization_enabled = false;
    blackboard.camera_enabled = false;

    /* wait for threads to finish */
    cameraThread.join();
    localizationThread.join();
}

//#include <opencv2/opencv.hpp>
//
//int main() {
//    cv::VideoCapture video("/home/jakob/Documents/SESE_Projekt/mpsees/test/test_video_with_controller.mp4");
//
//    if (!video.isOpened()) {
//        std::cout << "Error opening video file" << std::endl;
//        return -1;
//    }
//
//    cv::Mat frame;
//    while (true) {
//        if (!video.read(frame))
//            break;
//
//        cv::imshow("Video", frame);
//
//        if (cv::waitKey(25) == 'q')
//            break;
//    }
//
//    video.release();
//    cv::destroyAllWindows();
//
//    return 0;
//}
