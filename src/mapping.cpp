/**
 * This file should contain the mapping class for the SESE driving projekt.
 * Currently it only contains a test script to test ORB_SLAM on the raspberry pi.
*/
#include <signal.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <ctime>

#include <ORB_SLAM3/include/System.h>

bool stop = false;

int main(int argc, char **argv) {

    /* handle keyboard interrupt */
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = [](int s) {
        std::cout << "Caught SIGINT, exiting..." << std::endl;
        stop = true;
    };

    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    /* create slam instance */
    const std::string vocabularyFile = "/home/pi/mpsees-23-nxtosek-driving/libs/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string configFile = "/home/pi/mpsees-23-nxtosek-driving/src/mono_raspi_cam.yaml";
    ORB_SLAM3::System slam(vocabularyFile, configFile, ORB_SLAM3::System::MONOCULAR, false); // true to show visuals

    std::string connstr = "libcamerasrc ! video/x-raw,framerate=20/1 ! appsink";
    cv::VideoCapture cap(connstr, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    while(!stop) {
        cv::Mat frame;
        cap.read(frame);

        // If the frame was read successfully
        if (!frame.empty()) {

            // Get the current system time as a timestamp
            std::time_t timestamp = std::time(nullptr);

            Sophus::SE3f camera_pose = slam.TrackMonocular(frame, (double) timestamp);

            Eigen::Quaternionf q = camera_pose.unit_quaternion();
            Eigen::Vector3f twb = camera_pose.translation();
            cout << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

            /* Show current frame with opencv */
            cv::imshow("Frame", frame);

            // Wait for the 'q' key to be pressed to exit the loop
            if (cv::waitKey(1) == 'q')
                break;
        } else {
            cerr << "No frame received" << endl;
            // If the frame was not read successfully, exit the loop
            break;
        }
    }

    /* Stop ORB SLAM */
    if (slam.GetTrackingState() != ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY) {
        slam.Shutdown();
    }

    /* Release the VideoCapture object and close OpenCV windows */
    if (cap.isOpened()){
        cap.release();
    }
    cv::destroyAllWindows();
    exit(0);
}
