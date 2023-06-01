/**
 * This file should contain the mapping class for the SESE driving projekt.
 * Currently it only contains a test script to test ORB_SLAM on the raspberry pi.
*/
#include "core/libcamera_app.hpp"
#include "core/options.hpp"

#include <ORB_SLAM3/include/System.h>


static void event_loop(LibcameraApp &app, ORB_SLAM3::System &slam)
{
    Options const *options = app.GetOptions();

    app.OpenCamera();
    app.ConfigureViewfinder();
    app.StartCamera();

    auto start_time = std::chrono::high_resolution_clock::now();

    for (unsigned int count = 0; ; count++)
    {
        LibcameraApp::Msg msg = app.Wait();
        if (msg.type == LibcameraApp::MsgType::Timeout)
        {
            LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
            app.StopCamera();
            app.StartCamera();
            continue;
        }
        if (msg.type == LibcameraApp::MsgType::Quit)
            return;
        else if (msg.type != LibcameraApp::MsgType::RequestComplete)
            throw std::runtime_error("unrecognised message!");

        LOG(2, "Viewfinder frame " << count);
        auto now = std::chrono::high_resolution_clock::now();
        if (options->timeout && now - start_time > std::chrono::milliseconds(options->timeout))
            return;

        CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

        slam.TrackMonocular(completed_request);
    }

    slam.Shutdown();
}

int main(int argc, char **argv) {

    /* create slam instance */
    const std::string vocabularyFile = "../libs/ORB_SLAM/Vocabulary/ORBvoc.txt";
    const std::string configFile = "mono_raspi_cam.yaml";
    const int frames = 10;
    ORB_SLAM3::System slam(vocabularyFile, configFile, ORB_SLAM3::System::MONOCULAR, false);

    /* create camera node */
    LibcameraApp app;

    event_loop(app, slam);
}