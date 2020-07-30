#include <iostream>
#include <librealsense2/rs.hpp>     
#include <opencv2/opencv.hpp>
#include "segmentation.cpp"
using namespace cv;
int main(int argc, char * argv[]) try
{
    rs2::colorizer color_map;
    rs2::pipeline p;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8);

    rs2::pipeline_profile profile = p.start(cfg);
    namedWindow("RGB", WINDOW_AUTOSIZE);
    namedWindow("Depth", WINDOW_AUTOSIZE);
    while (waitKey(1) < 0 && getWindowProperty("RGB", WND_PROP_AUTOSIZE) >= 0 &&  getWindowProperty("Depth", WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset frames = p.wait_for_frames();
       
        rs2::frame depth_Frame = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame color_Frame = frames.get_color_frame();
        Mat colorFrame(Size(640, 480), CV_8UC3, (void*)color_Frame.get_data(), Mat::AUTO_STEP);
        Mat depthFrame(Size(640, 480), CV_8UC3, (void*)depth_Frame.get_data(), Mat::AUTO_STEP);
        
        imshow("RGB",colorFrame);
        imshow("Depth",depthFrame);
        imshow("Segmented",segmentation(colorFrame, 16));

    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}