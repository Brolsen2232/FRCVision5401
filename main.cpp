#include <iostream>
#include <librealsense2/rs.hpp>     
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char * argv[]) try
{
    rs2::colorizer color_map;
    rs2::pipeline p;
    p.start();
    namedWindow("RGB", WINDOW_AUTOSIZE);
    namedWindow("Depth", WINDOW_AUTOSIZE);
    while (waitKey(1) < 0 && getWindowProperty("RGB", WND_PROP_AUTOSIZE) >= 0 &&  getWindowProperty("Depth", WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset frames = p.wait_for_frames();
        rs2::frame depth_Frame = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame rgb_Frame = frames.get_color_frame();
        Mat rgbFrame(Size(640, 480), CV_8UC3, (void*)rgb_Frame.get_data(), Mat::AUTO_STEP);
        Mat depthFrame(Size(640, 480), CV_8UC3, (void*)depth_Frame.get_data(), Mat::AUTO_STEP);
        imshow("RGB",rgbFrame);
        imshow("Depth",depthFrame);

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