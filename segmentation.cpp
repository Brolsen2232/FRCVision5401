#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

Mat segmentation(Mat frame, int clusterCount){
    Mat _frame(frame.size(), frame.type());
    Mat points;
    blur(frame, frame, Size(15, 15));
    blur(frame, frame, Size(15, 15));
    
    frame.convertTo(points, CV_32FC3);
    points = points.reshape(3, frame.rows*frame.cols);
    
    Mat_<int> clusters(points.size(), CV_32SC1);
    Mat centers;

    kmeans(points, clusterCount, clusters, TermCriteria(1, 5, 1.0), 1, KMEANS_PP_CENTERS, centers);
    MatIterator_<Vec3b> it = _frame.begin<Vec3b>(), it_end = _frame.end<Vec3b>();
    for(int i = 0; it != it_end; it++, i++){
        Vec3f &rgb = centers.at<Vec3f>(clusters(i),0);
        (*it)[0] = saturate_cast<uchar>(rgb[0]);
        (*it)[1] = saturate_cast<uchar>(rgb[1]);
        (*it)[2] = saturate_cast<uchar>(rgb[2]);

    }  
    

    return _frame;
}


