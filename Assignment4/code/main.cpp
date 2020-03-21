#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}


cv::Vec3b getColorByDistance(cv::Point2f &tragetpixel,cv::Point2f &curpixel)
{
    // estimate
    cv::Point2f offset = tragetpixel - curpixel;
    float percentage = (offset.x + offset.y) / 2.0f;
    return cv::Vec3b(0,255.0 * percentage,255.0 * percentage);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // std::cout<<"recursive_bezier "<<control_points.size()<<std::endl;
    if (control_points.size() == 2)
        return (1- t) * control_points[0] + t * control_points[1];
    int arrayLength = control_points.size();
    std::vector<cv::Point2f> tempcontrol_point;
    for(int i = 1;i < arrayLength; i ++)
    {
        tempcontrol_point.push_back(t * control_points[i] + (1-t) * control_points[i - 1]);
    }
    return recursive_bezier(tempcontrol_point,t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t = 0.0f;t <= 1.0f ; t += 0.001)
    {
        //std::cout<<"t "<<t<<std::endl;
        cv::Point2f point = recursive_bezier(control_points,t);

        // cv::Point2f point0(floor(point.x),floor(point.y));
        // cv::Point2f point1(ceil(point.x),floor(point.y));
        // cv::Point2f point2(floor(point.x),ceil(point.y));
        // cv::Point2f point3(ceil(point.x),ceil(point.y));

        // window.at<cv::Vec3b>(point0) = getColorByDistance(point0,point);
        // window.at<cv::Vec3b>(point1) = getColorByDistance(point1,point);
        // window.at<cv::Vec3b>(point2) = getColorByDistance(point2,point);
        // window.at<cv::Vec3b>(point3) = getColorByDistance(point3,point);
        

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}


int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
