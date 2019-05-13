#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "lanedetector.h"
#include <fstream>
//#include "LaneDetector.cpp"

using namespace std;

/**
*@brief Function main that runs the main algorithm of the lane detection.
*@brief It will read a video of a car in the highway and it will output the
*@brief same video but with the plotted detected lane
*@param argv[] is a string to the full path of the demo video
*@return flag_plot tells if the demo has sucessfully finished
*/
int main() {

    // for videos
    /*
    cv::VideoCapture cap("Test.MOV");
    if (!cap.isOpened())
    {
        std::cout << "Read video Failed !" << std::endl;
        return -1;
    }
    */

    LaneDetector lanedetector;
    cv::Mat frame;
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
    std::string turn;
    int flag_plot = -1;
    //int i = 0;


     std::vector<cv::String> filenames; // notice here that we are using the Opencv's embedded "String" class
     cv::String folder = "/home/parallels/Documents/Dataset by video recorder/Daytime/Inter-city road/test";

     cv::glob(folder, filenames);




     //for seperate image files
     for(size_t i = 0; i < filenames.size(); ++i)
     {
        std::cout<<filenames[i]<<std::endl;
        cv::Mat frame = cv::imread(filenames[i]);
        cv::namedWindow("1");
        cv::imshow("1",frame);
        if(!frame.data)
            std::cerr << "Problem loading image!!!" << std::endl;


            // Denoise the image using a Gaussian filter
            img_denoise = lanedetector.deNoise(frame);

            // Detect edges in the image
            img_edges = lanedetector.edgeDetector(img_denoise);

            // Mask the image so that we only get the ROI
            img_mask = lanedetector.mask(img_edges);

            // Obtain Hough lines in the cropped image
            lines = lanedetector.houghLines(img_mask);

            if (!lines.empty())
            {
                // Separate lines into left and right lines
                left_right_lines = lanedetector.lineSeparation(lines, img_edges);

                // Apply regression to obtain only one line for each side of the lane
                lane = lanedetector.regression(left_right_lines, frame);

                // Predict the turn by determining the vanishing point of the the lines
                turn = lanedetector.predictTurn();

                // Plot lane detection
                flag_plot = lanedetector.plotLane(frame, lane, turn);

                cv::waitKey(25);
            }
            else {
                flag_plot = -1;
            }
            cv::waitKey(0);
     }


    /*
    // for videos
    while (i < 540) {
        // Capture frame
        if (!cap.read(frame))
            break;

        // Denoise the image using a Gaussian filter
        img_denoise = lanedetector.deNoise(frame);

        // Detect edges in the image
        img_edges = lanedetector.edgeDetector(img_denoise);

        // Mask the image so that we only get the ROI
        img_mask = lanedetector.mask(img_edges);

        // Obtain Hough lines in the cropped image
        lines = lanedetector.houghLines(img_mask);

        if (!lines.empty())
        {
            // Separate lines into left and right lines
            left_right_lines = lanedetector.lineSeparation(lines, img_edges);

            // Apply regression to obtain only one line for each side of the lane
            lane = lanedetector.regression(left_right_lines, frame);

            // Predict the turn by determining the vanishing point of the the lines
            turn = lanedetector.predictTurn();

            // Plot lane detection
            flag_plot = lanedetector.plotLane(frame, lane, turn);

            i += 1;
            cv::waitKey(25);
        }
        else {
            flag_plot = -1;
        }
    }
    */
    return flag_plot;
}
