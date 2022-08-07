
// #pragma once
#ifndef __INNER_RECT_H__
#define __INNER_RECT_H__

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include "rotator.h"


const double PI  = 3.14159265359;
const double PI_2_INV = 1./(0.5*PI);



// Struct that is used to create a node and save the corresponding neighbors.
struct InnerExploratorNode
{
	cv::Point center_;
	int leftCount;
	InnerExploratorNode()
	{
		leftCount = 0;
	}
};


class InnerRect
{
public:
	// constructor
	InnerRect();

	void getInnerRect(const cv::Mat& image, std::vector<cv::Point2f>& Rect_points, const double grid_spacing_in_pixel);
  bool completeCellTest(const cv::Mat& image, cv::Point& cell_center, const int cell_size);

};

#endif