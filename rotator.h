
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "histogram.h"



class Rotator
{
public:
	Rotator()
	{
	}

	void rotateImage(const cv::Mat& Image, cv::Mat& rotated_image, const cv::Mat& R, const cv::Rect& bounding_rect);

	// compute the affine rotation matrix for rotating a image into parallel alignment with x-axis (longer side of the image is aligned with x-axis)
	// R is the transform
	// bounding_rect is the ROI of the warped image
	// rotation_offset is an optional offset to the determined rotation, in [rad]
	// returns the computed optimal rotation angle, in [rad]
	double computeImageRotationMatrix(const cv::Mat& image, cv::Mat& R, cv::Rect& bounding_rect,
			const cv::Point* center=0, const double rotation_offset=0.);

	// computes the major direction of the walls from a image 
	// the image (CV_8UC1) is black (0) at impassable areas and white (255) on drivable areas
	double computeImageMainDirection(const cv::Mat& image);

	// transforms a vector of points back to the original image and generates poses
	void transformPathBackToOriginalRotation(const std::vector<cv::Point2f>& fov_middlepoint_path, std::vector<cv::Point2f>& path_fov_poses, const cv::Mat& R);

	// converts a point path to a pose path with angles
	void transformPointPathToPosePath(const std::vector<cv::Point2f>& point_path, std::vector<cv::Point2f>& pose_path);

	// converts a point path to a pose path with angles, the points are already stored in pose_path
	void transformPointPathToPosePath(std::vector<cv::Point2f>& pose_path);

	// get min/max coordinates of free pixels (255)
	void getMinMaxCoordinates(const cv::Mat& image, cv::Point& min_room, cv::Point& max_room);
};
