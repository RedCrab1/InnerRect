
#include "rotator.h"

void Rotator::rotateImage(const cv::Mat& image, cv::Mat& rotated_image, const cv::Mat& R, const cv::Rect& bounding_rect)
{
	// rotate the image
	cv::warpAffine(image, rotated_image, R, bounding_rect.size(), cv::INTER_AREA);

	// apply a binary filter to create a binary image, also use a closing operator to smooth the output (the rotation might produce
	// black pixels reaching into the white area that were not there before, causing new, wrong cells to open)
	cv::threshold(rotated_image, rotated_image, 127, 255, CV_THRESH_BINARY);
}

// compute the affine rotation matrix for rotating a image into parallel alignment with x-axis (longer side of the image is aligned with x-axis)
// R is the transform
// bounding_rect is the ROI of the warped image
double Rotator::computeImageRotationMatrix(const cv::Mat& image, cv::Mat& R, cv::Rect& bounding_rect, const cv::Point* center, const double rotation_offset)
{
	// rotation angle of the image s.t. the most occurring gradient is in 90 degree to the x-axis
	double rotation_angle = computeImageMainDirection(image) + rotation_offset;
	std::cout << "Rotator::computeImageRotationMatrix: main rotation angle: " << rotation_angle << std::endl;

	// get rotation matrix R for rotating the image around the center of the room contour
	//	Remark: rotation angle in degrees for opencv
	cv::Point center_of_rotation;
	if (center == 0)
	{
		cv::Point min_room, max_room;
		getMinMaxCoordinates(image, min_room, max_room);
		center_of_rotation.x = 0.5*(min_room.x+max_room.x);
		center_of_rotation.y = 0.5*(min_room.y+max_room.y);
	}
	else
		center_of_rotation = *center;

	// compute rotation
	R = cv::getRotationMatrix2D(center_of_rotation, (rotation_angle*180)/CV_PI, 1.0);

	// determine bounding rectangle to find the size of the new image
	bounding_rect = cv::RotatedRect(center_of_rotation, image.size(), (rotation_angle*180)/CV_PI).boundingRect();
	// adjust transformation matrix
	R.at<double>(0,2) += 0.5*bounding_rect.width - center_of_rotation.x;
	R.at<double>(1,2) += 0.5*bounding_rect.height - center_of_rotation.y;

	return rotation_angle;
}

// computes the major direction of the walls from a map 
// the map (image, CV_8UC1) is black (0) at impassable areas and white (255) on drivable areas
double Rotator::computeImageMainDirection(const cv::Mat& image)
{

	// compute Hough transform on edge image of the map
	cv::Mat edge_map;

	cv::Canny(image, edge_map, 50, 150, 3);
	

	std::vector<cv::Vec4i> lines;
	double min_line_length = 20;	// in [m]
	for (; min_line_length > 2; min_line_length -= 4)
	{
		cv::HoughLinesP(edge_map, lines, 1, CV_PI/180, min_line_length, min_line_length, 1.5*min_line_length);
		cv::Mat room_hough = edge_map.clone();
		for (size_t i=0; i<lines.size(); ++i)
		{
			cv::Point p1(lines[i][0], lines[i][1]), p2(lines[i][2], lines[i][3]);
			cv::line(room_hough, p1, p2, cv::Scalar(128), 2);
		}
		if (lines.size() >= 4)
			break;
	}
	// setup a histogram on the line directions weighted by their length to determine the major direction
	Histogram<double> direction_histogram(0, CV_PI, 36);
	for (size_t i=0; i<lines.size(); ++i)
	{
		double dx = lines[i][2] - lines[i][0];
		double dy = lines[i][3] - lines[i][1];
		if(dy*dy+dx*dx > 0.0)
		{
			double current_direction = std::atan2(dy, dx);
			while (current_direction < 0.)
				current_direction += CV_PI;
			while (current_direction > CV_PI)
				current_direction -= CV_PI;
			direction_histogram.addData(current_direction, sqrt(dy*dy+dx*dx));
			//std::cout << " dx=" << dx << "   dy=" << dy << "   dir=" << current_direction << "   len=" << sqrt(dy*dy+dx*dx) << std::endl;
		}
	}
	return direction_histogram.getMaxBinPreciseVal();
}

void Rotator::transformPathBackToOriginalRotation(const std::vector<cv::Point2f>& fov_middlepoint_path, std::vector<cv::Point2f>& path_fov_poses, const cv::Mat& R)
{
	path_fov_poses.clear();

	// transform the calculated path back to the originally rotated map
	cv::Mat R_inv;
	cv::invertAffineTransform(R, R_inv);
	std::vector<cv::Point2f> fov_middlepoint_path_transformed;
	cv::transform(fov_middlepoint_path, fov_middlepoint_path_transformed, R_inv);

	// create poses with an angle
	transformPointPathToPosePath(fov_middlepoint_path_transformed, path_fov_poses);
}

void Rotator::transformPointPathToPosePath(const std::vector<cv::Point2f>& point_path, std::vector<cv::Point2f>& pose_path)
{
	// create poses with an angle
	for(size_t point_index = 0; point_index < point_path.size(); ++point_index)
	{
		// get the vector from the previous to the current point
		const cv::Point2f& current_point = point_path[point_index];

		// add the next navigation goal to the path
		cv::Point2f current_pose;
		current_pose.x = current_point.x;
		current_pose.y = current_point.y;
		pose_path.push_back(current_pose);

	}
}

void Rotator::transformPointPathToPosePath(std::vector<cv::Point2f>& pose_path)
{
	// create point vector
	std::vector<cv::Point2f> point_path;
	for (size_t i=0; i<pose_path.size(); ++i)
		point_path.push_back(cv::Point2f(pose_path[i].x, pose_path[i].y));

	// create poses with an angle
	pose_path.clear();
	transformPointPathToPosePath(point_path, pose_path);
}

void Rotator::getMinMaxCoordinates(const cv::Mat& map, cv::Point& min_room, cv::Point& max_room)
{
	min_room.x = std::numeric_limits<int>::max();
	min_room.y = std::numeric_limits<int>::max();
	max_room.x = 0;
	max_room.y = 0;
	for (int v=0; v<map.rows; ++v)
	{
		for (int u=0; u<map.cols; ++u)
		{
			if (map.at<uchar>(v,u)==255)
			{
				min_room.x = std::min(min_room.x, u);
				min_room.y = std::min(min_room.y, v);
				max_room.x = std::max(max_room.x, u);
				max_room.y = std::max(max_room.y, v);
			}
		}
	}
}
