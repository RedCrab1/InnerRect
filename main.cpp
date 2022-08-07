#include "innerRect.h"

int main()
{
	cv::Mat image;
	image = cv::imread("./test_image.png", CV_LOAD_IMAGE_GRAYSCALE);
	InnerRect test;
	std::vector<cv::Point2f> Rect_points;
	test.getInnerRect(image,  Rect_points, 4);

	if(Rect_points.size() < 4)
	{
		std::cout<<" get InnerRect failed.."<<std::endl;
		return 0;
	}

	cv::Mat InnerRect_image = image.clone();
	cv::line(InnerRect_image, Rect_points[0], Rect_points[1], cv::Scalar(127));
	cv::line(InnerRect_image, Rect_points[1], Rect_points[2], cv::Scalar(127));
	cv::line(InnerRect_image, Rect_points[2], Rect_points[3], cv::Scalar(127));
	cv::line(InnerRect_image, Rect_points[3], Rect_points[0], cv::Scalar(127));

	cv::imwrite("./InnerRect_image.png", InnerRect_image);

	return 0;
}