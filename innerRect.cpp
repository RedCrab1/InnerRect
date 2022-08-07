#include "innerRect.h"
#include <stack>

// Constructor
InnerRect::InnerRect()
{

}

void InnerRect::getInnerRect(const cv::Mat& image, std::vector<cv::Point2f>& Rect_points, const double grid_spacing_in_pixel)
{
	const int grid_spacing_as_int = std::floor(grid_spacing_in_pixel);
	const int half_grid_spacing_as_int = std::floor(grid_spacing_in_pixel*0.5);

	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_image;
	Rotator image_rotation;
	image_rotation.computeImageRotationMatrix(image, R, bbox);
	image_rotation.rotateImage(image, rotated_image, R, bbox);

	// compute min/max room coordinates
	cv::Point min_room(1000000, 1000000), max_room(0, 0);
	for (int v=0; v<rotated_image.rows; ++v)
	{
		for (int u=0; u<rotated_image.cols; ++u)
		{
			if (rotated_image.at<uchar>(v,u)==255)
			{
				min_room.x = std::min(min_room.x, u);
				min_room.y = std::min(min_room.y, v);
				max_room.x = std::max(max_room.x, u);
				max_room.y = std::max(max_room.y, v);
			}
		}
	}

	cv::Mat inflated_rotated_image;
	cv::erode(rotated_image, inflated_rotated_image, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int);

	// *********************** II. Find the nodes and their neighbors ***********************
	// get the nodes in the free space
	std::vector<std::vector<InnerExploratorNode> > nodes; // 2-dimensional vector to easily find the neighbors
	int number_of_nodes = 0;


	// todo: create grid in external class - it is the same in all approaches
	// todo: if first/last row or column in grid has accessible areas but center is inaccessible, create a node in the accessible area
	for(int y=min_room.y+half_grid_spacing_as_int; y<max_room.y; y+=grid_spacing_as_int)
	{
		// for the current row create a new set of neurons to span the network over time
		std::vector<InnerExploratorNode> current_row;
		for(int x=min_room.x+half_grid_spacing_as_int; x<max_room.x; x+=grid_spacing_as_int)
		{
			// create node if the current point is in the free space
			InnerExploratorNode current_node;
			current_node.center_ = cv::Point(x,y);
			//if(rotated_image.at<uchar>(y,x) == 255)				
			// could make sense to test all pixels of the cell, not only the center
			if (completeCellTest(inflated_rotated_image, current_node.center_, grid_spacing_as_int) == true)
			{
				//如果是第一个元素，则为0，否则是前一个元素计数基础上加1
				current_node.leftCount = (x == (min_room.x+half_grid_spacing_as_int) ? 0 : (current_row.back().leftCount +1));
				++number_of_nodes;
			}
			// add the obstacle nodes as already visited
			else
			{
				current_node.leftCount = 0;
				++number_of_nodes;
			}
			current_row.push_back(current_node);
		}

		// insert the current row into grid
		nodes.push_back(current_row);
	}
	std::cout << "found " << number_of_nodes <<  " nodes" << std::endl;

	if(nodes.empty())
	{
		return;
	}

	//采用柱状直方图统计方式，对每一列找最大面积
	int max_area = 0;
	int max_up = 0;
	int max_down = 0;
	int max_left = 0;
	int max_right = 0;
	int m = nodes.size();
	int n = nodes[0].size();
	for(int j = 0; j < n; j++)
	{
		std::vector<int> up(m, 0), down(m, 0);
		
		std::stack<int> stk;
		for(int i = 0; i < m; i++)
		{
			while(!stk.empty() && nodes[stk.top()][j].leftCount >= nodes[i][j].leftCount)
			{
				stk.pop();
			}
			up[i] = stk.empty() ? -1 : stk.top();
			stk.push(i);
		}
		stk = std::stack<int>();
		for (int i = m-1; i >= 0; i--){
			while(!stk.empty() && nodes[stk.top()][j].leftCount >= nodes[i][j].leftCount)
			{
				stk.pop();
			}
			down[i] = stk.empty() ? m : stk.top();
			stk.push(i);
		}


		for(int i = 0; i < m; i++)
		{
			int height = down[i] - up[i] -1;
			int area = height * nodes[i][j].leftCount;
			if(max_area < area)
			{
				max_area = area;
				max_up = up[i] + 1;
				max_down = down[i];
				max_left = j - nodes[i][j].leftCount + 1;
				max_right = j;
			}
		}
	}
	
	int min_x = min_room.x + max_left * grid_spacing_as_int + half_grid_spacing_as_int;
	int min_y = min_room.y + max_up * grid_spacing_as_int + half_grid_spacing_as_int;
	int max_x = min_room.x + max_right * grid_spacing_as_int + half_grid_spacing_as_int;
	int max_y = min_room.y + max_down * grid_spacing_as_int;



	//transform the calculated path back to the originally rotated map
	std::vector<cv::Point2f> fov_poses;
	std::vector<cv::Point2f> fov_coverage_path;

	fov_coverage_path.push_back(cv::Point2f(min_x, min_y));
	fov_coverage_path.push_back(cv::Point2f(max_x, min_y));
	fov_coverage_path.push_back(cv::Point2f(max_x, max_y));
	fov_coverage_path.push_back(cv::Point2f(min_x, max_y));
	fov_coverage_path.push_back(cv::Point2f(min_x, min_y));

	image_rotation.transformPathBackToOriginalRotation(fov_coverage_path, fov_poses, R);

	Rect_points.clear();
	Rect_points.insert(Rect_points.end(), fov_poses.begin(), fov_poses.end());

}

bool InnerRect::completeCellTest(const cv::Mat& image, cv::Point& cell_center, const int cell_size)
{
	const int x = cell_center.x;
	const int y = cell_center.y;
	if (image.at<unsigned char>(y,x)==255)
	{
		// just take cell center if accessible
		return true;
	}
	else
	{
		const uint16_t half_cell_size = (uint16_t)cell_size/(uint16_t)2;		// just ensure that the rounding is always reproducible
		const bool even_grid_size = ((cell_size%2)==0);

		// check whether there are accessible pixels within the cell
		const int upper_bound = even_grid_size==true ? half_cell_size-1 : half_cell_size;	// adapt the neighborhood accordingly for even and odd grid sizes
		cv::Mat cell_pixels = cv::Mat::zeros(cell_size, cell_size, CV_8UC1);
		int accessible_pixels = 0;
		for (int dy=-half_cell_size; dy<=upper_bound; ++dy)
		{
			for (int dx=-half_cell_size; dx<=upper_bound; ++dx)
			{
				const int nx = x+dx;
				const int ny = y+dy;
				if (nx<0 || nx>=image.cols || ny<0 || ny>=image.rows)
					continue;
				if (image.at<unsigned char>(ny,nx)==255)
				{
					++accessible_pixels;
					cell_pixels.at<unsigned char>(half_cell_size+dy, half_cell_size+dx) = 255;
				}
			}
		}
		// if there are accessible pixels within the cell, find their center and use this as cell center
		if (accessible_pixels>0)
		{
			return true;
			// use distance transform to find the pixels with maximum distance to obstacles, take from the maximum distance pixels the one
			// closest to the original cell center
			cv::Mat distances;
			cv::distanceTransform(cell_pixels, distances, CV_DIST_L2, 5);
			double max_distance = 0.;
			cv::minMaxLoc(distances, 0, &max_distance, 0, &cell_center);
			cell_center.x += x-half_cell_size;
			cell_center.y += y-half_cell_size;
			// if there are multiple candidates with same max distance, take the one closest to the center
			double min_squared_center_distance = (x-cell_center.x)*(x-cell_center.x) + (y-cell_center.y)*(y-cell_center.y);
			for (int v=0; v<distances.rows; ++v)
			{
				for (int u=0; u<distances.cols; ++u)
				{
					if ((double)distances.at<float>(v,u)==max_distance)
					{
						const double squared_center_distance = (u-half_cell_size)*(u-half_cell_size)+(v-half_cell_size)*(v-half_cell_size);
						if (squared_center_distance < min_squared_center_distance)
						{
							cell_center = cv::Point(x-half_cell_size+u, y-half_cell_size+v);
							min_squared_center_distance = squared_center_distance;
						}
					}
				}
			}
			
			return true;
		}
	}
	return false;
}
