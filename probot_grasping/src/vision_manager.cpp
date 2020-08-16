/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "probot_grasping/vision_manager.h"

VisionManager::VisionManager(float length, float breadth)
{
	this->table_length = length;
	this->table_breadth = breadth;
}

void VisionManager::get2DLocation(cv::Mat img, float &x, float &y)
{
	this->curr_img = img; //将图像保存到这个实例的成员中
	 //获得图像中心坐标
	img_centre_x_ = img.rows / 2;
	img_centre_y_ = img.cols / 2;

	cv::Rect tablePos; //定义一个矩形

	detectTable(tablePos); //找到桌面并计算每mm有多少像素
	detect2DObject(x, y, tablePos); //找到物体中心坐标
	convertToMM(x, y); // 转化到世界坐标系
}

void VisionManager::detectTable(cv::Rect &tablePos)
{
	// Extract Table from the image and assign values to pixel_per_mm fields
	cv::Mat BGR[3];
	cv::Mat image = curr_img.clone();//定义一个图像副本
	split(image, BGR); // 将图像切分成三份，按照BGR 颜色
	cv::Mat gray_image_red = BGR[2]; //red color 
	cv::Mat gray_image_green = BGR[1]; // green color
	cv::Mat denoiseImage; //定义一个去噪图像 
	cv::medianBlur(gray_image_red, denoiseImage, 3); //中值滤波 

	// Threshold the Image
	// 将图像二值化，通过red green颜色
	cv::Mat binaryImage = denoiseImage; 
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++) //遍历去噪后的红色分量图
		{
			int editValue = binaryImage.at<uchar>(i, j); //此像素点的红色分量
			int editValue2 = gray_image_green.at<uchar>(i, j);//绿色分量

			//将符合条件（接近黑色）的像素点变为255，其余的变为0
			if ((editValue >= 0) && (editValue < 20) && (editValue2 >= 0) && (editValue2 < 20))
			{ // check whether value is within range.
				binaryImage.at<uchar>(i, j) = 255;
			}
			else
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
		}
	}
	cv::dilate(binaryImage, binaryImage, cv::Mat()); //膨胀，

	// Get the centroid of the of the blob
	// 找到桌面的中心
	std::vector<cv::Point> nonZeroPoints; //
	cv::findNonZero(binaryImage, nonZeroPoints); //存储图像中的非零元素的索引到nonZeroPoints中
	cv::Rect bbox = cv::boundingRect(nonZeroPoints); //计算这些非零元素的最小外接矩形
	cv::Point pt;
	pt.x = bbox.x + bbox.width / 2; //确定中心坐标
	pt.y = bbox.y + bbox.height / 2;
	//函数cvCircle绘制或填充一个给定圆心和半径的圆
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), -1, 8); 

	// Update pixels_per_mm fields
	pixels_permm_y = bbox.height / table_length; //求得y轴每mm的像素个数
	pixels_permm_x = bbox.width  / table_breadth;//求得x轴每mm的像素个数

    tablePos = bbox;//返回所求的矩形框

	// Test the conversion values
	std::cout << "Pixels in y" << pixels_permm_y << std::endl;
	std::cout << "Pixels in x" << pixels_permm_x << std::endl;

	// Draw Contours - For Debugging
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	//查找轮廓并绘制
	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++) //遍历所有轮廓并绘制
	{
		cv::Scalar color = cv::Scalar(255, 0, 0); //轮廓的颜色
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}
	// 显示
	// cv::namedWindow("Table Detection", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Table Detection", image);
	// cv::waitKey(100);
}

void VisionManager::detect2DObject(float &pixel_x, float &pixel_y, cv::Rect &tablePos)
{
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
	cv::Mat image, gray_image_green;
	cv::Mat BGR[3];
	image = curr_img.clone();
	cv::split(image, BGR); //将图像切分为包含单一颜色分量的图

	gray_image_green = BGR[1];//绿色分量图

	// Denoise the Image
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_green, denoiseImage, 3); //中值滤波去噪

	// Threshold the Image
	//将图像二值化，通过绿色分量
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++) //遍历
		{
			if((j<tablePos.x+3) || j>(tablePos.x+tablePos.width-3) || (i<tablePos.y+3) || i>(tablePos.y + tablePos.height-3))
			{//桌子外的值变为0,不考虑
				binaryImage.at<uchar>(i, j) = 0;
			}
			else
			{//在桌子范围内找到绿色分量比较大的像素点，这些点是物体的点
				int editValue = binaryImage.at<uchar>(i, j);//此像素绿色分量

				if ((editValue > 100) && (editValue <= 255))//认为是绿色，将值设为255
				{ // check whether value is within range.
					binaryImage.at<uchar>(i, j) = 255;
				}
				else//不是绿色，将值设为0
				{
					binaryImage.at<uchar>(i, j) = 0;
				}
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());//膨胀

	// Get the centroid of the of the blob
	//找到物体中心
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints); //将二值化的图的非零点的索引保存下来
	cv::Rect bbox = cv::boundingRect(nonZeroPoints); //找到这些非零点的最小外接矩形
	cv::Point pt;
	pixel_x = bbox.x + bbox.width / 2; //得到中心坐标
	pixel_y = bbox.y + bbox.height / 2;

	// Test the conversion values
	std::cout << "pixel_x" << pixel_x << std::endl;
	std::cout << "pixel_y" << pixel_y << std::endl;

	// For Drawing
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), -1, 8);//画一个圆

	// Draw Contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	//查找轮廓并绘制出来
	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}
	//显示
	// cv::namedWindow("Centre point", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Centre point", image);
	// cv::waitKey(100);
}

void VisionManager::convertToMM(float &x, float &y)
{
	// Convert from pixel to world co-ordinates in the camera frame
	x = (x - img_centre_x_) / pixels_permm_x; //物体中心-图像中心
	y = (y - img_centre_y_) / pixels_permm_y;
}

// Temporary Main Function for testing- This should go away later
// int main(int argc, char** argv ) {
// 	if ( argc != 2 )
//     {
//         printf("usage: VisionManager <Image_Path>\n");
//         return -1;
//     }

//     cv::Mat image;
//     image = cv::imread( argv[1], 1 );

//     if ( !image.data )
//     {
//         printf("No image data \n");
//         return -1;
//     }

//     float length = 0.3;
//     float breadth = 0.3;
//     float obj_x, obj_y;

//     VisionManager vm(length, breadth);
//     vm.get2DLocation(image, obj_x, obj_y);
//     std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
//     std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

//     cv::waitKey(0);
// }
