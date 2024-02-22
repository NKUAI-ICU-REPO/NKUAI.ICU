/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 8 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/features2d/features2d.hpp>
#include<opencv2/xfeatures2d/nonfree.hpp>
#include <time.h>

#include "harrisDetector.h"
//#define INTEREST
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
#ifdef INTEREST
int main()
{
	Mat image;
	vector<cv::KeyPoint>keypoints;
	// Read input image
	cv::Mat image= cv::imread("../church01.jpg",0);
	if (!image.data)
		return 0; 

    // display the image
	cv::namedWindow("original image");
	cv::imshow("original image",image);

	// detect harris corners
	cv::Mat cornerstrength;
	cv::cornerHarris(image,cornerstrength,
		          3,     // neighborhood size
					 3,     // aperture size
					 0.01); // harris parameter

   // threshold the corner strengths
	cv::Mat harriscorners;
	double threshold= 0.0001; 
	cv::threshold(cornerstrength,harriscorners,
                 threshold,255,THRESH_BINARY_INV);

    // display the corners
	cv::namedWindow("harris corner map");
	cv::imshow("harris corner map",harriscorners);

	// create harris detector instance
	HarrisDetector harris;
    // compute harris values
	harris.detect(image);
    // detect harris corners
	std::vector<Point> pts;
	harris.getCorners(pts,0.01);
	// draw harris corners
	harris.drawOnImage(image,pts);

    // display the corners
	cv::namedWindow("harris corners");
	cv::imshow("harris corners",image);

	// read input image
	image= cv::imread("../church01.jpg",0);

	// compute good features to track
	std::vector<cv::Point2f> corners;
	cv::goodFeaturesToTrack(image,corners,
		500,	// maximum number of corners to be returned
		0.01,	// quality level
		10);	// minimum allowed distance between points
	  
	// for all corners
	std::vector<cv::Point2f>::const_iterator it= corners.begin();
	while (it!=corners.end()) {

		// draw a circle at each corner location
		cv::circle(image,*it,3,cv::Scalar(255,255,255),2);
		++it;
	}

    // display the corners
	cv::namedWindow("good features to track");
	cv::imshow("good features to track",image);

	// read input image
	image= cv::imread("../church01.jpg",0);
	

	// vector of keypoints
	std::vector<cv::KeyPoint> keypoints;
	// construction of the good feature to track detector 
	//xfgoodfeaturestotrackdetector
	Ptr<GFTTDetector>gftt=GFTTDetector::create(
		500,	// maximum number of corners to be returned
		0.01,	// quality level
		10);	// minimum allowed distance between points
	// point detection using featuredetector method
	gftt->detect(image,keypoints);
	
	cv::drawKeypoints(image,		// original image
		keypoints,					// vector of keypoints
		image,						// the resulting image
		cv::Scalar(255,255,255),	// color of the points
		cv::DrawMatchesFlags::DRAW_OVER_OUTIMG); //drawing flag

    // display the corners
	cv::namedWindow("good features to track detector");
	cv::imshow("good features to track detector",image);

	// Read input image
	image= cv::imread("../1013.jpg",0);
	
	keypoints.clear();
	Ptr<FastFeatureDetector>fast=FastFeatureDetector::create(40);
	fast->detect(image,keypoints);
	
	cv::drawKeypoints(image,keypoints,image,cv::Scalar(255,255,255),cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

    // Display the corners
	cv::namedWindow("FAST Features");
	cv::imshow("FAST Features",image);

	////////////////////////////////////////////////
	// Read input image
	image= cv::imread("../t030.jpg",0);
	//cv::Mat imageColor = cv::imread("../1013.jpg",1);

	keypoints.clear();
	// Construct the SURF feature detector object

	clock_t start, finish;
	start = clock();

	Ptr<SurfFeatureDetector>surf=SurfFeatureDetector::create(2500);
	// Detect the SURF features
	surf->detect(image,keypoints);

	finish = clock();
	double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	std::cout<<"surf features: "<<keypoints.size()<<"surf time is " << duration << std::endl;

	
	cv::Mat featureImage;
	cv::drawKeypoints(image,keypoints,featureImage,cv::Scalar(255,255,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display the corners
	cv::namedWindow("SURF Features");
	cv::imshow("SURF Features",featureImage);

	// Read input image
	image= cv::imread("../t030.jpg",0);
	//imageColor = cv::imread("../1013.jpg",1);

	keypoints.clear();

	start = clock();
	// Construct the SURF feature detector object
	Ptr<SiftFeatureDetector>sift=SiftFeatureDetector::create(
		0.03,  // feature threshold
		10.);  // threshold to reduce
	           // sensitivity to lines

	// Detect the SURF features
	sift->detect(image,keypoints);
	
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	std::cout <<"sift features: "<<keypoints.size()<< "sift time is " << duration << std::endl;

	cv::drawKeypoints(image,keypoints,featureImage,cv::Scalar(255,255,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display the corners
	cv::namedWindow("SIFT Features");
	cv::imshow("SIFT Features",featureImage);

	keypoints.clear();
	image = cv::imread("../t030.jpg", 0);
	Ptr<SimpleBlobDetector>dec = SimpleBlobDetector::create();
	dec->detect(image, keypoints);
	drawKeypoints(image, keypoints, featureImage, Scalar(255, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imshow("BLOB Features", featureImage);


	keypoints.clear();
	image = imread("../t030.jpg");
	Mat gray;
	cvtColor(image, gray, COLOR_RGB2GRAY);
	medianBlur(gray, gray, 5);
	imshow("1", gray);
	vector<Vec3f>cir;
	HoughCircles(gray, cir, HOUGH_GRADIENT, 1, 20, 200, 80, 100, 600);
	for (size_t i = 0; i < cir.size(); i++) {
		Vec3i c = cir[i];
		Point center = Point(c[0], c[1]);
		circle(image, center, 1, Scalar(0, 255,0), -1, 8,0);
		int radi = c[2];
		circle(image, center, radi, Scalar(255, 0, 0), 2, 8,0);
	}
	imshow("HOUGH", image);
	
	// Read input image
	//image= cv::imread("../church01.jpg",0);

	//keypoints.clear();
	////MserFeatureDetector
	//Ptr<MSER>mser=MSER::create();
	//mser->detect(image,keypoints);
	//
	//// Draw the keypoints with scale and orientation information
	//cv::drawKeypoints(image,		// original image
	//	keypoints,					// vector of keypoints
	//	featureImage,				// the resulting image
	//	cv::Scalar(255,255,255),	// color of the points
	//	cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); //drawing flag

 //   // Display the corners
	//cv::namedWindow("MSER Features");
	//cv::imshow("MSER Features",featureImage);

	cv::waitKey();
	return 0;
}
#endif