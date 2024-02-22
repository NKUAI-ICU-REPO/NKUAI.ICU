#include <vector>
#include <opencv2/opencv.hpp>
#include "features2d_sift.hpp"


using namespace std;
using namespace cv;
#ifdef SIFT_TEST
int main(int argc, char** argv) {



	// Read input image
	Mat image1 = imread("./church01.jpg", 0);
	Mat imageColor1 = imread("./church01.jpg", 1);
	//Mat image2 = imread("./church02.jpg", 0);
	//Mat imageColor2 = imread("./church02.jpg", 1);

	Mat outImg, outImg1, outImg2;

	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;

	my_SIFT mySift = my_SIFT();
	mySift.myDetect(image1, keypoints1, descriptors1);
	cout << "number of keypoints: " << keypoints1.size() <<endl ;
	namedWindow("Features");
	//mySift.myDrawFeatures(imageColor1, keypoints1);
	//imshow("Features", imageColor1);
	drawKeypointsCV(imageColor1, keypoints1, outImg1, Scalar(255, 255, 255),(int) DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imshow("Features", outImg1);	

	// ÓÃÓÚÌØÕ÷Æ¥Åä
	//mySift.myDetect(image2, keypoints2, descriptors2);
	//mySift.myDrawFeatures(imageColor2, keypoints2);
	//cout << "number of keypoints: " << keypoints2.size() << endl;
	//namedWindow("Features2");
	//imshow("Features2", imageColor2);

	//mySift.matchAndDraw(imageColor1, keypoints1,
	//	imageColor2, keypoints2,
	//	outImg, outImg1, outImg2,
	//	true);

	//namedWindow("Match");
	//imshow("Match", outImg);

	waitKey();
	return 0;
}
#endif