/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef __OPENCV_NONFREE_FEATURES_2D_HPP__
#define __OPENCV_NONFREE_FEATURES_2D_HPP__


#include <vector>
#include <opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#ifdef __cplusplus

using namespace std;
using namespace cv;

/*!
 SIFT implementation.

 The class implements SIFT algorithm by D. Lowe.
*/
class  my_SIFT : public Feature2D
{
public:
    CV_WRAP explicit my_SIFT( int nfeatures=0, int nOctaveLayers=3,
          double contrastThreshold=0.04, double edgeThreshold=10,
          double sigma=1.6);

    //! returns the descriptor size in floats (128)
    CV_WRAP int descriptorSize() const;

    //! returns the descriptor type
    CV_WRAP int descriptorType() const;


    ////! finds the keypoints using SIFT algorithm
    //void operator()(InputArray img, InputArray mask,
    //                vector<KeyPoint>& keypoints) const;
    ////! finds the keypoints and computes descriptors for them using SIFT algorithm.
    ////! Optionally it can compute descriptors for the user-provided keypoints
    //void operator()(InputArray img, InputArray mask,
    //                vector<KeyPoint>& keypoints,
    //                OutputArray descriptors,
    //                bool useProvidedKeypoints=false) const;

	// 改写运算符重载
	void myDetect(InputArray image, 
		vector<KeyPoint>& keypoints, 
		OutputArray descriptors,
		bool useProvidedKeypoints = false) const;

    void buildGaussianPyramid( const Mat& base, vector<Mat>& pyr, int nOctaves ) const;
    void buildDoGPyramid( const vector<Mat>& pyr, vector<Mat>& dogpyr ) const;
    void findScaleSpaceExtrema( const vector<Mat>& gauss_pyr, const vector<Mat>& dog_pyr,
                                vector<KeyPoint>& keypoints ) const;

	void myDrawFeatures(Mat & image, const vector<KeyPoint>& keypoints) const;
	void matchAndDraw(const Mat& img1, const vector<KeyPoint>& keypoints1,
		const Mat& img2, const vector<KeyPoint>& keypoints2,
		Mat& outImg, Mat& outImg1, Mat& outImg2,
		bool ifDrawPoints) const;

    CV_PROP_RW int nfeatures;
    CV_PROP_RW int nOctaveLayers;
    CV_PROP_RW double contrastThreshold;
    CV_PROP_RW double edgeThreshold;
    CV_PROP_RW double sigma;
};

typedef my_SIFT SiftFeatureDetector;
typedef my_SIFT SiftDescriptorExtractor;


#endif /* __cplusplus */

#endif

/* End of file. */


////////////////////////////////////////////
// for class testing
////////////////////////////////////////////

// 计算中使用的一些opencv3.*中没有的函数
#define EXPTAB_SCALE 6
#define EXPTAB_MASK  ((1 << EXPTAB_SCALE) - 1)
#define EXPPOLY_32F_A0 .9670371139572337719125840413672004409288e-2

static const double expTab[] = {
	1.0 * EXPPOLY_32F_A0,
	1.0108892860517004600204097905619 * EXPPOLY_32F_A0,
	1.0218971486541166782344801347833 * EXPPOLY_32F_A0,
	1.0330248790212284225001082839705 * EXPPOLY_32F_A0,
	1.0442737824274138403219664787399 * EXPPOLY_32F_A0,
	1.0556451783605571588083413251529 * EXPPOLY_32F_A0,
	1.0671404006768236181695211209928 * EXPPOLY_32F_A0,
	1.0787607977571197937406800374385 * EXPPOLY_32F_A0,
	1.0905077326652576592070106557607 * EXPPOLY_32F_A0,
	1.1023825833078409435564142094256 * EXPPOLY_32F_A0,
	1.1143867425958925363088129569196 * EXPPOLY_32F_A0,
	1.126521618608241899794798643787 * EXPPOLY_32F_A0,
	1.1387886347566916537038302838415 * EXPPOLY_32F_A0,
	1.151189229952982705817759635202 * EXPPOLY_32F_A0,
	1.1637248587775775138135735990922 * EXPPOLY_32F_A0,
	1.1763969916502812762846457284838 * EXPPOLY_32F_A0,
	1.1892071150027210667174999705605 * EXPPOLY_32F_A0,
	1.2021567314527031420963969574978 * EXPPOLY_32F_A0,
	1.2152473599804688781165202513388 * EXPPOLY_32F_A0,
	1.2284805361068700056940089577928 * EXPPOLY_32F_A0,
	1.2418578120734840485936774687266 * EXPPOLY_32F_A0,
	1.2553807570246910895793906574423 * EXPPOLY_32F_A0,
	1.2690509571917332225544190810323 * EXPPOLY_32F_A0,
	1.2828700160787782807266697810215 * EXPPOLY_32F_A0,
	1.2968395546510096659337541177925 * EXPPOLY_32F_A0,
	1.3109612115247643419229917863308 * EXPPOLY_32F_A0,
	1.3252366431597412946295370954987 * EXPPOLY_32F_A0,
	1.3396675240533030053600306697244 * EXPPOLY_32F_A0,
	1.3542555469368927282980147401407 * EXPPOLY_32F_A0,
	1.3690024229745906119296011329822 * EXPPOLY_32F_A0,
	1.3839098819638319548726595272652 * EXPPOLY_32F_A0,
	1.3989796725383111402095281367152 * EXPPOLY_32F_A0,
	1.4142135623730950488016887242097 * EXPPOLY_32F_A0,
	1.4296133383919700112350657782751 * EXPPOLY_32F_A0,
	1.4451808069770466200370062414717 * EXPPOLY_32F_A0,
	1.4609177941806469886513028903106 * EXPPOLY_32F_A0,
	1.476826145939499311386907480374 * EXPPOLY_32F_A0,
	1.4929077282912648492006435314867 * EXPPOLY_32F_A0,
	1.5091644275934227397660195510332 * EXPPOLY_32F_A0,
	1.5255981507445383068512536895169 * EXPPOLY_32F_A0,
	1.5422108254079408236122918620907 * EXPPOLY_32F_A0,
	1.5590044002378369670337280894749 * EXPPOLY_32F_A0,
	1.5759808451078864864552701601819 * EXPPOLY_32F_A0,
	1.5931421513422668979372486431191 * EXPPOLY_32F_A0,
	1.6104903319492543081795206673574 * EXPPOLY_32F_A0,
	1.628027421857347766848218522014 * EXPPOLY_32F_A0,
	1.6457554781539648445187567247258 * EXPPOLY_32F_A0,
	1.6636765803267364350463364569764 * EXPPOLY_32F_A0,
	1.6817928305074290860622509524664 * EXPPOLY_32F_A0,
	1.7001063537185234695013625734975 * EXPPOLY_32F_A0,
	1.7186192981224779156293443764563 * EXPPOLY_32F_A0,
	1.7373338352737062489942020818722 * EXPPOLY_32F_A0,
	1.7562521603732994831121606193753 * EXPPOLY_32F_A0,
	1.7753764925265212525505592001993 * EXPPOLY_32F_A0,
	1.7947090750031071864277032421278 * EXPPOLY_32F_A0,
	1.8142521755003987562498346003623 * EXPPOLY_32F_A0,
	1.8340080864093424634870831895883 * EXPPOLY_32F_A0,
	1.8539791250833855683924530703377 * EXPPOLY_32F_A0,
	1.8741676341102999013299989499544 * EXPPOLY_32F_A0,
	1.8945759815869656413402186534269 * EXPPOLY_32F_A0,
	1.9152065613971472938726112702958 * EXPPOLY_32F_A0,
	1.9360617934922944505980559045667 * EXPPOLY_32F_A0,
	1.9571441241754002690183222516269 * EXPPOLY_32F_A0,
	1.9784560263879509682582499181312 * EXPPOLY_32F_A0,
};

static const double exp_prescale = 1.4426950408889634073599246810019 * (1 << EXPTAB_SCALE);
static const double exp_postscale = 1. / (1 << EXPTAB_SCALE);
static const double exp_max_val = 3000.*(1 << EXPTAB_SCALE); // log10(DBL_MAX) < 3000

static void Exp_32f(const float *_x, float *y, int n)
{
	static const float
		A4 = (float)(1.000000000000002438532970795181890933776 / EXPPOLY_32F_A0),
		A3 = (float)(.6931471805521448196800669615864773144641 / EXPPOLY_32F_A0),
		A2 = (float)(.2402265109513301490103372422686535526573 / EXPPOLY_32F_A0),
		A1 = (float)(.5550339366753125211915322047004666939128e-1 / EXPPOLY_32F_A0);

#undef EXPPOLY
#define EXPPOLY(x)  \
    (((((x) + A1)*(x) + A2)*(x) + A3)*(x) + A4)

	int i = 0;
	const Cv32suf* x = (const Cv32suf*)_x;
	Cv32suf buf[4];

	for (; i <= n - 4; i += 4)
	{
		double x0 = x[i].f * exp_prescale;
		double x1 = x[i + 1].f * exp_prescale;
		double x2 = x[i + 2].f * exp_prescale;
		double x3 = x[i + 3].f * exp_prescale;
		int val0, val1, val2, val3, t;

		if (((x[i].i >> 23) & 255) > 127 + 10)
			x0 = x[i].i < 0 ? -exp_max_val : exp_max_val;

		if (((x[i + 1].i >> 23) & 255) > 127 + 10)
			x1 = x[i + 1].i < 0 ? -exp_max_val : exp_max_val;

		if (((x[i + 2].i >> 23) & 255) > 127 + 10)
			x2 = x[i + 2].i < 0 ? -exp_max_val : exp_max_val;

		if (((x[i + 3].i >> 23) & 255) > 127 + 10)
			x3 = x[i + 3].i < 0 ? -exp_max_val : exp_max_val;

		val0 = cvRound(x0);
		val1 = cvRound(x1);
		val2 = cvRound(x2);
		val3 = cvRound(x3);

		x0 = (x0 - val0)*exp_postscale;
		x1 = (x1 - val1)*exp_postscale;
		x2 = (x2 - val2)*exp_postscale;
		x3 = (x3 - val3)*exp_postscale;

		t = (val0 >> EXPTAB_SCALE) + 127;
		t = !(t & ~255) ? t : t < 0 ? 0 : 255;
		buf[0].i = t << 23;

		t = (val1 >> EXPTAB_SCALE) + 127;
		t = !(t & ~255) ? t : t < 0 ? 0 : 255;
		buf[1].i = t << 23;

		t = (val2 >> EXPTAB_SCALE) + 127;
		t = !(t & ~255) ? t : t < 0 ? 0 : 255;
		buf[2].i = t << 23;

		t = (val3 >> EXPTAB_SCALE) + 127;
		t = !(t & ~255) ? t : t < 0 ? 0 : 255;
		buf[3].i = t << 23;

		x0 = buf[0].f * expTab[val0 & EXPTAB_MASK] * EXPPOLY(x0);
		x1 = buf[1].f * expTab[val1 & EXPTAB_MASK] * EXPPOLY(x1);

		y[i] = (float)x0;
		y[i + 1] = (float)x1;

		x2 = buf[2].f * expTab[val2 & EXPTAB_MASK] * EXPPOLY(x2);
		x3 = buf[3].f * expTab[val3 & EXPTAB_MASK] * EXPPOLY(x3);

		y[i + 2] = (float)x2;
		y[i + 3] = (float)x3;
	}

	for (; i < n; i++)
	{
		double x0 = x[i].f * exp_prescale;
		int val0, t;

		if (((x[i].i >> 23) & 255) > 127 + 10)
			x0 = x[i].i < 0 ? -exp_max_val : exp_max_val;

		val0 = cvRound(x0);
		t = (val0 >> EXPTAB_SCALE) + 127;
		t = !(t & ~255) ? t : t < 0 ? 0 : 255;

		buf[0].i = t << 23;
		x0 = (x0 - val0)*exp_postscale;

		y[i] = (float)(buf[0].f * expTab[val0 & EXPTAB_MASK] * EXPPOLY(x0));
	}
}


static const float atan2_p1 = 0.9997878412794807f*(float)(180 / CV_PI);
static const float atan2_p3 = -0.3258083974640975f*(float)(180 / CV_PI);
static const float atan2_p5 = 0.1555786518463281f*(float)(180 / CV_PI);
static const float atan2_p7 = -0.04432655554792128f*(float)(180 / CV_PI);

static void FastAtan2_32f(const float *Y, const float *X, float *angle, int len, bool angleInDegrees = true)
{
	int i = 0;
	float scale = angleInDegrees ? 1 : (float)(CV_PI / 180);

	for (; i < len; i++)
	{
		float x = X[i], y = Y[i];
		float ax = std::abs(x), ay = std::abs(y);
		float a, c, c2;
		if (ax >= ay)
		{
			c = ay / (ax + (float)DBL_EPSILON);
			c2 = c*c;
			a = (((atan2_p7*c2 + atan2_p5)*c2 + atan2_p3)*c2 + atan2_p1)*c;
		}
		else
		{
			c = ax / (ay + (float)DBL_EPSILON);
			c2 = c*c;
			a = 90.f - (((atan2_p7*c2 + atan2_p5)*c2 + atan2_p3)*c2 + atan2_p1)*c;
		}
		if (x < 0)
			a = 180.f - a;
		if (y < 0)
			a = 360.f - a;
		angle[i] = (float)(a*scale);
	}
}

static void Magnitude_32f(const float* x, const float* y, float* mag, int len)
{
	int i = 0;
	for (; i < len; i++)
	{
		float x0 = x[i], y0 = y[i];
		mag[i] = std::sqrt(x0*x0 + y0*y0);
	}
}


const int draw_shift_bits = 4;
const int draw_multiplier = 1 << draw_shift_bits;

/*
* 绘图函数 Functions to draw keypoints.
*/
static void _drawKeypointCV(Mat& img, const KeyPoint& p, const Scalar& color, int flags)
{
	CV_Assert(!img.empty());
	Point center(cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier));

	if (flags &(int)DrawMatchesFlags::DRAW_RICH_KEYPOINTS)
	{
		int radius = cvRound(p.size / 2 * draw_multiplier); // KeyPoint::size is a diameter

															// draw the circles around keypoints with the keypoints size
		circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);

		// draw orientation of the keypoint, if it is applicable
		if (p.angle != -1)
		{
			float srcAngleRad = p.angle*(float)CV_PI / 180.f;
			Point orient(cvRound(cos(srcAngleRad)*radius),
				cvRound(sin(srcAngleRad)*radius)
				);
			line(img, center, center + orient, color, 1, CV_AA, draw_shift_bits);
		}
#if 0
		else
		{
			// draw center with R=1
			int radius = 1 * draw_multiplier;
			circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);
		}
#endif
	}
	else
	{
		// draw center with R=3
		int radius = 3 * draw_multiplier;
		circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);
	}
}

static void drawKeypointsCV(const Mat& image, const vector<KeyPoint>& keypoints, Mat& outImage,
	const Scalar& _color, int flags)
{
	if (!(flags & (int)DrawMatchesFlags::DRAW_OVER_OUTIMG))
	{
		if (image.type() == CV_8UC3)
		{
			image.copyTo(outImage);
		}
		else if (image.type() == CV_8UC1)
		{
			cvtColor(image, outImage, COLOR_GRAY2BGR);
		}
		else
		{
			CV_Error(Error::StsBadArg, "Incorrect type of input image.\n");
		}
	}

	RNG& rng = theRNG();
	bool isRandColor = _color == Scalar::all(-1);

	CV_Assert(!outImage.empty());
	vector<KeyPoint>::const_iterator it = keypoints.begin(),
		end = keypoints.end();
	for (; it != end; ++it)
	{
		Scalar color = isRandColor ? Scalar(rng(256), rng(256), rng(256)) : _color;
		_drawKeypointCV(outImage, *it, color, flags);
	}
}