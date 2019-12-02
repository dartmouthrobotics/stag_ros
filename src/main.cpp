#include "opencv2/opencv.hpp"
#include "Stag.h"


int main()
{
	cv::Mat image = cv::imread("/home/sam/my_photo-5.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	Stag stag(15, 7, true);

	stag.detectMarkers(image);
	stag.logResults("log/");

    return 0;
}
