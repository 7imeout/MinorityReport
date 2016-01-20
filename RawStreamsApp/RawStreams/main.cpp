#include <iostream>
#include <pxcsensemanager.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

enum RequestedFormat {
	RGB24 = 0,
	RGB32 = 1,
	DEPTH = 2
};

static const int w = 640;
static const int h = 480;

void convertPXCImageToOpenCVMat(PXCImage * inImage, cv::Mat& outImg, RequestedFormat format) {
	if (inImage) {
		PXCImage::ImageInfo imgInfo = inImage->QueryInfo();
		PXCImage::ImageData imgData;

		switch (format) {
		case RequestedFormat::RGB24:
			if (inImage->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &imgData) == PXC_STATUS_NO_ERROR) {
				outImg.create(imgInfo.height, imgInfo.width, CV_8UC3);
				{
					unsigned char *pS = (unsigned char *)imgData.planes[0];
					unsigned char *pD = (unsigned char *)outImg.data;
					unsigned char *pDEnd = pD + (imgInfo.height * imgInfo.width * 3);

					while (pD < pDEnd) {
						*pD++ = *pS++;
						*pD++ = *pS++;
						*pD++ = *pS++;
					}
				}
				inImage->ReleaseAccess(&imgData);
			}
			else {
				std::cout << "Encountered error while trying to acquire access for supposed PXCImage::PIXEL_FORMAT_RGB24 image\n";
			}
			break;
		case RequestedFormat::RGB32:
			if (inImage->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &imgData) == PXC_STATUS_NO_ERROR) {
				outImg.create(imgInfo.height, imgInfo.width, CV_8UC3);
				{
					unsigned char *pS = (unsigned char *)imgData.planes[0];
					unsigned char *pD = (unsigned char *)outImg.data;
					unsigned char *pDEnd = pD + (imgInfo.height * imgInfo.width * 4);

					while (pD < pDEnd) {
						if (*(pS + 3) >= 128) {
							*pD++ = *pS++;
							*pD++ = *pS++;
							*pD++ = *pS++;
							pS++;
						}
						else {
							pS += 4;
						}
					}
				}
				inImage->ReleaseAccess(&imgData);
			}
			else {
				std::cout << "Encountered error while trying to acquire access for supposed PXCImage::PIXEL_FORMAT_RGB32 image\n";
			}
			break;
		case RequestedFormat::DEPTH:
			if (inImage->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &imgData) == PXC_STATUS_NO_ERROR) {
				outImg.create(imgInfo.height, imgInfo.width, CV_16UC1);
				{
					unsigned short *pS = (unsigned short *)imgData.planes[0];
					unsigned short *pD = (unsigned short *)outImg.data;
					unsigned short *pDEnd = pD + (imgInfo.height * imgInfo.width);

					while (pD < pDEnd) {
						*pD = *pS * 32;
						*pS++;
						*pD++;
					}
				}
				inImage->ReleaseAccess(&imgData);
			}
			else {
				std::cout << "Encountered error while trying to acquire access for supposed PXCImage::PIXEL_FORMAT_DEPTH image\n";
			}
			break;
		}
	} else {
		std::cout << "Error: input image is NULL while trying to perform PXCImage->OpenCV conversion\n";
	}
}

int main() {
	PXCSenseManager *senseManager = PXCSenseManager::CreateInstance();
	if (!senseManager) {
		std::cout << "Could not create PXCSenseManager\n";
		return -1;
	}

	senseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, w, h);
	senseManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, w, h);

	char * color_window = "Color Stream";
	char * depth_window = "Depth Stream";
	cv::namedWindow(color_window, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(depth_window, cv::WINDOW_AUTOSIZE);

	PXCCapture::Sample *sample;
	PXCImage *colorImage, *depthImage;
	cv::Mat colorMat, depthMat;
	int key;

	// this has to come after all the enabling above
	if (senseManager->Init() < PXC_STATUS_NO_ERROR) {
		std::cout << "Could not initialize PXCSenseManager\n";
		return -1;
	}
	
	while (senseManager->AcquireFrame(true) >= PXC_STATUS_NO_ERROR) { 
		sample = senseManager->QuerySample();
		colorImage = sample->color;
		depthImage = sample->depth;

		convertPXCImageToOpenCVMat(colorImage, colorMat, RequestedFormat::RGB24);
		convertPXCImageToOpenCVMat(depthImage, depthMat, RequestedFormat::DEPTH);

		cv::imshow(color_window, colorMat);
		cv::imshow(depth_window, depthMat);
		key = cv::waitKey(10);
		if (key == 27) {
			break;
		}
		senseManager->ReleaseFrame();
	}
	senseManager->Release();
	return 0;
}
