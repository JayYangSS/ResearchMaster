
//Initialize all the PGR cameras and use the transform the captured image format into OpenCV format


#include "stdafx.h"

using namespace FlyCapture2;
using namespace std;
using namespace cv;

void PrintBuildInfo();
void PrintCameraInfo( CameraInfo* pCamInfo );
void PrintError( Error error );
IplImage* ConvertImageToOpenCV(Image* pImage);