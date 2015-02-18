
#include "stdafx.h"

using namespace FlyCapture2;
using namespace std;
using namespace cv;

Image colorImage;
bool bInitialized = false;

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf( 
        version, 
        "FlyCapture2 library version: %d.%d.%d.%d\n", 
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

    printf( "%s", version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf( "%s", timeStamp );
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

IplImage* ConvertImageToOpenCV(Image* pImage)
{
	IplImage* cvImage = NULL;
	bool bColor = true;
	CvSize mySize;
	mySize.height = pImage->GetRows();
	mySize.width = pImage->GetCols();

	switch ( pImage->GetPixelFormat() )
	{
		case PIXEL_FORMAT_MONO8:	 cvImage = cvCreateImageHeader(mySize, 8, 1 );
									 cvImage->depth = IPL_DEPTH_8U;
									 cvImage->nChannels = 1;
									 bColor = false;
									 break;
		case PIXEL_FORMAT_411YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_422YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_444YUV8:   cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_RGB8:      cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_MONO16:    cvImage = cvCreateImageHeader(mySize, 16, 1 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 1;
									 bColor = false;
                                     break;
		case PIXEL_FORMAT_RGB16:     cvImage = cvCreateImageHeader(mySize, 16, 3 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_S_MONO16:  cvImage = cvCreateImageHeader(mySize, 16, 1 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 1;
									 bColor = false;
                                     break;
		case PIXEL_FORMAT_S_RGB16:   cvImage = cvCreateImageHeader(mySize, 16, 3 );
                                     cvImage->depth = IPL_DEPTH_16U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_RAW8:      cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_RAW16:     cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_MONO12:    printf("Not supported by OpenCV");
									 bColor = false;
                                     break;
		case PIXEL_FORMAT_RAW12:	 printf("Not supported by OpenCV");
									 break;
		case PIXEL_FORMAT_BGR:       cvImage = cvCreateImageHeader(mySize, 8, 3 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 3;
                                     break;
		case PIXEL_FORMAT_BGRU:      cvImage = cvCreateImageHeader(mySize, 8, 4 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 4;
                                     break;
		case PIXEL_FORMAT_RGBU:      cvImage = cvCreateImageHeader(mySize, 8, 4 );
                                     cvImage->depth = IPL_DEPTH_8U;
                                     cvImage->nChannels = 4;
                                     break;
		default: printf("Some error occured...\n");
				 return NULL;
	}

	if(bColor)
	{
		if(!bInitialized)
		{
			colorImage.SetData(new unsigned char[pImage->GetCols() * pImage->GetRows()*3], pImage->GetCols() * pImage->GetRows()*3);
			bInitialized = true;
		}
		     
		pImage->Convert(PIXEL_FORMAT_BGR, &colorImage); //needs to be as BGR to be saved
		
	    cvImage->width = colorImage.GetCols();
		cvImage->height = colorImage.GetRows();
		cvImage->widthStep = colorImage.GetStride();

	    cvImage->origin = 0; //interleaved color channels
	
		cvImage->imageDataOrigin = (char*)colorImage.GetData(); //DataOrigin and Data same pointer, no ROI
	    cvImage->imageData         = (char*)(colorImage.GetData());
		cvImage->widthStep		= colorImage.GetStride();
	    cvImage->nSize = sizeof (IplImage);
	    cvImage->imageSize = cvImage->height * cvImage->widthStep;
	}
	else
	{
        cvImage->imageDataOrigin = (char*)(pImage->GetData());
        cvImage->imageData         = (char*)(pImage->GetData());
        cvImage->widthStep         = pImage->GetStride();
        cvImage->nSize             = sizeof (IplImage);
        cvImage->imageSize         = cvImage->height * cvImage->widthStep;
        
		//at this point cvImage contains a valid IplImage
     }
	return cvImage;
}
//call via: IplImage* myCVImage = SaveImageWithOpenCV(&rawImage);