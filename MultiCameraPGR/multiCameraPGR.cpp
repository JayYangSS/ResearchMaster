//=============================================================================
// Copyright ?2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: MultipleCameraEx.cpp,v 1.17 2010-02-26 01:00:50 soowei Exp $
//=============================================================================

//#define rotateImage

#include "FlyCapture2.h"
#include "cv.h"
#include "FlyCapture2GUI.h"
#include "highgui.h"
#include "cxcore.h"
#include "opencv\cv.h"
#include "opencv\cvaux.h"

#include <cstdlib>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdarg>
#include <iostream>
#include <sstream>


using namespace FlyCapture2;
using namespace cv;
using namespace std;

Image colorImage;
bool bInitialized = false;

bool running = true;
bool write_video = false;//if want to write video, change it to true
bool shoot = false; //capture one picture
bool switchCamera = false; //switch the cameras on the dispaly
bool pause = false;    //press P to pause when capturing the video

int k = 0;//count the video captured

void handleKey(char key)
{
    switch (key)
    {
    case 27:
        running = false;
        break; 
    case 'r':
    case 'R':
		if (write_video == false)
		{
			cout << "Start recording" << endl;
			write_video = true;
		}
		else 
		{
			write_video = false;
			pause = false;
			++k;
			cout << "Stop recording" << endl;
		}
		break;
    case 'c':
    case 'C':
        shoot = 1;
        cout << "Image saved!" << endl;
        break;
	case 'p':
    case 'P':
        pause = !pause;
		if (pause == false)
			cout << "Paused!" << endl;
		else
			cout << "Continued!" <<endl;
		break;

	case 's':
    case 'S':
        switchCamera = !switchCamera;
        cout << "Camera Switched!" << endl;
        break;
    }
}

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

void PrintInfo()
{
    printf(
        "\n*** Instruction ***\n"
        "Press 'c' to capture picture\n"
        "Press 'r' to start/stop recording video\n"
		"Press 'p' to pause recording\n"		
        "Press 's' to switch left and right\n"
        "Press 'esc' to exit\n"
		);
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

//旋转图像内容不变，尺寸相应变大
IplImage* rotateImage2(IplImage* src, int degree, double factor)  
{  
	//resize the src image based on the factor************
	IplImage* src_resize = NULL;		
	CvSize src_resize_size;
	src_resize_size.width = src->width * factor + 1;
	src_resize_size.height = src->height * factor + 1;
	src_resize = cvCreateImage(src_resize_size, src->depth, src->nChannels);
	cvResize( src, src_resize, 1);
	//Mat src_resize_m = Mat (src_resize);
	//namedWindow("resize test", 1);
	//imshow("resize test", src_resize_m);

	IplImage* img = src_resize;
	double angle = degree  * CV_PI / 180.; 
	double a = sin(angle), b = cos(angle); 
	int width=img->width, height=img->height;
	//旋转后的新图尺寸
	int width_rotate= int(height * fabs(a) + width * fabs(b));  
	int height_rotate=int(width * fabs(a) + height * fabs(b));  
	IplImage* img_rotate = cvCreateImage(cvSize(width_rotate, height_rotate), img->depth, img->nChannels);  
	cvZero(img_rotate);  
	//保证原图可以任意角度旋转的最小尺寸
	int tempLength = sqrt((double)width * width + (double)height *height) + 10;  
	int tempX = (tempLength + 1) / 2 - width / 2;  
	int tempY = (tempLength + 1) / 2 - height / 2;  
	IplImage* temp = cvCreateImage(cvSize(tempLength, tempLength), img->depth, img->nChannels);  
	cvZero(temp);  
	//将原图复制到临时图像tmp中心
	cvSetImageROI(temp, cvRect(tempX, tempY, width, height));  
	cvCopy(img, temp, NULL);  
	cvResetImageROI(temp);  
	//旋转数组map
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]
	// [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]
	float m[6];  
	int w = temp->width;  
	int h = temp->height;  
	m[0] = b;  
	m[1] = a;  
	m[3] = -m[1];  
	m[4] = m[0];  
	// 将旋转中心移至图像中间  
	m[2] = w * 0.5f;  
	m[5] = h * 0.5f;  
	CvMat M = cvMat(2, 3, CV_32F, m);  
	cvGetQuadrangleSubPix(temp, img_rotate, &M);  
	cvReleaseImage(&temp);  
	return img_rotate;
}  


int main(int /*argc*/, char** /*argv*/)
{
    PrintBuildInfo();

    const int k_numImages = 2;
    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    printf( "Number of cameras detected: %u\n", numCameras );

    if ( numCameras < 1 )
    {
        printf( "Insufficient number of cameras... press Enter to exit.\n" );
        getchar();
        return -1;
    }

    Camera** ppCameras = new Camera*[numCameras];

    // Connect to all detected cameras and attempt to set them to
    // a common video mode and frame rate
    for ( unsigned int i = 0; i < numCameras; i++)
    {
        ppCameras[i] = new Camera();

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex( i, &guid );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        // Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        PrintCameraInfo(&camInfo); 

        // Set all cameras to a specific mode and frame rate so they
        // can be synchronized
        error = ppCameras[i]->SetVideoModeAndFrameRate( 
            VIDEOMODE_1280x960RGB, 
            FRAMERATE_30 );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            printf( 
                "Error starting cameras. \n"
                "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
                "If your camera does not support this mode, please edit the source code and recompile the application. \n"
                "Press Enter to exit. \n");
            getchar();
            return -1;
        }
    
	}
    
//  error = Camera::StartSyncCapture( numCameras, (const Camera**)ppCameras );
    
	for ( int i=0; i<numCameras; i++ )
	{

		error = ppCameras[i]->StartCapture();

		if (error != PGRERROR_OK)
		{
			PrintError( error );
			printf( 
				"Error starting cameras. \n"
				"If the Error is because shared.cpp, please remove all the pluged-in on USB3.0.\n"
				"Only plug in the cameras, and setup the driver again using DriverControlGUI.exe"
				"Press Enter to exit. \n");
			getchar();
			return -1;
		}
	}
	
	PrintInfo();

	//creat the video from captures images*************
	VideoWriter video_writer_ls;
	VideoWriter video_writer_rl;  	
	bool make_gray = false;
	char videoname[512];
	double video_fps = 30.0;
	char temp = rand()*100;
	
	Mat img, img_to_show, img_temp, img_video;
	Mat img_save, img_save_show;

	int j = 0;
    while(running)
	{
		// Display the timestamps for all cameras to show that the image
        // capture is synchronized for each image
        for ( unsigned int i = 0; i < numCameras; i++ )
        {
			Image image;
            error = ppCameras[i]->RetrieveBuffer( &image );
            if (error != PGRERROR_OK)
            {
                PrintError( error );
                return -1;
            }
			// Since this application saves images in the current folder
			// we must ensure that we have permission to write to this folder.
			// If we do not have permission, fail right away.

			//FILE* tempFile = fopen("test.txt", "w+");
			//if (tempFile == NULL)
			//{
			//	printf("Failed to create file in current folder.  Please check permissions.\n");
			//	return -1;
			//}
			//fclose(tempFile);
			//remove("test.txt");

			if(shoot)
			{
			// Create a unique filename	
				char filename[512];									// Save the image. If a file format is not passed in, then the file
				sprintf( filename, "Calib_Camera-%d-%d.jpg", i, j );// extension is parsed to attempt to determine the file format.
	//changed
				error = image.Save( filename );				//save captured image into "filename"
				if (error != PGRERROR_OK)
				{
					PrintError( error );
					return -1;
				}  
				printf( "Grabbed image for camera-%d-%d\n", i, j );

				img_save = Mat (ConvertImageToOpenCV(&image));	// convert captured image into OPENCV format

				resize(img_save, img_save_show, Size(640, 480));

				if(!img_save.data) //test if the image is transmitted in
				{
					printf("调入图片失败则退出");
					return -1; //调入图片失败则退出
				}
				if(i == (numCameras-1))
				{
					++j;
					shoot=false;
				}
			}

//convert image from PGR to CV format and rotate it**********************
			img = Mat (ConvertImageToOpenCV(&image));
#ifdef rotateImage

			rotate the captured image************
			double scale = 0.5;
			int degree = 90;
			IplImage * psrc = ConvertImageToOpenCV(&image);
			Mat img_rotate = Mat (rotateImage2( psrc, degree, scale));
			
     		img_rotate.copyTo(img);
			img_to_show = img;
#endif
			//creat AVI format from captured image**********************
			
			//write video into avi
			if(!write_video)
			{
				if(video_writer_ls.isOpened())
					video_writer_ls.release();
				if(video_writer_rl.isOpened())
					video_writer_rl.release();
			}
			if(!pause && write_video)
			{
				char video_dst_ls[512],video_dst_rl[512];
				sprintf_s (video_dst_ls , "Video_Capture_%d_Right.avi",k);
				sprintf_s (video_dst_rl , "Video_Capture_%d_Left.avi",k);

				img.copyTo(img_temp);
				if(!video_writer_ls.isOpened())
				{
					video_writer_ls.open(video_dst_ls, CV_FOURCC('x','v','i','d'), video_fps, img_temp.size(), true);
					if (!video_writer_ls.isOpened())
					{
						printf("can't create video writer ls");
						getchar();
						return -1;
					}
               
				}
				if(!video_writer_rl.isOpened())
				{
					video_writer_rl.open(video_dst_rl, CV_FOURCC('x','v','i','d'), video_fps, img_temp.size(), true);
					if (!video_writer_rl.isOpened())
					{
						printf("can't create video writer rl");
						getchar();
						return -1;
					}
				}
				if (make_gray)	cvtColor(img_temp, img_video, CV_GRAY2BGR);
				else			cvtColor(img_temp, img_video, CV_BGRA2BGR);

				if( i == 0 )
					if( switchCamera == false )
						video_writer_ls << img_video;
					else
						video_writer_rl << img_video;
				else
					if( switchCamera == true )
						video_writer_ls << img_video;
					else
						video_writer_rl << img_video;
			}
							
			//create AVI format over**********************************************************
			Rect ROI;
			ROI.width = 200;
			ROI.height = 200;
			ROI.x = img.cols/2 - ROI.width/2;
			ROI.y = img.rows/2 - ROI.height/2;

		//	Mat img_to_focus(ROI.size(),img.type());
			Mat img_to_focus = img(ROI);

			resize(img, img_to_show, Size(640, 480));
			if(!img.data) 
			{
				printf("调入图片失败则退出");
				return -1; 
			}
							
			string windowname;
			string focusname;
			if (i == 0)
				if( switchCamera == false )
				{
					windowname = "Right";
					focusname = "Right-focus";
				}else{
					windowname = "Left";
					focusname = "Left-focus";
				}
			else
				if( switchCamera == true )
				{
					windowname = "Right";
					focusname = "Right-focus";
				}else{
					windowname = "Left";
					focusname = "Left-focus";
				}

			namedWindow(windowname, CV_WINDOW_AUTOSIZE);    
			imshow(windowname, img_to_show);

			namedWindow(focusname, CV_WINDOW_AUTOSIZE);    
			imshow(focusname, img_to_focus);

        }
		handleKey((char)waitKey(3));
    }

    for ( unsigned int i = 0; i < numCameras; i++ )
    {
        ppCameras[i]->StopCapture();
        ppCameras[i]->Disconnect();
        delete ppCameras[i];
    }

    delete [] ppCameras;

    printf( "Done! Press Enter to exit...\n" );
    getchar();

	return 0;
}