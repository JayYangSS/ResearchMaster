


#include "stdafx.h"

#define PGRCamera
//#define SAVE_IMAGE

using namespace std;
using namespace cv;
using namespace FlyCapture2;

bool help_showed = false;

//ofstream weightVector_formated;
void extractFloat(char* dst, const char* src);
const int dimension64 = 3780;		//dimensionality		
const int dimension48 = 1980;		//dimensionality		
const int dimension16 = 108;		//dimensionality		
const int dimension24 = 360;		//dimensionality
const int dimension32 = 756;		//dimensionality
const int dimension40 = 1296;		//dimensionality
const int dimension56 = 2808;		//dimensionality	
const int dimension72 = 4896;		//dimensionality
const int dimension80 = 6156;		//dimensionality
const int dimension96 = 9108;		//dimensionality

//traffic sign dimensions
const int dimensionSign = 576;

const float signTotal = 4.2100509f;
const float signstop = 3.2251248f;
const float sign40 = 4.0159234f;
const float sign50 = 3.4225286f;
//const float sign60 = ;

const double threshold16 = 19.704114; //threshold
const double threshold24 = 12.569505; //threshold
const double threshold32 = 9.8262787; //threshold
const double threshold40 = 6.3184363; //threshold
const double threshold48 = 4.0300296; //threshold
const double threshold56 = 3.1449572; //threshold
const double threshold64 = 2.9532082; //threshold
const double threshold72 = 2.7619303; //threshold
const double threshold80 = 2.1945252; //threshold
const double threshold96 = 1.0090097; //threshold

bool KFtrack = true;				//if use KalmanFilter
bool CStrack = false;				//if use CamShift 

const int stateNum = 8;				//x,y,width,height,deta x,deta y,deta width,deta height
const int measureNum = 4;			//deta x,deta y,deta width,deta height


//使用自己的SVMVector 64*128
vector<float> getThisObjectDetector64(const char* wv)	//input the weight factor
{
	static float d_detector[dimension64+1];		//dimension of weight vector;

	string st;
	ifstream weightVector;
	int i = 0;

	weightVector.open(wv);

	while (getline(weightVector, st))
	{
		char weight[100];
		extractFloat(weight, st.c_str());
		d_detector[i] = atof(weight);
		i++;
	}

	d_detector[dimension64] = 2.9532082f; // threshold b;
										//2.9532082 # threshold b for 64*128
										//4.0300296 # threshold b for 48*96
										// used in proceeding 6.3
	//cout << detector + sizeof(detector)/sizeof(detector[0]) << endl;

    return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
}

//使用自己的SVMVector 48*96
vector<float> getThisObjectDetector48(const char* wv)	//input the weight factor
{
	static float d_detector[dimension48+1];		//dimension of weight vector;

	string st;
	ifstream weightVector;
	int i = 0;

	weightVector.open(wv);

	while (getline(weightVector, st))
	{
		char weight[100];
		extractFloat(weight, st.c_str());
		d_detector[i] = atof(weight);
		i++;
	}

	d_detector[dimension48] = 4.0300296f; // threshold b;
										//2.9532082 # threshold b for 64*128
										//4.0300296 # threshold b for 48*96
										// used in proceeding 8.9
	//cout << detector + sizeof(detector)/sizeof(detector[0]) << endl;

    return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
}

//use own trained Traffic sign descriptors
vector<float> getTrafficSignDetector(string signType)	//input the weight factor
{
	string st;
	ifstream weightVector;
	int i = 0;
	float signDetector[dimensionSign+1];		//dimension of weight vector;

	if(signType == "40")
	{
		weightVector.open("final_mo40");
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			signDetector[i] = atof(weight);
			i++;
		}
		cout << "vector 40 synchronized successfylly" << endl;
		signDetector[dimensionSign] = 3.0159234f; // threshold b;
		return vector<float>(signDetector, signDetector + sizeof(signDetector)/sizeof(signDetector[0]));

	}
	else if(signType == "50")
	{
		weightVector.open("final_mo50");
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			signDetector[i] = atof(weight);
			i++;
		}
		cout << "vector 50 synchronized successfylly" << endl;
		signDetector[dimensionSign] = 3.4225286f; // threshold b;
		return vector<float>(signDetector, signDetector + sizeof(signDetector)/sizeof(signDetector[0]));

	}
	else if(signType == "stop")
	{
		weightVector.open("final_mostop");
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			signDetector[i] = atof(weight);
			i++;
		}
		cout << "vector stop synchronized successfylly" << endl;
		signDetector[dimensionSign] = 3.2251248f; // threshold b;
		return vector<float>(signDetector, signDetector + sizeof(signDetector)/sizeof(signDetector[0]));

	}

	else if(signType == "total")
	{
		weightVector.open("final_mototal");
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			signDetector[i] = atof(weight);
			i++;
		}
		cout << "vector total synchronized successfylly" << endl;
		signDetector[dimensionSign] = 4.2100509f; // threshold b;		
		return vector<float>(signDetector, signDetector + sizeof(signDetector)/sizeof(signDetector[0]));

	}	
	else
	{
		cout << "not effective string, please check again" << endl;
		system("pause");
		throw runtime_error(string("cannot detect input traffic sign descriptor"));
	}

}

vector<float> getThisObjectDetector(int wSize)	//input the weight factor
{
	string st;
	ifstream weightVector;
	int i = 0;

	if(wSize == 16)
	{
		static float d_detector[dimension16+1];		//dimension of weight vector;
		weightVector.open("finalvector16");		
		d_detector[dimension16] = 19.704114f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}
		cout << "pedestrian vector 16 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}	
	else if(wSize == 24)
	{
		static float d_detector[dimension24+1];		//dimension of weight vector;
		weightVector.open("finalvector24");		
		d_detector[dimension24] = 12.569505f	; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 24 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 32)
	{
		static float d_detector[dimension32+1];		//dimension of weight vector;
		weightVector.open("finalvector32");		
		d_detector[dimension32] = 9.8262787f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 32 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 40)
	{
		static float d_detector[dimension40+1];		//dimension of weight vector;
		weightVector.open("finalvector40");		
		d_detector[dimension40] = 6.3184363f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 40 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 48)
	{
		static float d_detector[dimension48+1];		//dimension of weight vector;
		weightVector.open("finalvector48");		
		d_detector[dimension48] = 4.0300296f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 48 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 56)
	{
		static float d_detector[dimension56+1];		//dimension of weight vector;
		weightVector.open("finalvector56");		
		d_detector[dimension56] = 3.1449572f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 56 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 64)
	{
		static float d_detector[dimension64+1];		//dimension of weight vector;
		weightVector.open("finalvector64");	
		d_detector[dimension64] = 2.9532082f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			//cout << "d_detector[" << i << "]" <<d_detector[i] << endl;
			i++;
		}	
		
		//cout << "d_detector[" << dimension64 << "]" <<d_detector[dimension64] << endl;
		cout << "pedestrian vector 64 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 72)
	{
		static float d_detector[dimension72+1];		//dimension of weight vector;
		weightVector.open("finalvector72");		
		d_detector[dimension72] = 2.7619303f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 72 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 80)
	{
		static float d_detector[dimension80+1];		//dimension of weight vector;
		weightVector.open("finalvector80");		
		d_detector[dimension80] = 2.1945252f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 80 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}
	else if(wSize == 96)
	{
		static float d_detector[dimension96+1];		//dimension of weight vector;
		weightVector.open("finalvector96");		
		d_detector[dimension96] = 1.0090097f; // threshold b;
		while (getline(weightVector, st))
		{
			char weight[100];
			extractFloat(weight, st.c_str());
			d_detector[i] = atof(weight);
			i++;
		}	
		cout << "pedestrian vector 96 synchronized successfylly" << endl;
		return vector<float>(d_detector, d_detector + sizeof(d_detector)/sizeof(d_detector[0]));
	}	

}

// 去掉vector前面的数字和":"
void extractFloat(char* dst, const char* src)
{
	for (int i = 0; i<strlen(src); i++)
	{
		if (src[i] == ':' )
		{
			i++;
			for (int j=0; j<=strlen(src)-i; j++)
			{
				dst[j] = src[i+j];
			}
			break;
		}
	}
}

//旋转图像内容不变，尺寸相应变大
//Rotate the captured image and enlarge the corresponding img_to_show
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


class Args
{
public:
    Args();
    static Args read(int argc, char** argv);

    string srcs;
	string srcl;
    bool src_is_videos;
    bool src_is_cameras;
	bool src_is_videol;
    bool src_is_cameral;

    bool write_video;
	string dst_videos;
	string dst_videol;
	string dst_video_combine;
    double dst_video_fps;
	bool dst_name_auto;

    bool make_gray;

    bool resize_src;
    int width, height;

    double scale;
    int nlevels;
    int gr_threshold;

    double hit_threshold;
    bool hit_threshold_auto;

    int win_width;
    int win_stride_width, win_stride_height;

	bool img_rotate;
	int degree;
	double factor;

    bool gamma_corr;

	double flength_s;
	double flength_l;

	bool btracking;
	bool SIFT;
	bool TrafficSignDetection;
	int signCategory;
};


class App
{
public:
    App(const Args& s);
    void run();

    void handleKey(char key);

    void hogWorkBegin();
    void hogWorkEnd();
    string hogWorkFps() const;

    void workBegin();
    void workEnd();
    string workFps() const;

    string message() const;

private:
    App operator=(App&);

    Args args;
    bool running;

	bool use_SIFT;
    bool use_gpu;
    bool make_gray;
    double scale;
    int gr_threshold;
    int nlevels;
    double hit_threshold;
    bool gamma_corr;

    int64 hog_work_begin;
    double hog_work_fps;
	double hog_work_spf;

    int64 work_begin;
    double work_fps;
	double work_spf;
	bool btracking;		//if tracking
	bool showboth;		//show both detecting and tracking results
	bool firstframe;
	bool showMeasurementTrajectory;

	string detectionModel;
	int count;

	bool detectSign;	//if detect trafffic sign
	int category;
};

static void printHelp()
{
    cout << "Histogram of Oriented Gradients descriptor and detector sample.\n"
         << "\nUsage: hog_gpu\n"
		 << "  [--flength_s <double>] # get the short focal length\n"
		 << "  [--flength_l <double>] # get the long focal length\n"
         << "  (<images>|--videos <vide>|--cameras ) # frames source\n"
		 << "  (<imagel>|--videol <vide1>|--cameral ) # frames source\n"
		 << "  [--tracking] # bool 'btracking' will be true, begin tracking\n"
		 << "  [--SIFT <true/false>] # bool 'SIFT' will be true, begin using SIFT/SURF elimination\n"
         << "  [--make_gray <true/false>] # convert image to gray one or not\n"
		 << "  [--trafficsign <true/false>] # convert image to gray one or not\n"
         << "  [--resize_src <true/false>] # do resize of the source image or not\n"
         << "  [--width <int>] # resized image width\n"
         << "  [--height <int>] # resized image height\n"
         << "  [--hit_threshold <double>] # classifying plane distance threshold (0.0 usually)\n"
         << "  [--scale <double>] # HOG window scale factor\n"
         << "  [--nlevels <int>] # max number of HOG window scales\n"
         << "  [--win_width <int>] # width of the window (48 or 64)\n"
         << "  [--win_stride_width <int>] # distance by OX axis between neighbour wins\n"
         << "  [--win_stride_height <int>] # distance by OY axis between neighbour wins\n"
         << "  [--gr_threshold <int>] # merging similar rects constant\n"
         << "  [--gamma_correct <int>] # do gamma correction or not\n"
		 << "  [--rotate <int degree(clockwise-/anticlockwise+)>] # rotate the captured image\n"
		 << "  [--factor <double>] # shrink or enlarge the captured picture\n"
		 << "  PLZ be ware that use [--factor] to resize picture by camera and [--resize_src] to vidoes \n"
         << "  [--write_video <bool>] # write video or not\n"
         << "  [--dst_videos <path.avi>] # output video path\n"
		 << "  [--dst_videol <path.avi>] # output video path\n"
		 << "  [--dst_video_combine <path.avi>] # output video path\n"
		 << "  [--dst_video_fps <double>] # output video fps\n";

    help_showed = true;
}



int main(int argc, char** argv)
{
    
	try
    {
        if (argc < 2)
            printHelp();
        Args args = Args::read(argc, argv);
        if (help_showed)
            return -1;
        App app(args);
        app.run();
    }
    catch (const Exception& e) { return cout << "error: "  << e.what() << endl, 1; }
    catch (const exception& e) { return cout << "error: "  << e.what() << endl, 1; }
    catch(...) { return cout << "unknown exception" << endl, 1; }

	//getchar();
	//system("pause");
	//system("pause");
    return 0;
}


Args::Args()
{
    src_is_videos = false;
	src_is_videol = false;
    src_is_cameras = false;
	src_is_cameral = false;

    write_video = false;
    dst_video_fps = 30.0;
	dst_name_auto = true;

    make_gray = false;

	img_rotate = false;
	degree = 0;
	factor = 1.0;

    resize_src = false;
    width = 640;
    height = 480;

    scale = 1.05;
    nlevels = 62;
    gr_threshold = 3;
    hit_threshold = 4;
//	hit_threshold_48 = 4.03;
//	hit_threshold_64 = 2.95;
    hit_threshold_auto = true;

    win_width = 64;
    win_stride_width = 8;
    win_stride_height = 8;

	flength_s = 5.0;
	flength_l = 5.0;

    gamma_corr = true;
	btracking = false;	//if tracking
	SIFT = false;
	TrafficSignDetection = false;
	signCategory = 3;
}


Args Args::read(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; i++)
    {
        if (string(argv[i]) == "--make_gray") args.make_gray = (string(argv[++i]) == "true");
        else if (string(argv[i]) == "--resize_src") args.resize_src = (string(argv[++i]) == "true");
        else if (string(argv[i]) == "--width") args.width = atoi(argv[++i]);
        else if (string(argv[i]) == "--height") args.height = atoi(argv[++i]);
        else if (string(argv[i]) == "--hit_threshold")
        {
            args.hit_threshold = atof(argv[++i]);
            args.hit_threshold_auto = false;
        }
        else if (string(argv[i]) == "--scale") args.scale = atof(argv[++i]);
        else if (string(argv[i]) == "--nlevels") args.nlevels = atoi(argv[++i]);
        else if (string(argv[i]) == "--win_width") args.win_width = atoi(argv[++i]);
        else if (string(argv[i]) == "--win_stride_width") args.win_stride_width = atoi(argv[++i]);
        else if (string(argv[i]) == "--win_stride_height") args.win_stride_height = atoi(argv[++i]);
        else if (string(argv[i]) == "--gr_threshold") args.gr_threshold = atoi(argv[++i]);
        else if (string(argv[i]) == "--gamma_correct") args.gamma_corr = (string(argv[++i]) == "true");
        else if (string(argv[i]) == "--write_video") args.write_video = (string(argv[++i]) == "true");
        else if (string(argv[i]) == "--dst_videos")
		{
			args.dst_videos = argv[++i];
			args.dst_name_auto = false;
		}
		else if (string(argv[i]) == "--dst_videol") args.dst_videol = argv[++i];
		else if (string(argv[i]) == "--dst_video_combine") args.dst_video_combine = argv[++i];		
        else if (string(argv[i]) == "--dst_video_fps") args.dst_video_fps = atof(argv[++i]);
		else if (string(argv[i]) == "--flength_s") args.flength_s = atof(argv[++i]);
		else if (string(argv[i]) == "--flength_l") args.flength_l = atof(argv[++i]);
        else if (string(argv[i]) == "--help") printHelp();
        else if (string(argv[i]) == "--videos") { args.srcs = argv[++i]; args.src_is_videos = true; }
		else if (string(argv[i]) == "--videol") { args.srcl = argv[++i]; args.src_is_videol = true; }
        else if (string(argv[i]) == "--cameras") {args.src_is_cameras = true; }
		else if (string(argv[i]) == "--cameral") {args.src_is_cameral = true; }
		else if (string(argv[i]) == "--tracking") {args.btracking = true; }
		else if (string(argv[i]) == "--SIFT") {args.SIFT = true;}
		else if (string(argv[i]) == "--signdetection") {args.TrafficSignDetection = true;}
		else if (string(argv[i]) == "--signcategory") args.signCategory = atoi(argv[++i]);
		else if (string(argv[i]) == "--rotate") {args.img_rotate = true; args.degree = atoi(argv[++i]);}
		else if (string(argv[i]) == "--factor") {args.factor = atof(argv[++i]);}
        else if (args.srcs.empty()) 
		{
			args.srcs = argv[i++];
			args.srcl = argv[i];
		}	
        else throw runtime_error((string("unknown key: ") + argv[i]));
    }
    return args;
}


App::App(const Args& s)
{
    cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

    args = s;
    cout << "\nControls:\n"
         << "\tESC - exit\n"
         << "\tu - change mode GPU <-> CPU\n"
         << "\tg - convert image to gray or not\n"
         << "\t1/q - increase/decrease HOG scale\n"
         << "\t2/w - increase/decrease levels count\n"
         << "\t3/e - increase/decrease HOG group threshold\n"
         << "\t4/r - increase/decrease hit threshold\n"
		 << "\t't' - if tracking\n"	
		 << "\t'b' - if show both detecting and tracking results\n"
		 << "\t'z' - Clear tracking results and Re-tracking\n"
		 << "\t'm' - Show measurement trajectory\n"
		 << "\t's' - Use SIFT or Regular Duplication Elimination\n"
		 << "\t'l' - Detect or not detect traffic sign\n"
         << endl;

    use_gpu = true;
	use_SIFT = args.SIFT; //if use SIFT to do the elimination proceess
	detectSign = args.TrafficSignDetection;	//if detect traffic sign
	category = args.signCategory;		//catetories of traffic sign detection
	showboth = false;
	showMeasurementTrajectory = false;
    make_gray = args.make_gray;
	btracking = args.btracking;					//if tracking
    scale = args.scale;
    gr_threshold = args.gr_threshold;
    nlevels = args.nlevels;
	firstframe = true;
	count = 0;

    if (args.hit_threshold_auto)
	{
		if(args.win_width == 16)
			args.hit_threshold = threshold16*2 + 0.99;
		if(args.win_width == 24)
			args.hit_threshold = threshold24*2;
		if(args.win_width == 32)
			args.hit_threshold = threshold32*2;
		if(args.win_width == 40)
			args.hit_threshold = threshold40*2;
		if(args.win_width == 48)
			args.hit_threshold = threshold48*2 + 1.01;//1.59
		if(args.win_width == 56)
			args.hit_threshold = threshold56*2;
		if(args.win_width == 64)
			args.hit_threshold = threshold64*2 + 0.75;//0.85
		if(args.win_width == 72)
			args.hit_threshold = threshold72*2;
		if(args.win_width == 80)
			args.hit_threshold = threshold80*2;
		if(args.win_width == 96)
			args.hit_threshold = threshold96*2;
	}
    hit_threshold = args.hit_threshold;

	if(args.dst_name_auto)
	{
		stringstream videoName;
		videoName.str("");
		videoName << "../" << args.width << "_" << args.win_width << "_" << "resultLong.avi" ;
		args.dst_videol = videoName.str() ;
		videoName.str("");
		videoName << "../" << args.width << "_" << args.win_width << "_" << "resultShort.avi" ;
		args.dst_videos = videoName.str() ;
		videoName.str("");
		videoName << "../" << args.width << "_" << args.win_width << "_" << "resultCombine.avi" ;
		args.dst_video_combine = videoName.str() ;
	}

    gamma_corr = args.gamma_corr;

    if (args.win_width != 64 && args.win_width != 48 && args.win_width != 16 && args.win_width != 24 && args.win_width != 32
		 && args.win_width != 40 && args.win_width != 56 && args.win_width != 72 && args.win_width != 80 && args.win_width != 96)
        args.win_width = 64;

    cout << "Scale: " << scale << endl;
    if (args.resize_src)
        cout << "Resized source: (" << args.width << ", " << args.height << ")\n";
    cout << "Group threshold: " << gr_threshold << endl;
    cout << "Levels number: " << nlevels << endl;
    cout << "Win width: " << args.win_width << endl;
    cout << "Win stride: (" << args.win_stride_width << ", " << args.win_stride_height << ")\n";
    cout << "Hit threshold: " << hit_threshold << endl;
    cout << "Gamma correction: " << gamma_corr << endl;
	cout << "Write video: " << args.write_video << endl;
	cout << "Traffic sign classification category: " << category << endl;
    cout << endl;
}


void App::run()
{
    running = true;
    cv::VideoWriter video_writer_ls, video_writer_rl, video_writer_combine;

    Size win_size(args.win_width, args.win_width * 2); //(64, 128) or (48, 96)
    Size win_stride(args.win_stride_width, args.win_stride_height);
    // Create HOG descriptors and detectors here
    //vector<float> detector;
    //if (win_size == Size(64, 128))
        //detector = getThisObjectDetector64("hogweightvector64");
		//detector = cv::gpu::HOGDescriptor::getPeopleDetector64x128();
    //else
        //detector = getThisObjectDetector48("hogweightvector48");
		//detector = cv::gpu::HOGDescriptor::getPeopleDetector48x96();

	//declare the pedestrian HOG descriptors
	vector<float> detector;
	if(args.win_width)
		detector = getThisObjectDetector(args.win_width);
	else
	{
		//detector = getThisObjectDetector64("hogweightvector64");
		cout<< "there is no win_width set in the parameters and the system did not detecte the default win_width"<<endl;
		running = false;
		system("pause");
		system("pause");

	}

	//declaration of pedestrian HOG descriptors
    gpu::HOGDescriptor gpu_hog(win_size, Size(16, 16), Size(8, 8), Size(8, 8), 9,
                                   cv::gpu::HOGDescriptor::DEFAULT_WIN_SIGMA, 0.2, gamma_corr,
								   cv::gpu::HOGDescriptor::DEFAULT_NLEVELS);
    HOGDescriptor cpu_hog(win_size, Size(16, 16), Size(8, 8), Size(8, 8), 9, 1, -1,
							HOGDescriptor::L2Hys, 0.2, gamma_corr, cv::HOGDescriptor::DEFAULT_NLEVELS);
	gpu_hog.setSVMDetector(detector);
    cpu_hog.setSVMDetector(detector);


	//declaration of pedestrian HOG descriptors
    cv::HOGDescriptor cpuHOGSign(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9, 1, -1,
                              HOGDescriptor::L2Hys, 0.2, gamma_corr, cv::HOGDescriptor::DEFAULT_NLEVELS);
    cpuHOGSign.setSVMDetector(getTrafficSignDetector("total"));

	vector<HOGDescriptor> cpuTrafficSignClassifier;
	for(int i = 0; i < category; ++i)
	{

		HOGDescriptor descriptorTemp(Size(40, 40), Size(16, 16), Size(8,8), Size(8, 8), 9,
										1, -1, HOGDescriptor::L2Hys, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS);

		switch(i)
		{
			//alwasy multiplied by 2
			case 0:
				descriptorTemp.setSVMDetector( getTrafficSignDetector("40") );//40-8.1
				cpuTrafficSignClassifier.push_back(descriptorTemp);
				break;
			case 1:
				descriptorTemp.setSVMDetector( getTrafficSignDetector("50") );//50-7.5
				cpuTrafficSignClassifier.push_back(descriptorTemp);
				break;
			case 2:
				descriptorTemp.setSVMDetector( getTrafficSignDetector("stop") );//stop-6.44
				cpuTrafficSignClassifier.push_back(descriptorTemp);
				break;
			//case 3:
			//	descriptorTemp.setSVMDetector( getDetector(argv[temp], 6.6089472) );//stop-6.44
			//	descriptorTempGpu.setSVMDetector( getDetector(argv[temp], 6.6089472) );
			//	trafficSignClassify.push_back(descriptorTemp);
			//	trafficSignClassifyGpu.push_back(descriptorTempGpu);
			//	break;
		}

	}


#ifdef PGRCamera
	//initialize the information of cameras
	//begin test multiple camreas if they are ready
	PrintBuildInfo();
	Error error;
	BusManager busMgr;
	unsigned int numCameras = 0;
	error = busMgr.GetNumOfCameras(&numCameras);
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		printf("can't get number of cameras");
		getchar();
		throw runtime_error(string("can't get number of cameras"));
	}

	if (args.src_is_cameras && args.src_is_cameral )
	{
		printf( "Number of cameras detected: %u\n", numCameras );
		if ( numCameras < 1 )
		{
			printf( "Insufficient number of cameras... press Enter to exit.\n" );
			getchar();
			throw runtime_error(string("number of cameras is not efficient"));
		}
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
			printf("can't get number of cameras");
			getchar();
			throw runtime_error(string("can't get number of cameras"));
		}

		// Connect to a camera
		error = ppCameras[i]->Connect( &guid );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			printf("can't get connection with cameras");
			getchar();
			throw runtime_error(string("can't get connection with cameras"));
		}

		// Get the camera information
		CameraInfo camInfo;
		error = ppCameras[i]->GetCameraInfo( &camInfo );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			printf("can't get cameras info");
			getchar();
			throw runtime_error(string("can't get cameras info"));
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
			throw runtime_error(string("please change the video mode"));
		}
    
	}

	if (args.src_is_cameras && args.src_is_cameral )
	{
		for ( int i=0; i<numCameras; i++ )
		{
			error = ppCameras[i]->StartCapture();
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				printf( 
					"Error starting cameras. \n"
					"This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
					"If your camera does not support this mode, please edit the source code and recompile the application. \n"
					"Press Enter to exit. \n");
				getchar();
				throw runtime_error(string("please check the cameras can't StartCapture"));
			}
		}
	}
#endif

    while (running)
    {
        VideoCapture vcs;
		VideoCapture vcl;
        Mat frames, framel;

		Mat temp;// temporary Mat to use for the continue detection by using cameras
		Image image;// temporary Image to use for the continue detection by using cameras

		//case 1 use videos
        if (args.src_is_videos && args.src_is_videol)
        {
            vcs.open(args.srcs.c_str());
			vcl.open(args.srcl.c_str());
            if (!vcs.isOpened() || !vcl.isOpened())
			{
                throw runtime_error(string("can't open video file: " + args.srcs +  "can't open video file: " + args.srcl));
			}
            vcs >> frames;
			vcl >> framel;
        }

		//case 2 use cameras
		//if the PGRCamera is active
        else if (args.src_is_cameras && args.src_is_cameral )
        {	
			for ( unsigned int i = 0; i < numCameras; i++ )
			{
				//a temporary image space to store the camera image
				error = ppCameras[i]->RetrieveBuffer( &image );
				if (error != PGRERROR_OK)
				{
					PrintError( error );
					throw runtime_error(string("can't retrueve image"));
				}
				
				//rotate the image based on the command line
				if(args.img_rotate)
				{
					IplImage * psrc = ConvertImageToOpenCV(&image);
					temp = Mat (rotateImage2( psrc, args.degree, args.factor));
					printf("img roateted to degree: %d \n",args.degree);
				}
				else
					temp = Mat (ConvertImageToOpenCV(&image));

				//copy temp to frames or framel				
				if (i == 0)
				{
					temp.copyTo(frames);
					printf("first get picure into frames \n");
					if(!frames.data) 
						throw runtime_error(string("can't change PGR into OpenCV image format"));
				}
				else
				{
					temp.copyTo(framel);
					printf("first get picure into framel \n");
					if(!framel.data) 
						throw runtime_error(string("can't change PGR into OpenCV image format")); 
				}
#ifdef SAVE_IMAGE
				char filename[512];
				sprintf( filename, "PGRCamera-%d.jpg", i );

				error = image.Save( filename );
				if (error != PGRERROR_OK)
				{
					PrintError( error );
				}  
				printf( "Grabbed image for camera-%d press ENTER to get naext image\n", i );
				getchar();
#endif //SAVE_IMAGE
			}
		}

		//case 3 use images
        else
        {
            frames = imread(args.srcs);
			framel = imread(args.srcl);
            if (frames.empty() || framel.empty())			
                throw runtime_error(string("can't open image file: " + args.srcs + "or" + args.srcl));
        }

		//compare if the datas in frames and framel are the same
		if(frames.data == framel.data)
			throw runtime_error(string("the data for frames and framel are the same"));
		
        Mat img_auxs, imgs, img_to_shows; //第一个视频相关 short focal length camera
        gpu::GpuMat gpu_imgs;
		
		Mat img_auxl, imgl, img_to_showl; //第二个视频相关 long focal length camera
        gpu::GpuMat gpu_imgl;

		Mat img_combine, img_combine_gray;//for the test of combination

		Mat imgROIs, imgROIl;
		gpu::GpuMat imgROIGpus, imgROIGpul;

		//************************************KalmanFilter vector initialized ******************
		vector<KalmanFilterTracker> KFTvectors, KFTvectorl;								//define KalmanFilter Vector for tracking detection results
		//vector<KalmanFilterTracker> KFTvectorlts;										//from long to short

		//definition of the ratio between short and long cameras
		//double fs = args.flength_s;//focal length of short camera
		//double fl = args.flength_l;//focal length of long camera
		double ratio = args.flength_s / args.flength_l; 
		int resx = args.width/2 - ratio * ( args.width/2 );				//relationship with the pixels of detection image
		int resy = args.height/2 - ratio * ( args.height/2 );
		int tempx = 12;
		int tempy = 4;

        // Iterate over all frames
        while (running && !frames.empty() && !framel.empty())
        {
			//cout << count << endl;
			count += 1;
			if(firstframe)
			{
				if(!KFTvectors.empty())
				{
					KFTvectors.clear();
					cout<<"KFTvectors initialed and cleared"<<endl;
				}
				if(!KFTvectorl.empty())
				{
					KFTvectorl.clear();
					cout<<"KFTvectorl initialed and cleared"<<endl;
				}
			}
			vector<KalmanFilterTracker> KFTvectorlns;											//initialize the KFTvectorlns to show in combine image
			if(!KFTvectorlns.empty())
			{
				KFTvectorlns.clear();
				cout<<"KFTvectorlns initialed and cleared"<<endl;
			}
            workBegin();

			vector<Rect> founds, foundl, found_filters, found_filterl;	//temporary found results
			vector<Rect> foundtempl, foundtemps, foundtemplns;//foundtemps are not used, foundtempl is used for tracking 
			
			vector<Rect> foundSigns, foundSignl, foundSignFilters, foundSignFilterl;
			vector<Rect> foundSignTempl, foundSignTemps, foundSignTemplns;//foundtemps are not used, foundtempl is used for tracking 
			//vector<Rect> foundKFTs, foundKFTl, foundKFTlns;
			
            // Change format of the image
            if (make_gray) 
			{
				cvtColor(frames, img_auxs, CV_BGR2GRAY);
				cvtColor(framel, img_auxl, CV_BGR2GRAY);
			}
            else if (use_gpu) 
			{
				cvtColor(frames, img_auxs, CV_BGR2BGRA);
				cvtColor(framel, img_auxl, CV_BGR2BGRA);
			}
            else 
			{
				frames.copyTo(img_auxs);
				framel.copyTo(img_auxl);
			}

            // Resize image
            if (args.resize_src) 
			{
				resize(img_auxs, imgs, Size(args.width, args.height));
				resize(img_auxl, imgl, Size(args.width, args.height));
				resize(img_auxs, img_combine, Size(args.width, args.height));
				
			}
            else 
			{
				imgs = img_auxs;
				imgl = img_auxl;
				img_auxs.copyTo(img_combine);
			}
			img_to_shows = imgs;
            img_to_showl = imgl;


            gpu_hog.nlevels = nlevels;
            cpu_hog.nlevels = nlevels;

            // Perform HOG classification
            hogWorkBegin();
            if (use_gpu)
            {
                gpu_imgs.upload(imgs);
				gpu_imgl.upload(imgl);
                gpu_hog.detectMultiScale(gpu_imgs, founds, hit_threshold, win_stride,
                                         Size(0, 0), scale, gr_threshold);
										//Coefficient to regulate the similarity threshold. When detected, 
										//some objects can be covered by many rectangles. 0 means not to perform grouping.
				gpu_hog.detectMultiScale(gpu_imgl, foundl, hit_threshold, win_stride,
                                         Size(0, 0), scale, gr_threshold);

				//cout << "founds.size(): " << founds.size() << endl;
				//cout << "foundl.size(): " << foundl.size() << endl;
				detectionModel = "GPU";
            }
            else 
			{
				cpu_hog.detectMultiScale(imgs, founds, hit_threshold, win_stride,
                                          Size(0, 0), scale, gr_threshold);
				cpu_hog.detectMultiScale(imgl, foundl, hit_threshold, win_stride,
                                          Size(0, 0), scale, gr_threshold);
				if(detectSign)
				{
					cpuHOGSign.detectMultiScale(imgs, foundSigns, 9.0, win_stride,
											  Size(0, 0), scale, gr_threshold);
					cpuHOGSign.detectMultiScale(imgl, foundSignl, 9.0, win_stride,
											  Size(0, 0), scale, gr_threshold);
				}
				detectionModel = "CPU";
			}

			hogWorkEnd();


            // Draw positive classified windows
			//detection image of SHORT FOCAL LENGTH
			//test if the results are the same person

			int countAll = 0 , countElimination = 0;

			if(!use_SIFT)
			{
				for (size_t i=0; i<founds.size(); i++)
				{
					size_t j;
					Rect rs = founds[i];
					for ( j=0; j<founds.size(); j++) 
						if (j!=i && (rs & founds[j]) == rs)
							break;

					rs.x += cvRound(rs.width*0.1);
					rs.width = cvRound(rs.width*0.8);
					rs.y += cvRound(rs.height*0.07);
					rs.height = cvRound(rs.height*0.8);

					if (j == founds.size())
					{
						found_filters.push_back(rs);
						foundtemplns.push_back(rs);							//push back to show
						//foundtemps.push_back(rs);
					}

					//get the enlarged elimination area
					Rect rstemp;
					int tempx = rs.x;
					int tempy = rs.y;
					int tempwidth = rs.width;
					int tempheight = rs.height;
					rstemp.x = tempx - args.win_width/2;
					rstemp.y = tempy - args.win_width;
					rstemp.width = tempx + tempwidth + args.win_width/2 - rstemp.x;
					rstemp.height = tempy +tempheight + args.win_width - rstemp.y;
					foundtemps.push_back(rstemp);

					countAll += 1; 
				}
				countElimination = countAll;

				//detection image of LONG FOCAL LENGTH 
				//test if the results are the same person
				for (size_t i=0; i<foundl.size(); i++)
				{
					size_t j;
					Rect rl = foundl[i];
					for ( j=0; j<foundl.size(); j++) 
						if (j!=i && (rl & foundl[j]) == rl)
							break;

					rl.x += cvRound(rl.width*0.1);
					rl.width = cvRound(rl.width*0.95);
					rl.y += cvRound(rl.height*0.07);
					rl.height = cvRound(rl.height*0.9);

					if (j == foundl.size())
					{
						found_filterl.push_back(rl);

						//Rect rls;					// a filter a eliminate the same rect both in image1 and image2
						//rls.x = rl.x*ratio + resx + tempx;
						//rls.y = rl.y*ratio + resy + tempy;
						//rls.width = rl.width*ratio;
						//rls.height = rl.height*ratio;
						//foundtemplns.push_back(rls);//push long result to L and S without elimination
					}
					countAll += 1;
				}

				//put the number of all detection results into one file
				stringstream ssTempAll, ssTempAllResults;
				ssTempAll << count << "\t" << countAll;
				ssTempAllResults << "DetectionResultsAll_" << args.width << "_" << args.win_width <<".txt";
				//ssTempAllResults << count << "\t" << "AllDetectionResults_" << args.width << "_" << args.win_width <<".txt";
				ofstream resultsAll;
				resultsAll.open(ssTempAllResults.str(),ios::app);
				resultsAll << ssTempAll.str() << endl;
				resultsAll.close();
				
				for ( size_t i = 0; i < found_filterl.size(); ++i )			//eleminiate same detection results
				{
					//cout << "for ( size_t i = 0; i < found_filterl.size(); ++i )" <<endl;
					//reduce the ratio of the detection from the	
					//large focal length image to show it on small image

					Rect rls;												// a filter a eliminate the same rect both in image1 and image2
					rls.x = found_filterl[i].x*ratio + resx + tempx;
					rls.y = found_filterl[i].y*ratio + resy + tempy;
					rls.width = found_filterl[i].width*ratio;
					rls.height = found_filterl[i].height*ratio;

					if( foundtemps.empty() )
					{
						countElimination += 1;
						foundtemplns.push_back(rls);						//pushback to show
						foundtempl.push_back( found_filterl[i] );
						continue;
					}

					size_t j = 200;
					for ( j = 0; j < foundtemps.size(); ++j )
					{
						Rect rs = foundtemps[j];			

						//eliminate the reduplicated results for the same person,if there are some in common, 
						//if the centroid of the long recs in the short recs, eliminate them
						if( ( (rls.x + rls.width/2) > rs.x) && (( rls.x + rls.width/2 ) < ( rs.x + rs.width ))
							&& ((rls.y + rls.height/2 ) > rs.y) && (( rls.y + rls.height/2 ) < ( rs.y + rs.height )) )
							break;
						//if the abs of the left-upper corner of the long recs and short recs less than 30, eliminate them
						//else if ( (abs(rls.x-rs.x) <= 35) && (abs(rls.y-rs.y) <= 60) )
						//	break;
						//if the abs of the left-lower corner of the long recs and short recs less than 30, eliminate them
						//else if ( (abs(rls.y + rls.height - rs.y - rs.height) <= 60) && (abs(rls.x - rs.x) <= 35) )
						//	break;
						//if the abs of the right-upper corner of the long recs and short recs less than 30, eliminate them
						//else if ( (abs(rls.y - rs.y ) <= 60) && (abs(rls.x + rls.width - rs.x - rs.width) <= 35) )
						//	break;
						//if the abs of the right-lower corner of the long recs and short recs less than 30, eliminate them
						//else if ( (abs(rls.br().x - rs.br().x) <= 35) && (rls.br().y - rs.br().y) <= 60 )
						//	break;
					}
					//countElimination += 1;
					if ( j == found_filters.size() && j != 0)
					{
						countElimination += 1;
						foundtemplns.push_back(rls);
						foundtempl.push_back( found_filterl[i] );
					}
				} //for() for update the detection results ends
				

				stringstream ssTempElimination, ssTempEliminationResults;
				ssTempElimination <<  count << "\t" << countElimination;
				ssTempEliminationResults << "DetectionResultsEliminated_" << args.width << "_" << args.win_width <<".txt";
				//ssTempEliminationResults << count << "\t" << "EliminatedDetectionResults_" << args.width << "_" << args.win_width <<".txt";
				ofstream resultsElimination;
				resultsElimination.open(ssTempEliminationResults.str(),ios::app);
				resultsElimination << ssTempElimination.str() << endl;
				resultsElimination.close();

				if(detectSign)
				{
					for(size_t i = 0; i < foundSigns.size(); ++i)
					{
						size_t j;
						Rect sSign = foundSigns[i];
						for ( j = 0; j < foundSigns.size(); ++j ) 
							if (j!=i && (sSign & foundSigns[j]) == sSign)
								break;

						sSign.x += cvRound(sSign.width*0.1);
						sSign.width = cvRound(sSign.width*0.8);
						sSign.y += cvRound(sSign.height*0.07);
						sSign.height = cvRound(sSign.height*0.8);

						if (j == foundSigns.size())
						{
							foundSignFilters.push_back(sSign);
							foundSignTemplns.push_back(sSign);							//push back to show
							//foundtemps.push_back(rs);
						}

						Rect rstemp;
						int tempx = sSign.x;
						int tempy = sSign.y;
						int tempwidth = sSign.width;
						int tempheight = sSign.height;
						rstemp.x = tempx - args.win_width/2;
						rstemp.y = tempy - args.win_width;
						rstemp.width = tempx + tempwidth + args.win_width/2 - rstemp.x;
						rstemp.height = tempy +tempheight + args.win_width - rstemp.y;
						foundSignTemps.push_back(rstemp);

					}

					for (size_t i=0; i<foundSignl.size(); i++)
					{
						size_t j;
						Rect rSign = foundSignl[i];
						for ( j=0; j<foundSignl.size(); j++) 
							if (j!=i && (rSign & foundSignl[j]) == rSign )
								break;

						rSign.x += cvRound(rSign.width*0.1);
						rSign.width = cvRound(rSign.width*0.95);
						rSign.y += cvRound(rSign.height*0.07);
						rSign.height = cvRound(rSign.height*0.9);

						if (j == foundSignl.size())
							foundSignFilterl.push_back(rSign);
					}

					//cout << "foundSignFilters.size(): " << foundSignFilters.size() << endl;
					//cout << "foundSignFilterl.size(): " << foundSignFilterl.size() << endl;

					for ( size_t i = 0; i < foundSignFilterl.size(); ++i )			//eleminiate same detection results
					{
						//cout << "for ( size_t i = 0; i < found_filterl.size(); ++i )" <<endl;
						//reduce the ratio of the detection from the	
						//large focal length image to show it on small image
						Rect rlsSign;												// a filter a eliminate the same rect both in image1 and image2
						rlsSign.x = foundSignFilterl[i].x*ratio + resx + tempx;
						rlsSign.y = foundSignFilterl[i].y*ratio + resy + tempy;
						rlsSign.width = foundSignFilterl[i].width*ratio;
						rlsSign.height = foundSignFilterl[i].height*ratio;

						if( foundSignTemps.empty() )
						{
							foundSignTemplns.push_back(rlsSign);
							foundSignTemps.push_back( foundSignFilterl[i] );
							continue;
						}

						size_t j = 200;
						for ( j = 0; j < foundSignTemps.size(); ++j )
						{
							Rect rsSign = foundSignTemps[j];			

							//eliminate the reduplicated results for the same person,if there are some in common, 
							//if the centroid of the long recs in the short recs, eliminate them
							if( ((rlsSign.x + rlsSign.width/2 ) > rsSign.x )
								&& (( rlsSign.x + rlsSign.width/2 ) < ( rsSign.x + rsSign.width ))
								&& ( (rlsSign.y + rlsSign.height/2 ) > rsSign.y)
								&& (( rlsSign.y + rlsSign.height/2 ) < ( rsSign.y + rsSign.height )) )
								break;
							//if the abs of the left-upper corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rlsSign.x - rsSign.x) <= 30) && (abs(rlsSign.y - rsSign.y) <= 30) )
							//	break;
							//if the abs of the left-lower corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rlsSign.y + rlsSign.height - rsSign.y - rsSign.height) <= 30) 
							//		&& (abs(rlsSign.x + rlsSign.height - rsSign.x - rsSign.height) <= 30) )
							//	break;
							//if the abs of the right-upper corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rlsSign.y + rlsSign.width - rsSign.y - rsSign.width) <= 30) 
							//		&& (abs(rlsSign.x + rlsSign.width - rsSign.x - rsSign.width) <= 30) )
							//	break;
							//if the abs of the right-lower corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rlsSign.br().x - rsSign.br().x) <= 30) 
							//		&& (rlsSign.br().y - rsSign.br().y) <= 30 )
							//	break;
						}
						//countElimination += 1;
						if ( j == foundSignFilters.size() && j != 0)
						{
							foundSignTemplns.push_back(rlsSign);
							foundSignTempl.push_back( foundSignFilterl[i] );
						}
					} //for() for update the detection results ends

					//cout << "foundSignTemplns.size(): " << foundSignTemplns.size() << endl;

				}//end if(detectSign)

			}//if use regular duplication elimination method

			if(use_SIFT)
			{
				for (size_t i=0; i<founds.size(); i++)
				{
					size_t j;
					Rect rs = founds[i];
					for ( j=0; j<founds.size(); j++) 
						if (j!=i && (rs & founds[j]) == rs)
							break;
					if (j == founds.size())
					{
						found_filters.push_back(rs);
						foundtemplns.push_back(rs);							//push back to show
						//foundtemps.push_back(rs);
					}
					countAll += 1; 
				}
				countElimination = countAll;

				//detection image of LONG FOCAL LENGTH 
				//test if the results are the same person
				for (size_t i=0; i<foundl.size(); i++)
				{
					size_t j;
					Rect rl = foundl[i];
					for ( j=0; j<foundl.size(); j++) 
						if (j!=i && (rl & foundl[j]) == rl)
							break;
					if (j == foundl.size())
						found_filterl.push_back(rl);

					countAll += 1;
				}

				//put the number of all detection results into one file
				stringstream ssTempAll, ssTempAllResults;
				ssTempAll << count << "\t" << countAll;
				ssTempAllResults << "DetectionResultsAll_SIFT_" << args.width << "_" << args.win_width <<".txt";
				//ssTempAllResults << count << "\t" << "AllDetectionResults_" << args.width << "_" << args.win_width <<".txt";
				ofstream resultsAll;
				resultsAll.open(ssTempAllResults.str(),ios::app);
				resultsAll << ssTempAll.str() << endl;
				resultsAll.close();


				for (size_t j=0; j<found_filterl.size(); ++j)
				{
					Mat resultImgS, resultImgL;
					//Rect rectL = Rect(100,100,300,300);
					Rect rectL = found_filterl[j];
					imgl(rectL).copyTo(resultImgL);
					resize(resultImgL, resultImgL, Size(args.win_width, args.win_width*2));
					cvtColor(resultImgL, resultImgL, CV_RGBA2GRAY);

					size_t i= 200;
					if(found_filters.empty())
					{
						countElimination += 1;
						rectL.x = found_filterl[j].x*ratio + resx + tempx;
						rectL.y = found_filterl[j].y*ratio + resy + tempy;
						rectL.width = found_filterl[j].width*ratio;
						rectL.height = found_filterl[j].height*ratio;
						foundtemplns.push_back(rectL);						//pushback to show
						foundtempl.push_back( found_filterl[j] );
						continue;
					}

					for ( ; i<found_filters.size(); ++i)
					{
						//Rect rectS = Rect(100,100,300,300);
						Rect rectS = found_filters[i];
						imgs(rectS).copyTo(resultImgS);
						resize(resultImgS, resultImgS, Size(args.win_width, args.win_width*2));
						cvtColor(resultImgS, resultImgS, CV_RGBA2GRAY);
						//-- Step 1: Detect the keypoints using SURF Detector
						int minHessian = 400;

						SurfFeatureDetector detector( minHessian );

						std::vector<KeyPoint> keypoints_1, keypoints_2;

						detector.detect( resultImgS, keypoints_1 );
						detector.detect( resultImgL, keypoints_2 );


						//-- Draw keypoints
						Mat img_keypoints_1, img_keypoints_2;


						drawKeypoints( resultImgS, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
						drawKeypoints( resultImgL, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

						//-- Show detected (drawn) keypoints
						imshow("Keypoints 1", img_keypoints_1 );
						imshow("Keypoints 2", img_keypoints_2 );

						//-- Step 2: Calculate descriptors (feature vectors)
						SurfDescriptorExtractor extractor;

						Mat descriptors_1, descriptors_2;

						extractor.compute( resultImgS, keypoints_1, descriptors_1 );
						extractor.compute( resultImgL, keypoints_2, descriptors_2 );

						//-- Step 3: Matching descriptor vectors with a brute force matcher
						BFMatcher matcher(NORM_L2);
						std::vector< DMatch > matches;
						matcher.match( descriptors_1, descriptors_2, matches );

						//-- Draw matches
						Mat img_matches;
						drawMatches( resultImgS, keypoints_1, resultImgL, keypoints_2, matches, img_matches );

						//-- Show detected matches
						imshow("Matches", img_matches );

						if(matches.size() > 15)
							break;
					}
					if( i == found_filters.size() && i != 0 )
					{
						countElimination += 1;
						rectL.x = found_filterl[j].x*ratio + resx + tempx;
						rectL.y = found_filterl[j].y*ratio + resy + tempy;
						rectL.width = found_filterl[j].width*ratio;
						rectL.height = found_filterl[j].height*ratio;
						
						foundtemplns.push_back(rectL);
						foundtempl.push_back( found_filterl[j] );
					}
				}

				stringstream ssTempElimination, ssTempEliminationResults;
				ssTempElimination <<  count << "\t" << countElimination;
				ssTempEliminationResults << "DetectionResultsEliminated_SIFT_" << args.width << "_" << args.win_width <<".txt";
				//ssTempEliminationResults << count << "\t" << "EliminatedDetectionResults_" << args.width << "_" << args.win_width <<".txt";
				ofstream resultsElimination;
				resultsElimination.open(ssTempEliminationResults.str(),ios::app);
				resultsElimination << ssTempElimination.str() << endl;
				resultsElimination.close();


			}//if use SIFT duplication elimination method


//traffic sign classification************************************************//
			//draw traffic sign detection resutls on original images
			if(detectSign)
			{
				
				for( size_t i = 0; i < foundSignFilters.size(); ++i)
				{
					//extract the region of interest from the orignal image.
					Rect roi;	
					roi.x += foundSignFilters[i].x - foundSignFilters[i].width*0.2;
					roi.width = foundSignFilters[i].width*1.4;
					roi.y += foundSignFilters[i].y - foundSignFilters[i].height*0.2;
					roi.height = foundSignFilters[i].height*1.4;
					if(roi.x < 0)
						roi.x = 0;
					if(roi.y < 0)
						roi.y = 0;
					if( (roi.x + roi.width) > img_combine.cols )
						roi.width = img_combine.cols - roi.x;
					if( (roi.y + roi.height) > img_combine.rows )
						roi.height = img_combine.rows - roi.y;
					if ( (roi.width <40)||(roi.height < 40) )	//the effect or role??
						continue;

					bool flagClassification[20] = {0};
					for(int j = 0; j < category; ++j)
					{
						Mat imgToImgGpu;
						Mat imgROI = img_combine(roi);
						gpu::GpuMat imgROIGpu;

						imshow("imgROI_DetectionResult", imgROI);
						cvtColor(imgROI, imgToImgGpu, CV_BGR2BGRA);
						imgROIGpu.upload(imgToImgGpu);				

						string categoryClassified;
						vector<Rect> foundClassified;
						//vector<int> flagClassification(category);
				
						switch(j)
						{
							case 0:
								categoryClassified = "40";
								cpuTrafficSignClassifier[j].detectMultiScale(imgROI, foundClassified, 8.1, 
															Size(8,8), Size(0,0), 1.05, 2); //threshold						
								break;
							case 1:
								categoryClassified = "50";
								cpuTrafficSignClassifier[j].detectMultiScale(imgROI, foundClassified, 7.5, 
															Size(8,8), Size(0,0), 1.05, 2); //threshold						
								break;
							case 2:
								categoryClassified = "stop";
								cpuTrafficSignClassifier[j].detectMultiScale(imgROI, foundClassified, 6.8, 
															Size(8,8), Size(0,0), 1.05, 2); //threshold						
								break;
					
						}				

						if(foundClassified.empty()) continue;
						else
						{
							if(!flagClassification[j] )
							{
								stringstream signClassifiedName;
								signClassifiedName << "./image/" << categoryClassified << ".jpg" ;
								Mat temp = imread(signClassifiedName.str());
								imshow(categoryClassified, temp);
								flagClassification[j] = 1 ;
							}
						}

					}//end of "for(int i = 0; i < category; ++i)"

					rectangle(img_to_shows, foundSignFilters[i], CV_RGB(255, 255, 0), 2);
				}

				for( size_t i = 0; i < foundSignTempl.size(); ++i)
				{
					//extract the region of interest from the orignal image.
					Rect roi;	
					roi.x += foundSignTempl[i].x - foundSignTempl[i].width*0.2;
					roi.width = foundSignTempl[i].width*1.4;
					roi.y += foundSignTempl[i].y - foundSignTempl[i].height*0.2;
					roi.height = foundSignTempl[i].height*1.4;
					if(roi.x < 0)
						roi.x = 0;
					if(roi.y < 0)
						roi.y = 0;
					if( (roi.x + roi.width) > img_combine.cols )
						roi.width = img_combine.cols - roi.x;
					if( (roi.y + roi.height) > img_combine.rows )
						roi.height = img_combine.rows - roi.y;
					if ( (roi.width <40)||(roi.height < 40) )	//the effect or role??
						continue;

					bool flagClassification[20] = {0};
					for(int j = 0; j < category; ++j)
					{
						Mat imgROI = img_combine(roi);
						imshow("imgROI_DetectionResult", imgROI);				

						string categoryClassified;
						vector<Rect> foundClassified;
						//vector<int> flagClassification(category);
				
						switch(j)
						{
							case 0:
								categoryClassified = "40";
								cpuTrafficSignClassifier[j].detectMultiScale(imgROI, foundClassified, 8.1, 
															Size(8,8), Size(0,0), 1.05, 2); //threshold						
								break;
							case 1:
								categoryClassified = "50";
								cpuTrafficSignClassifier[j].detectMultiScale(imgROI, foundClassified, 7.5, 
															Size(8,8), Size(0,0), 1.05, 2); //threshold						
								break;
							case 2:
								categoryClassified = "stop";
								cpuTrafficSignClassifier[j].detectMultiScale(imgROI, foundClassified, 6.8, 
															Size(8,8), Size(0,0), 1.05, 2); //threshold						
								break;
					
						}				

						if(foundClassified.empty()) continue;
						else
						{
							if(!flagClassification[j] )
							{
								stringstream signClassifiedName;
								signClassifiedName << "./image/" << categoryClassified << ".jpg" ;
								Mat temp = imread(signClassifiedName.str());
								imshow(categoryClassified, temp);
								flagClassification[j] = 1 ;
							}
						}

					}//end of "for(int i = 0; i < category; ++i)"

					rectangle(img_to_showl, foundSignTempl[i], CV_RGB(0, 255, 255), 2);
				}//end "for( size_t i = 0; i < foundSignTempl.size(); ++i)"

				for( size_t i = 0; i < foundSignTemplns.size(); ++i)
				{
					rectangle(img_combine, foundSignTemplns[i], CV_RGB(255, 0, 255), 2);
				}//end "for( size_t i = 0; i < foundSignTemplns.size(); ++i)"
			}//end "if(detectSign)"


//*********************************tracking begin************************************************//
			if( btracking )
			{
				if(!firstframe)												//not the initialization step
				{
					//cout<<"Continue tracking"<<endl;
					//******************************Check if new object detected******************************************
					cout << "\n";
					cout << "KFTvectors.size(): " << KFTvectors.size() << endl;	//get the current vector size
					cout << "KFTvectorl.size(): " << KFTvectorl.size() << endl;

					//put new detection results into KalmanFilters---short focal length image
					for(size_t i = 0; i < found_filters.size(); ++i)					
					{
						
						cout << "put new detection results into KFTvectors, found_filters.size: "<< found_filters.size() <<endl;
						cout << " found_filters["<< i <<"]"<<endl;

						KalmanFilterTracker KFTtemps;
						Rect ls = found_filters[i];
						size_t j;
						int tempx, tempy, tempwidth, tempheight;
						for( j = 0; j < KFTvectors.size(); ++j)					//cycle for the original KFTvector
						{	
							cout<< "begin cycle for the original KFTvectors: =  " << j << endl;

							int t = KFTvectors[j].statePostRectVector.size() - 1;
							if( t < 0 )											//ensure the pointer is not out of range
								continue;
							//get the comparing area					
							tempx = KFTvectors[j].statePostRectVector[t].x;
							tempy = KFTvectors[j].statePostRectVector[t].y;
							tempwidth = KFTvectors[j].statePostRectVector[t].width;
							tempheight = KFTvectors[j].statePostRectVector[t].height;
							Rect temp;
							temp.x = tempx - args.win_width/2;
							temp.y = tempy - args.win_width;
							temp.width = tempx + tempwidth + args.win_width/2 - temp.x;
							temp.height = tempy + tempheight + args.win_width - temp.y;					

							//if the measurement result's center in the rectangle of statepost,update it
							if( ( (ls.x + ls.width/2) > temp.x )
								&& (( ls.x + ls.width/2 ) < ( temp.x + temp.width ))
								&& ( (ls.y + ls.height/2) > temp.y)
								&& (( ls.y + ls.height/2 ) < ( temp.y + temp.height )) )
							{
								break;
							}
							//if the abs of the left-upper corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(ls.x-temp.x) <= 15) && (abs(ls.y-temp.y) <= 15) )
							//	break;
							//if the abs of the left-lower corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(ls.y + ls.height - temp.y - temp.height) <= 15) && (abs(ls.x + ls.height - temp.x - temp.height) <= 15) )
							//	break;
							//if the abs of the right-upper corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(ls.y + ls.width - temp.y - temp.width) <= 15) && (abs(ls.x + ls.width - temp.x - temp.width) <= 15) )
							//	break;
							//if the abs of the right-lower corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(ls.br().x - temp.br().x) <= 15) && (abs(ls.br().y - temp.br().y) <= 15) )
							//	break;
							cout << "end cycle for the original KFTvectors: j =  "<<j<<endl;
						}
						cout<< "after cycle for the original, KFTvectors.size()= " << KFTvectors.size() << ", j = " << j << endl;
						cout<< "after cycle for the original KFTvectors, KFTvectors.size = "<<KFTvectors.size()<<endl;

						if( j == KFTvectors.size() )
						{	
							cout<< "before KFTvectors.push_back(KFTtempl): "<< KFTvectors.size()<<endl;
							KFTtemps.initKF(ls.x, ls.y, ls.width, ls.height);		//put new detection in to tracking, even if the the KFTvector is empty
							KFTvectors.push_back(KFTtemps);
							cout<< "after KFTvectors.push_back(KFTtempl): "<< KFTvectors.size()<<endl;
						}

					}

					//put new detection results into KalmanFilters---long local length image
					for(size_t i = 0; i < foundtempl.size(); ++i)
					{
						cout<<"\n";
						cout << "put new detection results into KFTvectorl, found_filterl.size: "<< found_filters.size() <<endl;
						cout << " found_filterl["<< i <<"]"<<endl;

						KalmanFilterTracker KFTtempl;
						Rect rl = foundtempl[i];
						size_t j;
						int tempx, tempy, tempwidth, tempheight;

						for( j = 0; j < KFTvectorl.size(); ++j)						//cycle for the original KFTvector
						{
							cout<< "beging cycle for the original KFTvectorl: j =  "<<j<<endl;

							int t = KFTvectorl[j].statePostRectVector.size() - 1;	
							if( t < 0 )
								continue;

							//get the comparing area					
							tempx = KFTvectorl[j].statePostRectVector[t].x;
							tempy = KFTvectorl[j].statePostRectVector[t].y;
							tempwidth = KFTvectorl[j].statePostRectVector[t].width;
							tempheight = KFTvectorl[j].statePostRectVector[t].height;
							Rect temp;
							temp.x = tempx - args.win_width/2;
							temp.y = tempy - args.win_width;
							temp.width = tempx + tempwidth + args.win_width/2 - temp.x;
							temp.height = tempy + tempheight + args.win_width - temp.y;
							//if the measurement result's center in the rectangle of statepost,update it
							if( ( (rl.x + rl.width/2) > temp.x )
								&& (( rl.x + rl.width/2 ) < ( temp.x + temp.width ))
								&& ((rl.y + rl.height/2) > temp.y )
								&& (( rl.y + rl.height/2 ) < ( temp.y + temp.height ))	)
							{
								break;
							}

							//if the abs of the left-upper corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rl.x-temp.x) <= 15) && (abs(rl.y-temp.y) <= 15) )
							//	break;
							//if the abs of the left-lower corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rl.y + rl.height - temp.y - temp.height) <= 15) && (abs(rl.x + rl.height - temp.x - temp.height) <= 15) )
							//	break;
							//if the abs of the right-upper corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rl.y + rl.width - temp.y - temp.width) <= 15) && (abs(rl.x + rl.width - temp.x - temp.width) <= 15) )
							//	break;
							//if the abs of the right-lower corner of the long recs and short recs less than 30, eliminate them
							//else if ( (abs(rl.br().x - temp.br().x) <= 15) && (abs(rl.br().y - temp.br().y) <= 15) )
							//	break;

						}
						cout<< "after cycle for the original, KFTvectorl.size() = " << KFTvectorl.size() << ", j = "<<j<<endl;

						if( j == KFTvectorl.size() )									//put new detection in to tracking, even if the the KFTvector is empty
						{
							cout<< "befroe KFTvectorl.push_back(KFTtempl): "<< KFTvectorl.size()<<endl;
							KFTtempl.initKF(rl.x, rl.y, rl.width, rl.height);
							KFTvectorl.push_back(KFTtempl);
							cout<< "after KFTvectorl.push_back(KFTtempl): "<< KFTvectorl.size()<<endl;
						}

					}

					//***************************update KalmanFilters*********************
					//update KFTvectors 
					int xtemp, ytemp, widthtemp, heighttemp;
					for(size_t j = 0; j < KFTvectors.size(); ++j)
					{
						cout << "KFTvectors.size() after new detection reults in, total: " << KFTvectors.size() << ", No.: KFTvectors["<<j<<"]"<<endl;

						int t = KFTvectors[j].statePostRectVector.size() - 1;			//t == the newest Rect vector number
						if( t < 0)
							continue;
						size_t i;
						//temps == the neweset Rect in statePostRectVector
						xtemp = KFTvectors[j].statePostRectVector[t].x;
						ytemp = KFTvectors[j].statePostRectVector[t].y;
						widthtemp = KFTvectors[j].statePostRectVector[t].width;
						heighttemp = KFTvectors[j].statePostRectVector[t].height;
						Rect temps;
						temps.x = xtemp - args.win_width/2;
						temps.y = ytemp - args.win_width;
						temps.width = xtemp + widthtemp + args.win_width/2 - temps.x;
						temps.height = ytemp + heighttemp + args.win_width - temps.y;
						
						for( i = 0; i < found_filters.size(); ++i)
						{
							Rect ls = found_filters[i];

							//if the measurement result's center in the rectangle of statepost,update it
							if( (( ls.x + ls.width/2 ) > temps.x )
								&& (( ls.x + ls.width/2 ) < ( temps.x + temps.width ))
								&& (( ls.y + ls.height/2 ) > temps.y)
								&& (( ls.y + ls.height/2 ) < ( temps.y + temps.height ))	)
							{
								cout << "KFTvectors["<<j<<"].updateKF" << endl;
								KFTvectors[j].updateKF(ls.x, ls.y, ls.width, ls.height);
								KFTvectorlns.push_back(KFTvectors[j]);						//show tracking results in combine image
								break;
							}		
						}
						if( i == found_filters.size() )
						{

							KFTvectors[j].addcounter();
							cout << "KFTvectors["<<j<<"].addcounter() : "<< KFTvectors[j].getcounter() << endl;
							if( KFTvectors[j].getcounter() > 3 )						//different miss rate define different performance
							{
								cout << "KFTvectors[ "<<j<<" ].getcounter()" << endl;
								KFTvectors.erase(KFTvectors.begin()+j);
								cout<<"Now j is : "<<j<<endl;
								if( j != 0 )
									--j;								
								cout<<"Now new j is : "<<j<<endl;

							}
							else
							{
								cout << "KFTvectors["<<j<<"].updateWithoutMeasurement()" << endl;
								KFTvectors[j].updateWithoutMeasurement();
								KFTvectorlns.push_back(KFTvectors[j]);						//show tracking results in combine image
							}
						}
					}
					
					//update KFTvectorl 
					for(int j = 0; j < KFTvectorl.size(); ++j)
					{
						cout << "KFTvectorl.size() after new detection reults in, total: " << KFTvectorl.size() << ", No.: KFTvectorl[" << j <<"]"<<endl;

						KalmanFilterTracker KFTtemplts;
						int t = KFTvectorl[j].statePostRectVector.size() - 1;			//t == the newest Rect vector number
						if( t < 0 )
							continue;
						//Rect templ = KFTvectorl[j].statePostRectVector[t];				//temp == the neweset Rect in statePostRectVector
						size_t i;
						xtemp = KFTvectorl[j].statePostRectVector[t].x;
						ytemp = KFTvectorl[j].statePostRectVector[t].y;
						widthtemp = KFTvectorl[j].statePostRectVector[t].width;
						heighttemp = KFTvectorl[j].statePostRectVector[t].height;
						Rect templ;
						templ.x = xtemp - args.win_width/2;
						templ.y = ytemp - args.win_width;
						templ.width = xtemp + widthtemp + args.win_width/2 - templ.x;
						templ.height = ytemp + heighttemp + args.win_width - templ.y;
						for( i = 0; i < foundtempl.size(); ++i)
						{
							Rect rl = foundtempl[i];

							//if the measurement result's center in the rectangle of statepost,update it
							if( (( rl.x + rl.width/2 ) > templ.x) 
								&& (( rl.x + rl.width/2 ) < ( templ.x + templ.width ))
								&& (( rl.y + rl.height/2 ) > templ.y)
								&& (( rl.y + rl.height/2 ) < ( templ.y + templ.height ))	)
							{
								cout << "KFTvectorl["<<j<<"].updateKF" << endl;
								KFTvectorl[j].updateKF( rl.x, rl.y, rl.width, rl.height );
								for( size_t t = 0; t < KFTvectorl[j].statePostRectVector.size(); ++t)
								{
									Rect temp;
									temp.x = KFTvectorl[j].statePostRectVector[t].x*ratio + resx + tempx;
									temp.y = KFTvectorl[j].statePostRectVector[t].y*ratio + resy + tempy;
									temp.width = KFTvectorl[j].statePostRectVector[t].width*ratio;
									temp.height = KFTvectorl[j].statePostRectVector[t].height*ratio;
									KFTtemplts.statePostRectVector.push_back( temp ) ;						
								}
								for( size_t t = 0; t < KFTvectorl[j].measureRectVector.size(); ++t)
								{
									Rect temp;
									temp.x = KFTvectorl[j].measureRectVector[t].x*ratio + resx + tempx;
									temp.y = KFTvectorl[j].measureRectVector[t].y*ratio + resy + tempy;
									temp.width = KFTvectorl[j].measureRectVector[t].width*ratio;
									temp.height = KFTvectorl[j].measureRectVector[t].height*ratio;
									KFTtemplts.measureRectVector.push_back( temp ) ;						
								}
								KFTvectorlns.push_back(KFTtemplts);
								break;
							}								

						}

						if( i == foundtempl.size() )
						{
							
							KFTvectorl[j].addcounter();
							cout << "KFTvectorl["<<j<<"].getcounter() : "<< KFTvectorl[j].getcounter() << endl;
							if( KFTvectorl[j].getcounter()> 3)
							{
								cout << "KFTvectorl["<<j<<"].getcounter" << endl;
								KFTvectorl.erase(KFTvectors.begin()+j);
								if( j != 0)
									--j;
							}
							else
							{
								cout << "KFTvectorl["<<j<<"].updateWithoutMeasurement" << endl;
								KFTvectorl[j].updateWithoutMeasurement();
								for( size_t t = 0; t < KFTvectorl[j].statePostRectVector.size(); ++t)
								{

									Rect temp;
									temp.x = KFTvectorl[j].statePostRectVector[t].x*ratio + resx + tempx;
									temp.y = KFTvectorl[j].statePostRectVector[t].y*ratio + resy + tempy;
									temp.width = KFTvectorl[j].statePostRectVector[t].width*ratio;
									temp.height = KFTvectorl[j].statePostRectVector[t].height*ratio;

									KFTtemplts.statePostRectVector.push_back( temp ) ;						
								}
								for( size_t t = 0; t < KFTvectorl[j].measureRectVector.size(); ++t)
								{

									Rect temp;
									temp.x = KFTvectorl[j].measureRectVector[t].x*ratio + resx + tempx;
									temp.y = KFTvectorl[j].measureRectVector[t].y*ratio + resy + tempy;
									temp.width = KFTvectorl[j].measureRectVector[t].width*ratio;
									temp.height = KFTvectorl[j].measureRectVector[t].height*ratio;

									KFTtemplts.measureRectVector.push_back( temp ) ;						
								}

								KFTvectorlns.push_back(KFTtemplts);

							}
						}
					}

					//*******************check the miss rate of KalmanFilterTrackers********************


					//********************************draw lines and rectangle of results******************************************
					for(size_t i = 0; i < KFTvectors.size(); ++i)
					{
						//cout << "KFTvectors.at(i).drawPredictLine(img_to_shows);" <<endl;
						KFTvectors.at(i).changeStatusImgS(img_to_shows);						//danger or warning status
						KFTvectors.at(i).drawPredictLine(img_to_shows);						//draw results and lines
						KFTvectors.at(i).putStatusText(img_to_shows);						//put Text on rectangle of the status

					}
					for(size_t i = 0; i < KFTvectorl.size(); ++i)
					{
						//cout << "KFTvectorl.at(i).drawPredictLine(img_to_showl);" <<endl;
						KFTvectorl.at(i).changeStatusImgL(img_to_showl);
						KFTvectorl.at(i).drawPredictLine(img_to_showl);
						KFTvectorl.at(i).putStatusText(img_to_showl);
					}


					stringstream positionT, areaT, diameterT, positionTName, areaTName, diameterTName;
					ofstream outputPositionT, outputAreaT, outputDiameterT;

					positionTName << "PositionTrackingResults_" << args.width << "_" << args.win_width << ".txt";
					areaTName << "AreaTrackingResults_" << args.width << "_" << args.win_width << ".txt";
					diameterTName << "DiameterTrackingResults_" << args.width << "_" << args.win_width << ".txt";
					outputPositionT.open(positionTName.str(),ios::app);
					outputAreaT.open(areaTName.str(),ios::app);
					outputDiameterT.open(diameterTName.str(),ios::app);

					//draw rectangular results both short and long focal length on combine image
					for(size_t i = 0; i < KFTvectorlns.size(); ++i)
					{
						//cout << "KFTvectorlns.at(i).drawPredictLine(img_combine);" <<endl;
						KFTvectorlns.at(i).changeStatusImgS(img_combine);
						KFTvectorlns.at(i).drawPredictLine(img_combine);
						KFTvectorlns.at(i).putStatusText(img_combine);
						
						int temp = KFTvectorlns[i].statePostRectVector.empty() ? 0 : KFTvectorlns[i].statePostRectVector.size() - 1;;
						if( temp > 0)
						{

							Rect rls = KFTvectorlns[i].statePostRectVector.at(temp);
							positionT.str("");
							areaT.str("");
							diameterT.str("");
							positionT << count << "\t" << rls.br().y;
							outputPositionT << positionT.str() << endl;
							areaT << count << "\t" << rls.area();
							outputAreaT << areaT.str() << endl;
							diameterT << count << "\t" << (int)sqrt((double)rls.width*rls.width + rls.height*rls.height);
							outputDiameterT << diameterT.str() << endl;
						}

					}
					outputPositionT.close();
					outputAreaT.close();
					outputDiameterT.close();

					if(showMeasurementTrajectory)
					{

					}
					
					if(showboth)
					{
						//draw rectangular results both short and long focal length on combine image
						for( size_t i = 0; i < foundtemplns.size(); ++i)
						{
							//Rect rls = foundtemplns[i];
							rectangle(img_combine, foundtemplns.at(i).tl(), foundtemplns.at(i).br(), CV_RGB(255, 0, 255), 2);
						}
						//draw rectangular results of short focal length on combine image
						for( size_t i = 0; i < found_filters.size(); ++i)
						{
							//Rect rs = found_filters[i];
							rectangle(img_to_shows, found_filters.at(i).tl(), found_filters.at(i).br(), CV_RGB(255, 255, 0), 2);
						}
						//draw rectangular results of long focal length on combine image
						for( size_t i = 0; i < foundtempl.size(); ++i)
						{
							//Rect rl = found_filterl[i];
							rectangle(img_to_showl, foundtempl.at(i).tl(), foundtempl.at(i).br(), CV_RGB(0, 255, 255), 2);
						}							

					}

				}//if(!firstframe)----not the initialization step ends		


				else//if the first frame
				{
					cout<<"For the first frame to initialize KFT"<<endl;

					//draw rectangular results both short and long focal length on combine image
					for( size_t i = 0; i < foundtemplns.size(); ++i)
					{
						//Rect rls = foundtemplns[i];
						rectangle(img_combine, foundtemplns[i].tl(), foundtemplns[i].br(), CV_RGB(255, 0, 255), 2);
					}
					//draw rectangular results of short focal length on combine image
					for( size_t i = 0; i < found_filters.size(); ++i)
					{
						//Rect rs = found_filters[i];
						KalmanFilterTracker KFTtemps;
						rectangle(img_to_shows, found_filters[i].tl(), found_filters[i].br(), CV_RGB(255, 255, 0), 2);
						KFTtemps.initKF( found_filters[i].x,
											found_filters[i].y,
											found_filters[i].width,
											found_filters[i].height	);
						KFTvectors.push_back(KFTtemps);
					
					}
					//draw rectangular results of long focal length on combine image
					for( size_t i = 0; i < foundtempl.size(); ++i)
					{
						//Rect rl = found_filterl[i];
						KalmanFilterTracker KFTtempl;
						rectangle(img_to_showl, foundtempl[i].tl(), foundtempl[i].br(), CV_RGB(0, 255, 255), 2);
						KFTtempl.initKF( foundtempl[i].x,
											foundtempl[i].y,
											foundtempl[i].width,
											foundtempl[i].height );
						KFTvectorl.push_back(KFTtempl);
					}

					firstframe = !firstframe;
					cout << "firstframe changed to: " << firstframe << endl; 

				}//else-----if(firstframe)   ends

			}//if(btracking)  ends

			//*************************************tracking ends*************************************//
			
			//show detection results without tracking
			else
			{
				stringstream positionD, areaD, diameterD, positionDName, areaDName, diameterDName;
				ofstream outputPositionD, outputAreaD, outputDiameterD;

				positionDName << "PositionDetectionResults_" << args.width << "_" << args.win_width << ".txt";
				areaDName << "AreaDetectionResults_" << args.width << "_" << args.win_width << ".txt";
				diameterDName << "DiameterDetectionResults_" << args.width << "_" << args.win_width << ".txt";
				outputPositionD.open(positionDName.str(),ios::app);
				outputAreaD.open(areaDName.str(),ios::app);
				outputDiameterD.open(diameterDName.str(),ios::app);

				//draw rectangular results both short and long focal length on combine image
				for( size_t i = 0; i < foundtemplns.size(); ++i)
				{
					Rect rls = foundtemplns[i];
					rectangle(img_combine, foundtemplns[i].tl(), foundtemplns[i].br(), CV_RGB(255, 0, 255), 2);
					
					positionD.str("");
					areaD.str("");
					diameterD.str("");
					positionD << count << "\t" << rls.br().y;
					outputPositionD << positionD.str() << endl;
					areaD << count << "\t" << rls.area();
					outputAreaD << areaD.str() << endl;
					diameterD << count << "\t" << (int)sqrt((double)rls.width*rls.width + rls.height*rls.height) ;
					outputDiameterD << diameterD.str() << endl;
				}
				outputPositionD.close();
				outputAreaD.close();
				outputDiameterD.close();
				//draw rectangular results of short focal length on combine image
				for( size_t i = 0; i < found_filters.size(); ++i)
				{
					//Rect rs = found_filters[i];
					rectangle(img_to_shows, found_filters[i].tl(), found_filters[i].br(), CV_RGB(255, 255, 0), 2);
				}
				//draw rectangular results of long focal length on combine image
				for( size_t i = 0; i < found_filterl.size(); ++i)
				{
					//Rect rl = found_filterl[i];
					rectangle(img_to_showl, found_filterl[i].tl(), found_filterl[i].br(), CV_RGB(0, 255, 255), 2);
				}

			}//********************else--detection without tracking  ends

			
            if (use_gpu)
                {
					putText(img_to_shows, "Mode: GPU", Point(5, 45), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
					putText(img_to_showl, "Mode: GPU", Point(5, 45), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
				}
            else
                {
					putText(img_to_shows, "Mode: CPU", Point(5, 45), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
					putText(img_to_showl, "Mode: CPU", Point(5, 45), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
				}
            putText(img_to_shows, "FPS (HOG only): " + hogWorkFps(), Point(5, 85), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
            putText(img_to_shows, "FPS (total): " + workFps(), Point(5, 125), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
			
			putText(img_to_showl, "FPS (HOG only): " + hogWorkFps(), Point(5, 85), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
            putText(img_to_showl, "FPS (total): " + workFps(), Point(5, 125), FONT_HERSHEY_SIMPLEX, 1., Scalar(255, 100, 0), 2);
			
			//get the HOG work fps:GPU and CPU
			stringstream ssOutput, ssOutputName;
			ofstream outputGpu, outputCpu, outputTotal;
			ssOutput << count << "\t" << hog_work_fps;

			ssOutputName << "HogWorkFps_GPU_" << args.width << "_" << args.win_width << ".txt";
			outputGpu.open(ssOutputName.str(),ios::app);
			outputGpu << ssOutput.str() << endl;
			outputGpu.close();

			ssOutput.str("");
			ssOutputName.str("");
			ssOutput << count << "\t" << hog_work_fps/12;
			ssOutputName << "HogWorkFps_CPU_" << args.width << "_" << args.win_width << ".txt";
			outputCpu.open(ssOutputName.str(),ios::app);
			outputCpu << ssOutput.str() << endl;
			outputCpu.close();

			ssOutput.str("");
			ssOutputName.str("");
			ssOutput << count << "\t" << work_fps;
			if(use_SIFT)
				ssOutputName << "TotalWorkFps_SIFT_" << args.width << "_" << args.win_width << ".txt";
			else
				ssOutputName << "TotalWorkFps_" << args.width << "_" << args.win_width << ".txt";

			outputTotal.open(ssOutputName.str(),ios::app);
			outputTotal << ssOutput.str() << endl;
			outputTotal.close();

            imshow("GpuHog_Left_short", img_to_shows);
			imshow("GpuHog_Right_long", img_to_showl);
			imshow("Results combination", img_combine);

            if (args.src_is_videos && args.src_is_videol ) 
			{
				vcs >> frames;
				vcl >> framel;
			}

			//update the image for frames and framel
			if (args.src_is_cameras && args.src_is_cameral )  
			{				
				for ( unsigned int i = 0; i < numCameras; i++ )
				{
					error = ppCameras[i]->RetrieveBuffer( &image );
					if (error != PGRERROR_OK)
					{
						PrintError( error );
						throw runtime_error(string("can't retrieve image"));
					}
					temp = Mat (ConvertImageToOpenCV(&image));

					if ( i == 0 )
					{
						temp.copyTo(frames);
						//printf("get picure into frames \n");
						if(!frames.data) 
							throw runtime_error(string("can't change PGR into OpenCV image format for short"));
					}

					else
					{
						temp.copyTo(framel);
//						printf("get picure into framel \n");
						if(!framel.data) 
							throw runtime_error(string("can't change PGR into OpenCV image format for long")); 
					}
				}
			}//if (args.src_is_cameras && args.src_is_cameral )   ends
					

            workEnd();

            if (args.write_video)						//write the results into video
            {
				
                if (!video_writer_ls.isOpened())
                {
                    video_writer_ls.open(args.dst_videos, CV_FOURCC('x','v','i','d'), args.dst_video_fps,
                                      img_to_shows.size(), true);
                    if (!video_writer_ls.isOpened())
                        throw std::runtime_error("can't create video writer for ls");
                }

				if (!video_writer_rl.isOpened())
                {
                    video_writer_rl.open(args.dst_videol, CV_FOURCC('x','v','i','d'), args.dst_video_fps,
                                      img_to_showl.size(), true);
                    if (!video_writer_rl.isOpened())
                        throw std::runtime_error("can't create video writer for ls");
                }

				if (!video_writer_combine.isOpened())
                {
                    video_writer_combine.open(args.dst_video_combine, CV_FOURCC('x','v','i','d'), args.dst_video_fps,
                                      img_combine.size(), true);
                    if (!video_writer_combine.isOpened())
                        throw std::runtime_error("can't create video writer for combine_img");
                }
                if (make_gray) 
				{
					cvtColor(img_to_shows, imgs, CV_GRAY2BGR);	
					cvtColor(img_to_showl, imgl, CV_GRAY2BGR);	
					cvtColor(img_combine, img_combine_gray, CV_GRAY2BGR);
				}
                else 
				{
					cvtColor(img_to_shows, imgs, CV_BGRA2BGR);
					cvtColor(img_to_showl, imgl, CV_BGRA2BGR);
					cvtColor(img_combine, img_combine_gray, CV_BGRA2BGR);
				}

                video_writer_ls << imgs;
				video_writer_rl << imgl;
				video_writer_combine << img_combine_gray;
            }//if (args.write_video)	ends

            handleKey((char)waitKey(10));

			//if(count >= 200)
				//running = false;
        }//while (running && !frames.empty() && !framel.empty())   ends

		video_writer_ls.release();
		video_writer_rl.release();
		video_writer_combine.release();
		for ( unsigned int i = 0; i < numCameras; i++ )
		{
			ppCameras[i]->StopCapture();
			ppCameras[i]->Disconnect();
			delete ppCameras[i];
		}

		delete [] ppCameras;

		running = false;

    }//while(running)  ends

}

void App::handleKey(char key)
{
    switch (key)
    {
    case 27:
        running = false;
        break;
	case 's':
	case 'S':
		use_SIFT = !use_SIFT;
		cout << "Switched to " << (use_SIFT ? "SIFT Elimination" : "Regular Eliminatoin") << " mode\n";
		break;
	case 'l':
	case 'L':
		detectSign = !detectSign;
		cout << "Switched to " << (detectSign ? "Has traffic sign detection" : "No traffic sign detection") << " mode\n";
		break;
	case 'v':
	case 'V':
		args.write_video = !args.write_video;
		cout << "Switched to " << (args.write_video ? "write video" : "no write video") << " mode\n";
		break;
    case 'u':
    case 'U':
        use_gpu = !use_gpu;
        cout << "Switched to " << (use_gpu ? "CUDA" : "CPU") << " mode\n";
        break;
	case 't':
    case 'T':
        btracking = !btracking;
		cout << "Switched to " << (btracking ? "tracking" : "untracking") << " mode\n";
        break;
	case 'b':
    case 'B':
        showboth = !showboth;
		cout << "Switched to " << (showboth ? "showboth" : "showtracking") << " mode\n";
        break;
    case 'g':
    case 'G':
        make_gray = !make_gray;
        cout << "Convert image to gray: " << (make_gray ? "YES" : "NO") << endl;
        break;
    case '1':
        scale *= 1.05;
        cout << "Scale: " << scale << endl;
        break;
    case 'q':
    case 'Q':
        scale /= 1.05;
        cout << "Scale: " << scale << endl;
        break;
    case '2':
        nlevels++;
        cout << "Levels number: " << nlevels << endl;
        break;
    case 'w':
    case 'W':
        nlevels = max(nlevels - 1, 1);
        cout << "Levels number: " << nlevels << endl;
        break;
    case '3':
        gr_threshold++;
        cout << "Group threshold: " << gr_threshold << endl;
        break;
    case 'e':
    case 'E':
        gr_threshold = max(0, gr_threshold - 1);
        cout << "Group threshold: " << gr_threshold << endl;
        break;
    case '4':
        hit_threshold+=0.25;
        cout << "Hit threshold: " << hit_threshold << endl;
        break;
    case 'r':
    case 'R':
        hit_threshold = max(0.0, hit_threshold - 0.25);
        cout << "Hit threshold: " << hit_threshold << endl;
        break;
    case 'c':
    case 'C':
        gamma_corr = !gamma_corr;
        cout << "Gamma correction: " << gamma_corr << endl;
        break;
    case 'z':
    case 'Z':
        firstframe = !firstframe;
        cout << "Clear tracking results and Re-tracking, firstframe is: " << firstframe << endl;
        break;
    case 'm':
    case 'M':
        showMeasurementTrajectory = !showMeasurementTrajectory;
        cout << "Show measurement trajectory: " << showMeasurementTrajectory << endl;
        break;
		
		
    }
}


inline void App::hogWorkBegin() { hog_work_begin = getTickCount(); }

inline void App::hogWorkEnd()
{
    int64 delta = getTickCount() - hog_work_begin;
    double freq = getTickFrequency();
    hog_work_fps = freq / delta;
}

inline string App::hogWorkFps() const
{
    stringstream ss;
    ss << hog_work_fps;
    return ss.str();
}


inline void App::workBegin() { work_begin = getTickCount(); }

inline void App::workEnd()
{
    int64 delta = getTickCount() - work_begin;
    double freq = getTickFrequency();
    work_fps = freq / delta;
}

inline string App::workFps() const
{
    stringstream ss;
    ss << work_fps;;
    return ss.str();
}

