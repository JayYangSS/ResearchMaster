#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace std;
using namespace cv;

void generateFile(int label, vector<float>& fv, ofstream& HOGfeature_SVMl);

int main(int argc, const char* argv[])
{
	ofstream log;
	log.open("featureVectorExtractionLog");
	ofstream HOGfeature_SVMl;

    int label;
	int width = 80;									//16,24,32,40,48,56,64,72,80,96
	int height = width*2;								//width*2
	int num = 10;

	const char* path_pos;
	const char* path_neg;

	if(argc == 6)
    {
		path_pos = argv[1];
		path_neg = argv[3];
		HOGfeature_SVMl.open (argv[5]);

	}
	else if(argc == 9)
    {
		path_pos = argv[1];
		path_neg = argv[3];
		HOGfeature_SVMl.open (argv[5]);
		num = atoi(argv[6]);
		width = atoi(argv[7]);
		height = atoi(argv[8]);
	}
	else
	{
		fprintf(stderr, "ERROR: arguments not appropriate\n");

		cout << "argv[1]: directory of the pos image\n\
				ex:		../../image/\n" << endl;
		cout <<	"argv[2]: file which contains the names of positive images" << endl; 
		cout << "		01.png" << endl; 
		cout << "		02.png" << endl;
		cout << "		..." << endl;
		cout << "argv[3]: directory of the neg image\n\
				ex:		../../image/\n" << endl;
		cout << "argv[4]: file which contains the names of negtive images" << endl;
		cout << "argv[5]: file to store the feature vector, used to feed svm light" << endl;
		cout << "argv[6]: # of random patches from negtive images, default to 10" << endl;
		cout << "argv[7]: width, default to 64" << endl;
		cout << "argv[8]: heigth, default to 128" << endl;
		system("PAUSE");
		return -1;
	}


//	HOGDescriptor hog;		//HOGDescriptor(Size win_size=Size(64, 128), Size block_size=Size(16, 16), Size block_stride=Size(8, 8), Size cell_size=Size(8, 8), int nbins=9, double win_sigma=DEFAULT_WIN_SIGMA, double threshold_L2hys=0.2, bool gamma_correction=true, int nlevels=DEFAULT_NLEVELS)

	HOGDescriptor hog(Size(width, height), Size(16, 16), Size(8, 8), Size(8, 8), 9, 1, -1, HOGDescriptor::L2Hys, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS);


	for(int i=0; i<2; i++)	//for pos and neg
	{

		string filename;
		ifstream inputList;

		if (i == 0)
		{
			inputList.open(argv[2]);
			label = 1;
		}
		else if (i == 1)
		{
			inputList.open(argv[4]);
			label = -1;
		}
		
		while (getline(inputList,filename))
		{
			char* path_filename = new char[200];		
			if (label == 1)
				sprintf (path_filename,"%s%s", path_pos, filename.c_str());
			else
				sprintf (path_filename,"%s%s", path_neg, filename.c_str());


			Mat img = imread(path_filename,1);		
			if (!img.data)
			{
				fprintf( stderr, "ERROR: no image data in file %s \n", path_filename);
				continue;
			}


			// choose ROI and compute HOG 
			int src_width = img.cols;
			int src_height = img.rows;

			if (label == 1)	//central ROI
			{

				if ((src_width < width) || (src_height < height))
					continue;
				
				Mat dst = img(Rect( (src_width-width)/2, (src_height-height)/2, width, height ));	

				vector<float> fv;		//store feature vector
				hog.compute(dst, fv,Size(8,8), Size(0,0)); //trainImg代表输入的图片（此处尺寸为64*128），descriptors表示保存特征结果的Vector，Size(64,48)表示windows的步进，第四个为padding，用于填充图片以适应大小。
				generateFile(label, fv, HOGfeature_SVMl);
				cout << label << "	" << "feature vector generated for " << path_filename << endl;
				log << label << "	" << "feature vector generated for " << path_filename << endl; 
			}
			else if (label == -1)
			{
				srand((unsigned)time(0));

				if ((src_width < width) || (src_height < height))
					continue;
		
				for(int i=0;i<num;i++)		//randomly choose num of ROI
				{

					int delta_w = src_width-width;
					int delta_h = src_height-height;

					int cutx = 0;
					int cuty = 0;

					if (delta_w != 0)
						cutx = rand()%(delta_w);
					
					if (delta_h != 0)
						 cuty = rand()%(delta_h);
					Mat dst = img(Rect( cutx, cuty, width, height ));

					vector<float> fv;
					hog.compute(dst, fv,Size(8,8), Size(0,0));
					generateFile(label, fv, HOGfeature_SVMl);
					cout << label << "	" << "feature vector generated for " << path_filename << "###" << i << endl;
					log << label << "	" << "feature vector generated for " << path_filename << "###" << i << endl;
				}
			}
			else 
			{
				cout << "label wrong!" << endl;
				return -1;
			}
			delete[] path_filename;
		}


		inputList.close();
			
	}

	log.close();

	HOGfeature_SVMl.close();

	system("pause");
	return 0;
}

void generateFile(int label, vector<float>& fv, ofstream& file)
{
	file << label << " ";
		
	for (int i=0; i<fv.size(); i++)
	{
		file << i+1 << ":" << fv[i] << " ";
	}
	file << endl;
}
