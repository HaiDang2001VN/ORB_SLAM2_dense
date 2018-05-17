#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;
int main(int argc,char** argv)
{
//Read the video file
cout<<argv[1]<<endl;
VideoCapture cap(argv[1]);
if (!cap.isOpened())
{
	cout << "Unable to read video!" << endl;
	exit(1);
}
namedWindow("frame");
Mat frame;
int findex=0;
while(cap.read(frame))
{
	//imshow("frame",frame);
	string kfstrFile; //set path for saving
	ostringstream kfstrFilen;
	kfstrFilen << kfstrFile<<"./images/"<<findex << ".jpg";
	if(findex%5==0)
	{
		imwrite(kfstrFilen.str(),frame);
	}
	findex++;
	if (cv::waitKey(2) == 'q')
	{
		break;
	}
}
return 0;
}
