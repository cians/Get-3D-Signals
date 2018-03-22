#include "Parameter.h"

class RectCalib
{
public:
    RectCalib(ParameterReader pd):pReader(pd){

    }
    cv::Rect LampCalib(cv::Rect ori, cv::Mat img)
    {
        using namespace cv;
    // cout<<ori.width<<"."<<ori.height<<endl;
        if(ori.width < 20 && ori.height < 40)
            return ori;
        int expand = 6;
        cv::Rect res(max(0,ori.x-expand), max(0,ori.y-expand), ori.width + 2*expand, ori.height+2*expand);
        res.width = res.x+res.width > img.cols ? img.cols-res.x : res.width;
        res.height = res.y+res.height > img.rows ? img.rows-res.height : res.height;
        cv::Mat img_cand = img(res);
        blur( img_cand, img_cand, Size(3,3));
        // imshow("tmp",img_cand);
        // waitKey();
        Mat src_gray;
        cvtColor( img_cand, src_gray, COLOR_BGR2GRAY );
    // Mat threshold_output;
    //  vector<vector<cv::Point> > contours;
    //  vector<Vec4i> hierarchy;
        //threshold( src_gray, threshold_output, 100, 255, THRESH_BINARY );
        Canny(src_gray, src_gray, 50, 100);
        //imshow("sss",src_gray);
        //保留1/4
        //int splitc = (int)(0.25 * src_gray.cols);
        int splitr = (int)(0.25 * src_gray.rows);
    // src_gray.colRange(splitc, src_gray.cols-splitc).setTo(0);
        src_gray.rowRange(splitr, src_gray.rows-splitr).setTo(0);
        // findContours( src_gray, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        //imshow("sss",src_gray);
        // Mat imageContours=Mat::zeros(src_gray.size(),CV_8UC1);
        // for( size_t i = 0; i< contours.size(); i++ )
        // {
        //     drawContours( imageContours, contours, (int)i, Scalar(255), 1, 8, hierarchy);
        // }
        //  imshow("tmp",imageContours);
        cv::Rect r0= boundingRect(src_gray);  
    // cv::rectangle(img_cand, r0, Scalar(255,0,0),2); 
        //cv::rectangle(img_cand, ori, Scalar(0,255,0),2);
    // imshow("tmp2",img_cand);
    if(r0.height < r0.width || r0.height < 20)
            r0.height = ori.height;
    if(r0.width < 10)
            r0.width = ori.width;
        //waitKey();
        return cv::Rect(r0.x+res.x, r0.y+res.y, r0.width, r0.height);
    }
    // void MultiviewTriangulation(std::vector < Eigen::Vector2f > &img_coords, Eigen::Matrix4d CAM_P, std::vector<Eigen::Matrix<float, 3, 4> > &cam_P_ext, Eigen::Vector3f &XYZ)
    // {
    // 	assert(img_coords.size() == cam_P_ext.size());
    // 	const size_t num = img_coords.size();
    // 	Eigen::MatrixX3f T;
    // 	Eigen::VectorXf p;
    // 	T.resize(2*num, 3);
    // 	p.resize(2 * num);
    // 	for (size_t i = 0; i < num; i++)
    // 	{
    // 		//Eigen::Matrix<float, 3, 4> CAM_P = cam_intrinsic.INTRINSIC*cam_P_ext[i];
    // 		T.block<1, 3>(2*i, 0) = CAM_P.block<1, 3>(0, 0) - img_coords[i](0) * CAM_P.block<1, 3>(2, 0);
    // 		T.block<1, 3>(2*i+1, 0) = CAM_P.block<1, 3>(1, 0) - img_coords[i](1) * CAM_P.block<1, 3>(2, 0);
    // 		p(2 * i) = CAM_P(0, 3) - img_coords[i](0) * CAM_P(2, 3);
    // 		p(2*i+1) = CAM_P(1, 3) - img_coords[i](1) * CAM_P(2, 3);
    // 	}
    // 	XYZ = (T.transpose()*T).colPivHouseholderQr().solve(-T.transpose()*p);
    // 	//cout << XYZ << endl;
    // }
    vector<cv::Rect> RebuildLamp (int index, cv::Mat CurImg, ParameterReader pd, vector<Eigen::Vector3f> LampLocation)
    {
        vector<cv::Rect> LampRects;
        if(pd.getLabels(index).size() < 1){
            
        }
        return LampRects;
    }

int run()
{
        cv::VideoCapture imgShowSeque;
        string img_path = pReader.getData("img_path");
        //imgShowSeque.open(img_path + "157_%05d.png");
        // if (!imgShowSeque.isOpened())
        // {
        // 	cerr << "can not find Image sequence!\n";
        // 	return -1;
        // }
        int drawImgsNum = 0;
        int rec_start = stoi(pReader.getData("rec_start"));
        bool playvideo = true;

        while(drawImgsNum < 600){
                //一帧帧显示图像
                cv::Mat img,img_calib;	
                //imgShowSeque >> img;
                stringstream numberss;
                numberss << std::setw(5) << std::setfill('0') << rec_start+drawImgsNum; // 0000, 0001, 0002, etc...
                std::string name = img_path + "157_" +numberss.str() + ".png";
                img = cv::imread(name);
                img_calib = img.clone();
                if (img.empty())
                    return 0;
                cv::Mat showImg = img.clone(), showImg_calib = img_calib.clone();
                auto Labels = pReader.getLabels(drawImgsNum + rec_start);
                for (auto lab : Labels)
                {
                    cv::Rect lamp = lab.position;
                    cv::rectangle(showImg, lamp, cv::Scalar(0,0,255), 2);
                    cv::Rect lamp_calib = LampCalib(lab.position, img);
                    cv::rectangle(showImg, lamp_calib, cv::Scalar(255,0,0), 2);
                }
                resize(showImg, showImg, cv::Size(968, 768));
                resize(showImg_calib, showImg_calib, cv::Size(968, 768));
                cv::imshow("original", showImg);
                //cv::imshow("rectified", showImg_calib);

                //cv::moveWindow("video", 868, 0);
                //imshow时间太短，movewindow好像没用，只好暂停手动调整下窗口位置便于录屏。
                // if (drawImgsNum < 1)
                char key = cv::waitKey(30);
                if(key == 'p') playvideo = !playvideo;
                if(playvideo){
                // cout<<drawImgsNum+rec_start<<endl;
                    drawImgsNum++;
                }
            }
        return 0;
    }
public:
    ParameterReader pReader;
    // Lamp
    vector < Eigen::Vector3f > LampLocation;
    vector < Eigen::Vector2f > Lamp_cands;
};

int main(int argc, char** argv){
    ///就一个参数，参数文件位置
    if (argc < 2){
        cerr<<"arguments missing!\n";
        return -1;
    }
    ParameterReader pReader(argv[1]);
    RectCalib RC(pReader);
    RC.run();
    return 0;
}