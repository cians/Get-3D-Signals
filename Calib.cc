#include "Parameter.h"

class RectCalib
{
public:
    RectCalib(ParameterReader pd):pReader(pd){}
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
    vector<cv::Rect> updateSignals(int index)
    {
        vector<cv::Rect> SignalRects;
        for(size_t j = 0; j < vSignals.size(); ++j){
            if(vSignals[j].ObsLabels.size() > 4){
                const size_t num = vSignals[j].ObsLabels.size();
                Eigen::MatrixX3f Tl;
                Eigen::VectorXf pl;
                Eigen::MatrixX3f Tr;
                Eigen::VectorXf pr;
                Tl.resize(2*num, 3); 
                Tr.resize(2*num, 3);
                pl.resize(2*num);
                pr.resize(2*num);
                for (size_t i = 0; i < num; i++)
                {
                    auto label = vSignals[j].ObsLabels[i];
                    size_t ti = label.frameIndex - rec_start;
                    Eigen::Matrix<float, 3, 4> CAM_P = pReader.cam_intrinsic.INTRINSIC*pReader.trajectory[ti];
                    Eigen::Vector2i leftu(label.position.x, label.position.y);
                    Eigen::Vector2i rightd(label.position.x+label.position.width, label.position.y+label.position.height); 
                    Tl.block<1, 3>(2*i, 0) = CAM_P.block<1, 3>(0, 0) - leftu(0) * CAM_P.block<1, 3>(2, 0);
                    Tl.block<1, 3>(2*i+1, 0) = CAM_P.block<1, 3>(1, 0) - leftu(1) * CAM_P.block<1, 3>(2, 0);
                    pl(2 * i) = CAM_P(0, 3) - leftu(0) * CAM_P(2, 3);
                    pl(2*i+1) = CAM_P(1, 3) - leftu(1) * CAM_P(2, 3);
                    Tr.block<1, 3>(2*i, 0) = CAM_P.block<1, 3>(0, 0) - rightd(0) * CAM_P.block<1, 3>(2, 0);
                    Tr.block<1, 3>(2*i+1, 0) = CAM_P.block<1, 3>(1, 0) - rightd(1) * CAM_P.block<1, 3>(2, 0);
                    pr(2 * i) = CAM_P(0, 3) - rightd(0) * CAM_P(2, 3);
                    pr(2*i+1) = CAM_P(1, 3) - rightd(1) * CAM_P(2, 3);
                }
                Eigen::Vector3f leftpos = (Tl.transpose()*Tl).colPivHouseholderQr().solve(-Tl.transpose()*pl);
                Eigen::Vector3f rightpos =  (Tr.transpose()*Tr).colPivHouseholderQr().solve(-Tr.transpose()*pr);
                //printf("leftu: %f %f %f, rightd: %f %f %f\n",leftpos(0),leftpos(1),leftpos(2),rightpos(0),rightpos(1),rightpos(2));
                //TODO:检查是否相差太远
                vSignals[j].LeftUpPos = leftpos;
                vSignals[j].RightDownPos = rightpos;
                //投影回来。
                Eigen::Vector3f CamBack = pReader.trajectory[index].block<3,1>(0,2);
                Eigen::Vector3f c2l = leftpos - pReader.trajectory[index].block<3,1>(0,3);
                if (c2l.dot(CamBack)/(c2l.norm()*CamBack.norm()) > -0.5){
                    printf("a passed lamp|\n");
                    continue;
                }
                auto cam_p = pReader.cam_intrinsic.INTRINSIC*pReader.trajectory[index];
                cv::Rect curRect = reproject2img(leftpos,rightpos,cam_p);
                vSignals[j].CurFrameRect = curRect;
                SignalRects.push_back(curRect);
            }
        }
        printf("total %zd signals\n",SignalRects.size());
        return SignalRects;
    }
    cv::Rect reproject2img(Eigen::Vector3f LeftUpPos, Eigen::Vector3f RightDownPos,Eigen::Matrix<float,3,4>cam_p){
        Eigen::Vector4f Lamp4f;
        //left up 
        Lamp4f.segment(0,3) = LeftUpPos;
        Lamp4f(3) = 1;
        auto left3f = cam_p*Lamp4f;
        Eigen::Vector2i Left2i((int)(left3f(0)/left3f(2)), (int)(left3f(1)/left3f(2)));
       // printf("imgcoord2i Left: %d,%d\n",Left2i(0),Left2i(1));
        // right down
        Lamp4f.segment(0,3) = RightDownPos;
        auto Right3f = cam_p*Lamp4f;
        Eigen::Vector2i Right2i((int)(Right3f(0)/Right3f(2)), (int)(Right3f(1)/Right3f(2)));
       // printf("imgcoord2i Right: %d,%d\n",Right2i(0),Right2i(1));
        cv::Rect curRect(Left2i(0),Left2i(1),Right2i(0)-Left2i(0),Right2i(1)-Left2i(1));
        return curRect;
    }

    vector<cv::Rect> RebuildLamp (int index, cv::Mat& CurImg)
    {
        vector<cv::Rect> LastLampRects;
        for(size_t i = 0; i < vSignals.size(); ++i){//投影当前三维灯到图像上来
            if(vSignals[i].ObsLabels.size() > 4){
                Eigen::Vector3f CamBack = pReader.trajectory[index].block<3,1>(0,2);
                Eigen::Vector3f c2l = vSignals[i].LeftUpPos - pReader.trajectory[index].block<3,1>(0,3);
                if (c2l.dot(CamBack)/(c2l.norm()*CamBack.norm()) > -0.5)
                    continue;
                auto cam_p = pReader.cam_intrinsic.INTRINSIC*pReader.trajectory[index];
                cv::Rect curRect = reproject2img(vSignals[i].LeftUpPos,vSignals[i].RightDownPos,cam_p);
                vSignals[i].CurFrameRect = curRect;
                LastLampRects.push_back(curRect);
            }
        }
      //  trajectory[index];
        if(!pReader.getLabels(index+rec_start)){ 
            return LastLampRects;
        } else {// 有lable的话判断label归类。
            vector<Label> *Labels = pReader.getLabels(index+rec_start);       
            for(size_t li = 0; li < Labels->size(); ++li){
                Label &label = (*Labels)[li];
                bool NoParent = true;
                for(int i = 1; i < 4 && NoParent; ++i){// 往前面查寻三帧
                    if(index >= i) {
                        vector<Label> *OldLabels = pReader.getLabels(index-i+rec_start);
                        if(OldLabels)
                        for (size_t oi = 0; oi < OldLabels->size() && NoParent;++oi){
                            Label &oldlab = (*OldLabels)[oi];
                            if((float)abs(oldlab.position.x - label.position.x)<0.4*label.position.width &&
                               (float)abs(oldlab.position.y - label.position.y)<0.5*label.position.height){
                                NoParent = false;
                                //加入队列
                                label.SID = oldlab.SID;
                                vSignals[oldlab.SID].ObsLabels.push_back(label);
                                break;
                              }
                        }
                    }
                } 
                if(NoParent){
                    for(size_t j = 0; j < vSignals.size() && NoParent; ++j){
                        if(vSignals[j].ObsLabels.size() >= 4){
                            auto &lamprect = vSignals[j].CurFrameRect;
                            if((float)abs(lamprect.x-label.position.x) < 0.5*lamprect.width &&
                               (float)abs(lamprect.y-label.position.y) < 0.5*lamprect.height){
                                NoParent = false;
                                //加入优化队列
                                label.SID = vSignals[j].SID;
                                vSignals[j].ObsLabels.push_back(label);
                                break;
                            }
                        }
                    }
                }
               if(NoParent){
                   //当做新来的灯。
                   printf("new a signal!\n");
                   Signal signal;
                   signal.startFrame = index;
                   signal.SID = (int)vSignals.size();
                   label.SID = signal.SID;// LabelData 里没有生效
                   signal.ObsLabels.push_back(label);
                   vSignals.push_back(signal);
               }
            }
        auto LampRects = updateSignals(index);
        return LampRects;
        }
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
        bool playvideo = true;
        rec_start = stoi(pReader.getData("rec_start"));
        while(drawImgsNum < 600){
                //一帧帧显示图像
                cv::Mat img,img_calib;	
                //imgShowSeque >> img;
                stringstream numberss;
                numberss << std::setw(5) << std::setfill('0') << rec_start+drawImgsNum; // 0000, 0001, 0002, etc...
                std::string name = img_path + "157_" +numberss.str() + ".png";
                img = cv::imread(name);
                auto camm = pReader.cam_intrinsic;
                cv::Mat camMatrix = (cv::Mat_<float>(3,3)<<camm.fx, 0, camm.cx, 
                                                        0, camm.fy, camm.cy,
                                                        0 , 0, 1);
                cv::Mat distCoeffs = cv::Mat(5,1,CV_32F,camm.distortion);
                cv::undistort(img, img_calib, camMatrix, distCoeffs);
                img = img_calib.clone();
               // img_calib = img.clone();
                if (img.empty())
                    return 0;
                cv::Mat showImg = img.clone(), showImg_calib = img_calib.clone();
                vector<Label> *Labels = pReader.getLabels(drawImgsNum + rec_start);
                vector<cv::Rect> RectsOri;
                if(Labels)
                    for (Label &lab : *Labels)
                    {
                        //rectify lab
                        using namespace cv;
                        std::vector<cv::Point2f> srcPoints,dstPoints;
                        srcPoints.push_back(cv::Point2f(lab.position.x, lab.position.y));
                        srcPoints.push_back(cv::Point2f(lab.position.x+lab.position.width, lab.position.y+lab.position.height));
                        cv::undistortPoints(srcPoints,dstPoints,camMatrix,distCoeffs);
                        dstPoints[0].x = dstPoints[0].x * camm.fx + camm.cx;
                        dstPoints[0].y = dstPoints[0].y * camm.fy + camm.cy;
                        dstPoints[1].x = dstPoints[1].x * camm.fx + camm.cx;
                        dstPoints[1].y = dstPoints[1].y * camm.fy + camm.cy;
                        lab.position.x = static_cast<int> (dstPoints[0].x);
                        lab.position.y = static_cast<int> (dstPoints[0].y);
                        lab.position.width = static_cast<int> (dstPoints[1].x - dstPoints[0].x);
                        lab.position.height = static_cast<int> (dstPoints[1].y - dstPoints[0].y);
                        if(lab.position.x < 0 || lab.position.y < 0 || dstPoints[1].x > img.cols || dstPoints[1].y > img.rows)
                            continue;
                        cv::rectangle(showImg, lab.position, cv::Scalar(0,255,0), 1);
                        RectsOri.push_back(lab.position);
                        cv::Rect lamp_calib = LampCalib(lab.position, img);
                        cv::rectangle(showImg, lamp_calib, cv::Scalar(0,0,255), 2);
                    }
                auto Rects = RebuildLamp(drawImgsNum, img);
                if(Rects.size() == 0) Rects = RectsOri;
                for(auto rect : Rects)
                {
                    cv::rectangle(showImg_calib,rect,cv::Scalar(255,0,0),2);
                }
                resize(showImg, showImg, cv::Size(968, 768));
                resize(showImg_calib, showImg_calib, cv::Size(968, 768));
                cv::imshow("original", showImg);
                //cv::imshow("rectified", showImg_calib);

                //cv::moveWindow("video", 868, 0);
                //imshow时间太短，movewindow好像没用，只好暂停手动调整下窗口位置便于录屏。
                // if (drawImgsNum < 1)
                char key = cv::waitKey(100);
                if(key == 'p') playvideo = !playvideo;
                if(playvideo){
                    cout<<drawImgsNum+rec_start<<endl;
                    drawImgsNum++;
                }
            }
        return 0;
    }
public:
    int rec_start;
    ParameterReader pReader;
    vector < Signal > vSignals;
};


//
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