/*!
 * @brief 
 * the project is aiming to triangulate the traffic signals with known camera pose and signals' image coordinates.
 * @file CalibLight.cc
 * @author CaoJian
 * @date 2018-04-02
 */
#include "Parameter.h"

class RectCalib
{
public:
    RectCalib(ParameterReader pd){
        pReader = pd;
        undistortedImg = pReader.getData("isUndistorted")=="yes";
        traj_start = stoi(pReader.getData("traj_start"));
        lightw = stof(pReader.getData("light_width_half"));
        lighth = stof(pReader.getData("light_height_half"));
        camm = pReader.camParams;
    }
    /*!
    *this function(LampCalib) is designed to calibrate the lamp's rectangle, but is abandoned now because of its poor robustness...
    */
    // cv::Rect LampCalib(cv::Rect oriRect, cv::Mat img)
    // {
    //     using namespace cv;
    //     if(oriRect.width < 20 && oriRect.height < 40)
    //         return oriRect;
    //     int expand = 6;
    //     cv::Rect epRect(max(0,oriRect.x-expand), max(0,oriRect.y-expand), oriRect.width + 2*expand, oriRect.height+2*expand);
    //     epRect.width = epRect.x+epRect.width > img.cols ? img.cols-epRect.x : epRect.width;
    //     epRect.height = epRect.y+epRect.height > img.rows ? img.rows-epRect.height : epRect.height;
    //     cv::Mat img_cand = img(epRect);
    //     blur( img_cand, img_cand, Size(3,3));
    //     // imshow("tmp",img_cand);
    //     // waitKey();
    //     Mat src_gray;
    //     cvtColor( img_cand, src_gray, COLOR_BGR2GRAY );
    //     // Mat threshold_output;
    //     //  std::vector<std::vector<cv::Point> > contours;
    //     //  std::vector<Vec4i> hierarchy;
    //     //threshold( src_gray, threshold_output, 100, 255, THRESH_BINARY );
    //     Canny(src_gray, src_gray, 50, 100);
    //     //imshow("sss",src_gray);
    //     //保留1/4的角落
    //     //int splitc = (int)(0.25 * src_gray.cols);
    //     int splitr = (int)(0.25 * src_gray.rows);
    // // src_gray.colRange(splitc, src_gray.cols-splitc).setTo(0);
    //     src_gray.rowRange(splitr, src_gray.rows-splitr).setTo(0);
    //     // findContours( src_gray, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    //     //imshow("sss",src_gray);
    //     // Mat imageContours=Mat::zeros(src_gray.size(),CV_8UC1);
    //     // for( size_t i = 0; i< contours.size(); i++ )
    //     // {
    //     //     drawContours( imageContours, contours, (int)i, Scalar(255), 1, 8, hierarchy);
    //     // }
    //     //  imshow("tmp",imageContours);
    //     cv::Rect myRect= boundingRect(src_gray);  
    // // cv::rectangle(img_cand, myRect, Scalar(255,0,0),2); 
    //     //cv::rectangle(img_cand, oriRect, Scalar(0,255,0),2);
    // // imshow("tmp2",img_cand);
    // if(myRect.height < myRect.width || myRect.height < 20)
    //         myRect.height = oriRect.height;
    // if(myRect.width < 10)
    //         myRect.width = oriRect.width;
    //     //waitKey();
    //     return cv::Rect(myRect.x+epRect.x, myRect.y+epRect.y, myRect.width, myRect.height);
    // }
    std::vector<cv::Rect> updateSignals(int FrameID, cv::Mat &CurImg, Eigen::Matrix4f Tcw){
        std::vector<cv::Rect> SignalRects;
        for(size_t j = 0; j < vSignals.size(); ++j){
            cv::Rect curRect;
            const size_t obsNum = vSignals[j].ObsLabels.size();
            if (obsNum > 1 && vSignals[j].ObsLabels[obsNum-1].frameIndex == (size_t)FrameID){
                int d_x = (vSignals[j].ObsLabels[obsNum-1].position.x + (vSignals[j].ObsLabels[obsNum-1].position.width>>1)
                         - vSignals[j].CurFrameRect.x - (vSignals[j].CurFrameRect.width>>1));
                int d_y = (vSignals[j].ObsLabels[obsNum-1].position.y + (vSignals[j].ObsLabels[obsNum-1].position.height>>1)
                         - vSignals[j].CurFrameRect.y - (vSignals[j].CurFrameRect.height>>1));
                vSignals[j].speed(0) = d_x;
                vSignals[j].speed(1) = d_y;
                vSignals[j].CurFrameRect = vSignals[j].ObsLabels[obsNum-1].position;
               // printf("show me dx: %d dy: %d \n",d_x, d_y);
            }
            if (obsNum > 2){
                Eigen::MatrixX3f Tl;
                Eigen::VectorXf pl;
                Eigen::MatrixX3f Tr;
                Eigen::VectorXf pr;
                Tl.resize(2*obsNum, 3); 
                Tr.resize(3*obsNum, 3);
                pl.resize(2*obsNum);
                pr.resize(3*obsNum);
                for (size_t i = 0; i < obsNum; i++){
                    auto label = vSignals[j].ObsLabels[i];
                    size_t ti = label.frameIndex - rec_start;
                    Eigen::Matrix3f normFactor;
                    normFactor<<(1/CurImg.cols),0,0,
                                0,(1/CurImg.rows),0,
                                0,0,1;
                    Eigen::Matrix4f T = pReader.trajectory[traj_start+ti].inverse();
                    Eigen::Matrix3f R = T.block<3,3>(0,0);
                    Eigen::Matrix<float, 3, 3> KR = pReader.camParams.INTRINSIC * R;
                    Eigen::Vector2f uv((label.position.x+(label.position.width>>1)), (label.position.y+(label.position.height>>1))); 
                    //KR(1c) - uR(3c) Xw = t3u - K(1c)t
                    Tr.block<1, 3>(3*i, 0)  = KR.row(0) - uv(0)*R.row(2);
                    Tr.block<1, 3>(3*i+1, 0)= KR.row(1) - uv(1)*R.row(2);
                    Tr.block<1, 3>(3*i+2, 0) = KR.row(2) - R.row(2);
                    pr( 3*i) = T(2,3)*uv(0) - pReader.camParams.INTRINSIC.row(0) * T.block<3,1>(0,3);
                    pr (3*i+1) = T(2,3)*uv(1) - pReader.camParams.INTRINSIC.row(1) * T.block<3,1>(0,3);
                    pr (3*i+2) = T(2,3) - pReader.camParams.INTRINSIC.row(2) * T.block<3,1>(0,3);
                    // Eigen::Vector2i leftu(label.position.x, label.position.y);
                    // Eigen::Vector2i rightd(label.position.x+label.position.width, label.position.y+label.position.height); 
                    // Tl.block<1, 3>(2*i, 0) = CAM_P.block<1, 3>(0, 0) - leftu(0) * CAM_P.block<1, 3>(2, 0);
                    // Tl.block<1, 3>(2*i+1, 0) = CAM_P.block<1, 3>(1, 0) - leftu(1) * CAM_P.block<1, 3>(2, 0);
                    // pl(2 * i) = CAM_P(0, 3) - leftu(0) * CAM_P(2, 3);
                    // pl(2*i+1) = CAM_P(1, 3) - leftu(1) * CAM_P(2, 3);
                    // Tr.block<1, 3>(2*i, 0) = CAM_P.block<1, 3>(0, 0) - rightd(0) * CAM_P.block<1, 3>(2, 0);
                    // Tr.block<1, 3>(2*i+1, 0) = CAM_P.block<1, 3>(1, 0) - rightd(1) * CAM_P.block<1, 3>(2, 0);
                    // pr(2 * i) = CAM_P(0, 3) - rightd(0) * CAM_P(2, 3);
                    // pr(2*i+1) = CAM_P(1, 3) - rightd(1) * CAM_P(2, 3);
                }
               // Eigen::Vector3f medpos = (Tl.transpose()*Tl). colPivHouseholderQr().solve(-Tl.transpose()*pl);
                Eigen::Vector3f medpos = (Tr.transpose()*Tr). colPivHouseholderQr().solve(Tr.transpose()*pr);
                printf(" light : %f %f %f \n", medpos(0),medpos(1), medpos(2));

                //Eigen::Vector3f leftpos = (Tl.transpose()*Tl).colPivHouseholderQr().solve(-Tl.transpose()*pl);
                // Eigen::Vector3f rightpos =  (Tr.transpose()*Tr).colPivHouseholderQr().solve(-Tr.transpose()*pr);
                //printf("leftu: %f %f %f, rightd: %f %f %f\n",leftpos(0),leftpos(1),leftpos(2),rightpos(0),rightpos(1),rightpos(2));
                //TODO:灯的实际大小应该是个固定值。。。
                Eigen::Vector3f leftpos(medpos(0)-lightw, medpos(1)-lighth, medpos(2));
                Eigen::Vector3f rightpos(medpos(0)+lightw, medpos(1)+lighth, medpos(2));
                // float errors = 0;
                // for(size_t i = 0; i < obsNum; ++i){
                //      auto cam_p = pReader.camParams.INTRINSIC*pReader.trajectory[vSignals[j].ObsLabels[i].frameIndex-rec_start+traj_start].inverse().block<3,4>(0,0);
                //      Eigen::Vector3f re_p = cam_p.block<3,3>(0,0)*medpos + cam_p.block<3,1>(0,3);
                //      Eigen::Vector2i re_pi((int)(re_p(0)/re_p(2)),(int)(re_p(1)/re_p(2)));
                //      errors = abs(re_pi(0) - vSignals[j].ObsLabels[i].position.x - (vSignals[j].ObsLabels[i].position.width>>1)) +
                //             abs(re_pi(1) - vSignals[j].ObsLabels[i].position.y - (vSignals[j].ObsLabels[i].position.height>>1));
                //     printf("| errors: %f", errors);
                // }
                vSignals[j].LeftUpPos = leftpos;
                vSignals[j].RightDownPos = rightpos;
                //投影回来。
                Eigen::Vector3f CamBack =Tcw.inverse().block<3,1>(0,2);
                Eigen::Vector3f c2l = medpos -Tcw.inverse().block<3,1>(0,3);
                //printf("cons %f  %f ",c2l.dot(CamBack)/(c2l.norm()*CamBack.norm()), CamBack(2));
                if (c2l.dot(CamBack)/(c2l.norm()*CamBack.norm()) < 0.1 ){
                    printf("a passed lamp|\n");
                    vSignals[j].trackStatus = "passed";
                    continue;
                } 
                auto cam_p = pReader.camParams.INTRINSIC*Tcw.block<3,4>(0,0);
                curRect = reproject2img(leftpos,rightpos,cam_p);
                // if ((curRect.width < 4 && curRect.height < 8) || (curRect.width > 30 && curRect.height > 60)){
                //     continue;
                // }
               // curRect = LampCalib(curRect,CurImg);
                if(vSignals[j].ObsLabels[obsNum-1].frameIndex < (size_t)FrameID)
                    vSignals[j].CurFrameRect = curRect;
                SignalRects.push_back(vSignals[j].CurFrameRect);
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

    void label2signal (int FrameID,  std::vector<Label> *Labels){
        std::vector<cv::Rect> LastLampRects;
        if(Labels){ 
            // 有lable的话判断label归类。
          //  std::vector<Label> *Labels = pReader.getLabels(FrameID);       
            for (Label &label : *Labels){
                bool NoParent = true;
                // 把现存的signal 投过来，比对。
                for (size_t i = 0; i < vSignals.size(); ++i){
                    if(vSignals[i].ObsLabels.size() > 0 && vSignals[i].trackStatus != "passed")
                    if(abs(label.position.x + (label.position.width>>1) - vSignals[i].CurFrameRect.x - 
                                            (vSignals[i].CurFrameRect.width>>1) - vSignals[i].speed(0)) < 2*label.position.width && 
                       abs(label.position.y + (label.position.height>>1) - vSignals[i].CurFrameRect.y -
                                             (vSignals[i].CurFrameRect.height>>1) - vSignals[i].speed(1)) < 2*label.position.height){
                            NoParent = false;
                            label.SID = vSignals[i].SID;
                            vSignals[i].ObsLabels.push_back(label);
                            break;
                       }
                }
                if(NoParent){
                   //当做新来的灯。
                   printf("new a signal!\n");
                   Signal signal;
                   signal.startFrame = FrameID;
                   signal.SID = (int)vSignals.size();
                   label.SID = signal.SID;// LabelData 里没有生效
                   signal.ObsLabels.push_back(label);
                   signal.CurFrameRect = label.position;
                   signal.speed = Eigen::Vector2i(0, 0);
                   vSignals.push_back(signal);
                }
            }
        }
    }
    void runOnce(cv::Mat img, Eigen::Matrix4f Tcw, int FrameID, std::vector<Label> *Labels){
        cv::Mat img_light = img.clone();
        std::vector<cv::Rect> RectsOri;
        if(Labels)
            for (Label &lab : *Labels){
                    using namespace cv;
                    if(!undistortedImg){
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
                    }
                    rectangle(img, lab.position, cv::Scalar(0,255,0), 2);
                    RectsOri.push_back(lab.position);
                    // cv::Rect lamp_calib = LampCalib(lab.position, img);
                    // cv::rectangle(img, lamp_calib, cv::Scalar(0,0,255), 2);
                }
            label2signal(FrameID, Labels);
            std::vector <cv::Rect> Rects = updateSignals(FrameID, img, Tcw);
            //if(Rects.size() == 0) Rects = RectsOri;
            for(auto rect : Rects){
                cv::rectangle(img_light,rect,cv::Scalar(0,0,255),2);
            }
       //  resize(img, img, cv::Size(968, 768));
      //  resize(img_light, img_light, cv::Size(968, 768));
        cv::imshow("original", img);
        cv::imshow("rectified", img_light);
        cv::waitKey(100);

    }
    int run()
    {
        cv::VideoCapture imgShowSeque;
        string img_path = pReader.getData("img_path");
        int drawImgsNum = 0;
        bool playvideo = true;
        rec_start = stoi(pReader.getData("rec_start"));
        while(drawImgsNum < 600){
                //一帧帧显示图像
                cv::Mat img, img_calib;	
                //imgShowSeque >> img;
                std::stringstream numberss;
                // numberss << std::setw(5) << std::setfill('0') << rec_start+drawImgsNum; // 0000, 0001, 0002, etc...
                // std::string name = img_path + "157_" +numberss.str() + ".png";
                numberss << std::setw(6) << std::setfill('0') << rec_start+drawImgsNum; // 0000, 0001, 0002, etc...
                std::string name = img_path + numberss.str() + ".png";
                
                img = cv::imread(name);
                if (img.empty()){
                    return 0;
                }
                if (!undistortedImg){
                    camMatrix = (cv::Mat_<float>(3,3)<<camm.fx, 0, camm.cx, 
                                                        0, camm.fy, camm.cy,
                                                        0 , 0, 1);
                    distCoeffs = cv::Mat(5,1,CV_32F,camm.distortion);
                    cv::undistort(img, img_calib, camMatrix, distCoeffs);
                    img = img_calib.clone();
                } else {
                    img_calib = img.clone();
                }
                //cv::Mat showImg = img.clone(), showImg_calib = img_calib.clone();
                std::vector<Label> *Labels = pReader.getLabels(drawImgsNum + rec_start);
                Eigen::Matrix4f Tcw = pReader.trajectory[traj_start + drawImgsNum].inverse();

                runOnce(img_calib, Tcw, rec_start+drawImgsNum, Labels);

                char presskey = cv::waitKey(500);
                if(presskey == 'p') playvideo = !playvideo;
                if(playvideo){
                    cout<<drawImgsNum+rec_start<<'\n';
                    drawImgsNum++;
                }
            }
        return 0;
    }
public:
    cv::Mat camMatrix;// camera intrinsic, cv Mat
    cv::Mat distCoeffs;// camera distortions
    bool undistortedImg;// a flag indicate if the input image is undistorted
    float lightw,lighth; // signal light's width、 height
    int rec_start,traj_start;// label record start from frameId xx, trajectory record start from frameId xx
    ParameterReader pReader; 
    CamParams camm; // camera intrinsic, a  special class
    std::vector < Signal > vSignals; // recontructed traffic signals
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