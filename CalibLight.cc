/*!
 * @brief
 * this project is aiming to triangulate the traffic signals with known camera pose and signals' image coordinates.
 * @file CalibLight.cc
 * @author CaoJian
 * @date 2018-04-02
 */
#include "CalibLight.h"

class RectCalib {
public:
    RectCalib() {}
    RectCalib(const RectCalib & rc):undistortedImg(rc.undistortedImg),
        traj_start(rc.traj_start), lightw(rc.lightw), lighth(rc.lighth), lightd(rc.lightd),
        camMatrix(rc.camMatrix), distCoeffs(rc.distCoeffs), pReader(rc.pReader), vSignals(rc.vSignals) {}
    RectCalib(ParameterReader pd, cv::Mat mK, cv::Mat mD):camMatrix(mK), distCoeffs(mD) {
        pReader = pd;
        undistortedImg = pReader.getData("isUndistorted") == "yes";
        traj_start = stoi(pReader.getData("traj_start"));
        lightw = stof(pReader.getData("light_width_half"));
        lighth = stof(pReader.getData("light_height_half"));
        lightd = stof(pReader.getData("light_depth_half"));
    }
    RectCalib(bool undistorted, int traj_start_, float lightw_, float lighth_, float lightd_,
        cv::Mat mK_, cv::Mat mD_):undistortedImg(undistorted), traj_start(traj_start_),
        lightw(lightw_), lighth(lighth_), lightd(lightd_), camMatrix(mK_), distCoeffs(mD_) {}
    /*!
    *this function(LampCalib) is designed to calibrate the lamp's rectangle, 
    *but is abandoned now because of its poor robustness...
    *
    * cv::Rect LampCalib(cv::Rect oriRect, cv::Mat img)
    * {
    *     using namespace cv;
    *     if(oriRect.width < 20 && oriRect.height < 40)
    *         return oriRect;
    *     int expand = 6;
    *     cv::Rect epRect(max(0,oriRect.x-expand), max(0,oriRect.y-expand), oriRect.width + 2*expand, oriRect.height+2*expand);
    *     epRect.width = epRect.x+epRect.width > img.cols ? img.cols-epRect.x : epRect.width;
    *     epRect.height = epRect.y+epRect.height > img.rows ? img.rows-epRect.height : epRect.height;
    *     cv::Mat img_cand = img(epRect);
    *     blur( img_cand, img_cand, Size(3,3));
    *     /// imshow("tmp",img_cand);
    *     /// waitKey();
    *     Mat src_gray;
    *     cvtColor( img_cand, src_gray, COLOR_BGR2GRAY );
    *     /// Mat threshold_output;
    *     ///  std::vector<std::vector<cv::Point> > contours;
    *     ///  std::vector<Vec4i> hierarchy;
    *     ///threshold( src_gray, threshold_output, 100, 255, THRESH_BINARY );
    *     Canny(src_gray, src_gray, 50, 100);
    *     ///imshow("sss",src_gray);
    *     ///保留1/4的角落
    *     ///int splitc = (int)(0.25 * src_gray.cols);
    *     int splitr = (int)(0.25 * src_gray.rows);
    * /// src_gray.colRange(splitc, src_gray.cols-splitc).setTo(0);
    *     src_gray.rowRange(splitr, src_gray.rows-splitr).setTo(0);
    *     /// findContours( src_gray, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    *     ///imshow("sss",src_gray);
    *     /// Mat imageContours=Mat::zeros(src_gray.size(),CV_8UC1);
    *     /// for( size_t i = 0; i< contours.size(); i++ )
    *     /// {
    *     ///     drawContours( imageContours, contours, (int)i, Scalar(255), 1, 8, hierarchy);
    *     /// }
    *     ///  imshow("tmp",imageContours);
    *     cv::Rect myRect= boundingRect(src_gray);
    * /// cv::rectangle(img_cand, myRect, Scalar(255,0,0),2);
    *     ///cv::rectangle(img_cand, oriRect, Scalar(0,255,0),2);
    * /// imshow("tmp2",img_cand);
    * if(myRect.height < myRect.width || myRect.height < 20)
    *         myRect.height = oriRect.height;
    * if(myRect.width < 10)
    *         myRect.width = oriRect.width;
    *     ///waitKey();
    *     return cv::Rect(myRect.x+epRect.x, myRect.y+epRect.y, myRect.width, myRect.height);
    * }
    */
    std::vector<cv::Rect> updateSignals(int FrameID, const cv::Mat &CurImg,
        cv::Mat Tcw, const std::vector<cv::Mat>& Twcss) {
        Eigen::Matrix3f CamInstri;
        Eigen::Matrix4f Pose;
        cv::cv2eigen(Tcw, Pose);
        cv::cv2eigen(camMatrix, CamInstri);
        std::vector<cv::Rect> SignalRects;
        for (size_t j = 0; j < vSignals.size(); ++j) {
            cv::Rect curRect;
            const size_t obsNum = vSignals[j].ObsLabels.size();
            if (obsNum > 1
                    && vSignals[j].ObsLabels[obsNum - 1].frameIndex == (size_t)FrameID) {
                int d_x = (vSignals[j].ObsLabels[obsNum - 1].position.x +
                           (vSignals[j].ObsLabels[obsNum - 1].position.width >> 1)
                           - vSignals[j].CurFrameRect.x - (vSignals[j].CurFrameRect.width >> 1));
                int d_y = (vSignals[j].ObsLabels[obsNum - 1].position.y +
                           (vSignals[j].ObsLabels[obsNum - 1].position.height >> 1)
                           - vSignals[j].CurFrameRect.y - (vSignals[j].CurFrameRect.height >> 1));
                vSignals[j].speed(0) = d_x;
                vSignals[j].speed(1) = d_y;
                vSignals[j].CurFrameRect = vSignals[j].ObsLabels[obsNum - 1].position;
                /// printf("show me dx: %d dy: %d \n",d_x, d_y);
            }
            if (obsNum > 2 && vSignals[j].trackStatus != "passed") {
                Eigen::MatrixXf Tl;
                Eigen::VectorXf pl;
                Eigen::MatrixXf Tr;
                Eigen::VectorXf pr;
                Tl.resize(2 * obsNum, 3);
                Tr.resize(3 * obsNum, 3);
                pl.resize(2 * obsNum);
                pr.resize(3 * obsNum);
                for (size_t i = 0; i < obsNum; i++) {
                    auto label = vSignals[j].ObsLabels[i];
                    Eigen::Matrix4f T;
                    cv::cv2eigen(Twcss[label.frameIndex-traj_start].inv(), T);
                    Eigen::Matrix3f R = T.block<3, 3>(0, 0);
                    Eigen::Matrix<float, 3, 3> KR = CamInstri * R;
                    Eigen::Vector2f uv((label.position.x + (label.position.width >> 1)),
                                       (label.position.y + (label.position.height >> 1)));
                    /// KR(1c) - uR(3c) Xw = t3u - K(1c)t
                    ///     Tr      *  Xw  = pr
                    /// 添加时序衰减系数权重，以使得越久远的观测量影响越小。
                    /// A: e^(0.5x), x = 2+fi-f,
                    /// B: -10/x, x = fi -f -1
                    ///  float coeff = exp(0.5 * (2 + static_cast<float>(label.frameIndex)
                    ///                   - vSignals[j].ObsLabels[obsNum-1].frameIndex));
                    float coeff = 10.0f /(1 - static_cast<float>(label.frameIndex)
                                     + vSignals[j].ObsLabels[obsNum-1].frameIndex);
                    /// coeff = 1;
                    Tr.block<1, 3>(3 * i, 0)  = coeff * (KR.row(0) - uv(0) * R.row(2));
                    Tr.block<1, 3>(3 * i + 1, 0) = coeff * (KR.row(1) - uv(1) * R.row(2));
                    Tr.block<1, 3>(3 * i + 2, 0) = coeff * (KR.row(2) - R.row(2));
                    pr(3 * i) = coeff * (T(2, 3) * uv(0) - CamInstri.
                                row(0) * T.block<3, 1>(0, 3));
                    pr(3 * i + 1) = coeff * (T(2, 3) * uv(1) - CamInstri.
                                    row(1) * T.block<3, 1>(0, 3));
                    pr(3 * i + 2) = coeff * (T(2, 3) - CamInstri.row(2) *
                                    T.block<3, 1>(0, 3));
                }
                /// Eigen::Vector3f medpos = (Tl.transpose()*Tl).colPivHouseholderQr()
                ///                             .solve(-Tl.transpose()*pl);
                Eigen::Vector3f medpos = (Tr). jacobiSvd(Eigen::ComputeThinU|
                                             Eigen::ComputeThinV).solve(pr);
                printf(" light : %f %f %f \n", medpos(0), medpos(1), medpos(2));

                /// Eigen::Vector3f leftpos = (Tl.transpose()*Tl).colPivHouseholderQr()
                ///        .solve(-Tl.transpose()*pl);
                /// Eigen::Vector3f rightpos =  (Tr.transpose()*Tr).colPivHouseholderQr()
                ///        .solve(-Tr.transpose()*pr);
                /// printf("leftu: %f %f %f, rightd: %f %f %f\n",leftpos(0),
                /// leftpos(1),leftpos(2),rightpos(0),rightpos(1),rightpos(2));
                /// 灯的实际大小应该是个固定值。。。
                Eigen::Vector3f leftpos(medpos(0) - lightw, medpos(1) - lighth, medpos(2) + lightd);
                Eigen::Vector3f rightpos(medpos(0) + lightw, medpos(1) + lighth, medpos(2) + lightd);
                Eigen::Vector3f rightdepPos(medpos(0) + lightw, medpos(1) - lighth, medpos(2) - lightd);
                /*!
                /// float errors = 0;
                /// for(size_t i = 0; i < obsNum; ++i){
                ///      auto cam_p = CamInstri*pReader.trajectory[vSignals[j].ObsLabels[i].
                /// frameIndex-rec_start+traj_start].inverse().block<3,4>(0,0);
                ///      Eigen::Vector3f re_p = cam_p.block<3,3>(0,0)*medpos + cam_p.block<3,1>(0,3);
                ///      Eigen::Vector2i re_pi((int)(re_p(0)/re_p(2)),(int)(re_p(1)/re_p(2)));
                ///      errors = abs(re_pi(0) - vSignals[j].ObsLabels[i].position.x - (vSignals[j].ObsLabels[i].position.width>>1)) +
                ///             abs(re_pi(1) - vSignals[j].ObsLabels[i].position.y - (vSignals[j].ObsLabels[i].position.height>>1));
                ///     printf("| errors: %f", errors);
                /// }
                */
                vSignals[j].LeftUpPos = leftpos;
                vSignals[j].RightDownPos = rightpos;
                /// 投影回来。
                Eigen::Vector3f CamBack = Pose.inverse().block<3, 1>(0, 2);
                Eigen::Vector3f c2l = medpos - Pose.inverse().block<3, 1>(0, 3);
                /// printf("cons %f  %f ",c2l.dot(CamBack) / (c2l.norm()*CamBack.norm()), CamBack(2));
                if ( c2l.dot(CamBack) / (c2l.norm() * CamBack.norm() ) < 0.2 ) {
                    printf("a passed lamp|\n");
                    vSignals[j].trackStatus = "passed";
                    continue;
                }
                auto cam_p = CamInstri * Pose.block<3, 4>(0, 0);
                curRect = reproject2img(leftpos, rightpos, rightdepPos, cam_p);
                /// if ((curRect.width < 4 && curRect.height < 8) ||
                ///             (curRect.width > 30 && curRect.height > 60)){
                ///     continue;
                /// }
                /// curRect = LampCalib(curRect,CurImg);
                if (vSignals[j].ObsLabels[obsNum - 1].frameIndex < (size_t)FrameID) {
                    vSignals[j].CurFrameRect = curRect;
                }
                SignalRects.push_back(vSignals[j].CurFrameRect);
            }
        }
        printf("total %zd signals\n", SignalRects.size());
        return SignalRects;
    }
    cv::Rect reproject2img(Eigen::Vector3f LeftUpPos, Eigen::Vector3f RightDownPos,
                   Eigen::Vector3f RightupDepPos, Eigen::Matrix<float, 3, 4>cam_p) {
        // left up
        auto left3f = cam_p.block<3, 3>(0, 0) * LeftUpPos + cam_p.block<3, 1>(0, 3);
        cv::Point2i Left2i(static_cast<int>(left3f(0)/left3f(2)), static_cast<int>(left3f(1)/left3f(2)));
        // right down
        auto Right3f = cam_p.block<3, 3>(0, 0) * RightDownPos + cam_p.block<3, 1>(0, 3);
        cv::Point2i Right2i(static_cast<int>(Right3f(0)/Right3f(2)), static_cast<int>(Right3f(1)/Right3f(2)));
        // right up depth
        auto RUPdepth3f = cam_p.block<3, 3>(0, 0) * RightupDepPos + cam_p.block<3, 1>(0, 3);
        cv::Point2i RUPdepth2i(static_cast<int>(RUPdepth3f(0)/RUPdepth3f(2)),
                                                static_cast<int>(RUPdepth3f(1)/RUPdepth3f(2)));
        std::vector<cv::Point2i> lightPoss;
        lightPoss.push_back(Left2i);
        lightPoss.push_back(Right2i);
        lightPoss.push_back(RUPdepth2i);
        cv::Rect curRect = cv::boundingRect(lightPoss);
        return curRect;
    }

    void label2signal(int FrameID,  std::vector<Label> *Labels) {
        std::vector<cv::Rect> LastLampRects;
        if (Labels->size()) {
            /// 有lable的话判断label归类。
            ///  std::vector<Label> *Labels = pReader.getLabels(FrameID);
            for (Label &label : *Labels) {
                bool NoParent = true;
                /// 把现存的signal 投过来，比对。
                for (size_t i = 0; i < vSignals.size(); ++i) {
                    if (vSignals[i].ObsLabels.size() > 0 && vSignals[i].trackStatus != "passed") {
                        if (abs(label.position.x + (label.position.width >> 1) -
                                vSignals[i].CurFrameRect.x - (vSignals[i].CurFrameRect.width >> 1)
                                 - vSignals[i].speed(0)) < 2 * label.position.width &&
                                abs(label.position.y + (label.position.height >> 1) -
                                 vSignals[i].CurFrameRect.y - (vSignals[i].CurFrameRect.height >> 1)
                                 - vSignals[i].speed(1)) < 2 * label.position.height) {
                            NoParent = false;
                            label.SID = vSignals[i].SID;
                            vSignals[i].ObsLabels.push_back(label);
                            break;
                        }
                    }
                }
                if (NoParent) {
                    /// 当做新来的灯。
                    printf("new a signal!\n");
                    Signal signal;
                    signal.startFrame = FrameID;
                    signal.SID = static_cast<int>(vSignals.size());
                    label.SID = signal.SID;     /// LabelData 里没有生效
                    signal.ObsLabels.push_back(label);
                    signal.CurFrameRect = label.position;
                    signal.speed = Eigen::Vector2i(0, 0);
                    vSignals.push_back(signal);
                }
            }
        }
    }
    std::vector <cv::Rect> runOnce(cv::Mat& img, cv::Mat Tcw, int FrameID,
        std::vector<Label> *Labels, std::vector<cv::Mat>& Twcss, int traj_st) {
        traj_start = traj_st;
        cv::Mat img_light = img.clone();
        std::vector<cv::Rect> RectsOri(0);
        if (Labels) {
            for (Label &lab : *Labels) {
                using namespace cv;
                if (!undistortedImg) {
                    std::vector<cv::Point2f> srcPoints, dstPoints;
                    srcPoints.push_back(cv::Point2f(lab.position.x, lab.position.y));
                    srcPoints.push_back(cv::Point2f(lab.position.x + lab.position.width,
                                                    lab.position.y + lab.position.height));
                    cv::undistortPoints(srcPoints, dstPoints, camMatrix, distCoeffs);
                    dstPoints[0].x = dstPoints[0].x * camMatrix.at<float>(0, 0) +  camMatrix.at<float>(0, 2);
                    dstPoints[0].y = dstPoints[0].y * camMatrix.at<float>(1, 1) +  camMatrix.at<float>(1, 2);
                    dstPoints[1].x = dstPoints[1].x * camMatrix.at<float>(0, 0) +  camMatrix.at<float>(0, 2);
                    dstPoints[1].y = dstPoints[1].y * camMatrix.at<float>(1, 1) +  camMatrix.at<float>(1, 2);
                    dstPoints[1].x = std::min(static_cast<int>(dstPoints[1].x), img.cols);
                    dstPoints[1].y = std::min(static_cast<int>(dstPoints[1].y), img.rows);
                    lab.position.x = std::max(static_cast<int> (dstPoints[0].x), 0);
                    lab.position.y = std::max(static_cast<int> (dstPoints[0].y), 0);
                    lab.position.width = static_cast<int> (dstPoints[1].x - dstPoints[0].x);
                    lab.position.height = static_cast<int> (dstPoints[1].y - dstPoints[0].y);
                    if (lab.position.x < 0 || lab.position.y < 0 || dstPoints[1].x > img.cols
                            || dstPoints[1].y > img.rows) {
                        continue;
                    }
                }
                rectangle(img, lab.position, cv::Scalar(0, 255, 0), 2);
                RectsOri.push_back(lab.position);
                /// cv::Rect lamp_calib = LampCalib(lab.position, img);
                /// cv::rectangle(img, lamp_calib, cv::Scalar(0,0,255), 2);
            }
            label2signal(FrameID, Labels);
        }
        std::vector <cv::Rect> Rects = updateSignals(FrameID, img, Tcw, Twcss);
        /// if(Rects.size() == 0) Rects = RectsOri;
        for (auto rect : Rects) {
            cv::rectangle(img_light, rect, cv::Scalar(0, 0, 255), 2);
        }
        ///  resize(img, img, cv::Size(968, 768));
        ///  resize(img_light, img_light, cv::Size(968, 768));
        cv::imshow("original", img);
        cv::imshow("rectified", img_light);
        cv::waitKey(100);
        return Rects;
    }
    int run() {
        cv::VideoCapture imgShowSeque;
        string img_path = pReader.getData("img_path");
        int drawImgsNum = 0;
        bool playvideo = true;
        int rec_start = stoi(pReader.getData("rec_start"));
        while (drawImgsNum < 600) {
            /// 一帧帧显示图像
            cv::Mat img, img_calib;
            /// imgShowSeque >> img;
            std::stringstream numberss;
            /// numberss << std::setw(5) << std::setfill('0')
            /// << rec_start+drawImgsNum; /// 0000, 0001, 0002, etc...
            /// std::string name = img_path + "157_" +numberss.str() + ".png";
            numberss << std::setw(6) << std::setfill('0') << rec_start +
                     drawImgsNum;    /// 0000, 0001, 0002, etc...
            std::string name = img_path + numberss.str() + ".png";

            img = cv::imread(name);
            if (img.empty()) {
                return 0;
            }
            if (!undistortedImg) {
                cv::undistort(img, img_calib, camMatrix, distCoeffs);
                img = img_calib.clone();
            } else {
                img_calib = img.clone();
            }
            /// cv::Mat showImg = img.clone(), showImg_calib = img_calib.clone();
            std::vector<Label> *Labels = pReader.getLabels(drawImgsNum + rec_start);
            cv::Mat Tcw = pReader.trajectory[rec_start + drawImgsNum - traj_start].inv();
            int traj_st = stoi(pReader.getData("traj_start"));
            runOnce(img_calib, Tcw, rec_start + drawImgsNum, Labels, pReader.trajectory, traj_st);

            char presskey = cv::waitKey(500);
            if (presskey == 'p') {
                playvideo = !playvideo;
            }
            if (playvideo) {
                cout << drawImgsNum + rec_start << '\n';
                drawImgsNum++;
            }
        }
        return 0;
    }

public:
    bool undistortedImg;    /// a flag indicate if the input image is undistorted
    int  traj_start;     /// trajectory record start index after label start
    float lightw, lighth, lightd;     /// signal light's width、 height,depth
    cv::Mat camMatrix;    /// camera intrinsic, cv Mat
    cv::Mat distCoeffs;    /// camera distortions
    ParameterReader pReader;
    std::vector < Signal > vSignals;   /// recontructed traffic signals
};
int main(int argc, char **argv) {
    ////就一个参数，参数文件位置
    if (argc < 2) {
        cerr << "arguments missing!\n";
        return -1;
    }
    ParameterReader pReader(argv[1]);
    cv::Mat camMatrix;
    cv::eigen2cv(pReader.camParams.INTRINSIC, camMatrix);
    cv::Mat distCoeffs = cv::Mat(5, 1, CV_32F, pReader.camParams.distortion);
    RectCalib RC(pReader, camMatrix, distCoeffs);
    RC.run();
    return 0;
}
