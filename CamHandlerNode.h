#pragma once
#include"common.h" 
#include <opencv2/opencv.hpp>

struct CamOption{
    /*ROI*/
    bool useROI;
    int roiWidth; //  2592
    int roiHeight;//  1038
    int roiX;// 0
    int roiY; // 600
    
    /*分辨率*/
    int nWidth; // 2592
    int nHeight;// 2048
    
    /* 初始曝光*/
    bool autoExposure;
    double exposureTime;// 10000
    
    /*白平衡*/
    bool setBalanceRatoio;
    double balanceRatoioRed; // 1.5 
    double balanceRatoioGreen; //  1
    double balanceRatoioBlue; //  1.5

    /*伽马*/
    bool setGamma;
    double dGamma;  // 0.6

    /*取图模式*/
    int grabMode;

    CamOption():useROI(true),roiWidth(2592),roiHeight(1038),
        roiX(0),roiY(600),
        nWidth(2592),nHeight(2048),
        autoExposure(true),exposureTime(10000.0),
        setBalanceRatoio(true),balanceRatoioRed(1.5),balanceRatoioBlue(1.5),balanceRatoioGreen(1),
        setGamma(true),dGamma(0.6){};
};


struct CamParam
{
    int imageWidth;
    int imageHeight;

    cv::Mat mapLx, mapLy, mapRx, mapRy;     //映射表  
    cv::Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
    cv::Mat xyz;              //三维坐标

    /*相机内参*/
    cv::Mat cameraMatrixL = (cv::Mat_<double>(3, 3) <<
        903.619734199452,0,952.971392111169,
        0,954.715461789098,542.112445049920,
        0,0,1);
    cv::Mat distCoeffL = (cv::Mat_<double>(4, 1) << 
        -0.309957088078872,0.0837818975629608,0,0);
    
    cv::Mat cameraMatrixR = (cv::Mat_<double>(3, 3) << 
        908.100241725721,0,928.649652459136,
        0,959.573864137008,550.978004165741,
        0,0,1);
    cv::Mat distCoeffR = (cv::Mat_<double>(5, 1) << 
        -0.306614842052119,0.0746103582700683,0,0);
    
    /*相机外参*/
    cv::Mat T = (cv::Mat_<double>(3, 1) << -181.951646922374,-3.78508082860841,2.56476205178126);//T平移向量
    //Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec旋转向量
    cv::Mat R = (cv::Mat_<double>(3,3)<<
        0.999877619025646,0.0125714116902012,0.00931163679068092,
        -0.0125563015687748,0.999919756227700,-0.00167940360014598,
        -0.00933200206386997,0.00156227835345441,0.999955235509983);//R 旋转矩阵
    
};


class CamHandlerNode
{
public:
    CamHandlerNode(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
    CamHandlerNode():CamHandlerNode(ros::NodeHandle(),ros::NodeHandle("~")){}
    ~CamHandlerNode();
    
    /* sdk硬触发的回调函数只能是void 型,
     * 使用类成员函数会隐式得带上CamHandlerNode*,所以几个函数和下面
     * 两个成员都设置成static了,能用但很屎,有大佬有办法可以改一下*/
    void CamInit();
    void static onGetLeftFrame(const CFrame& pframe);
    void static onGetRightFrame(const CFrame& pframe);
    void static convertFrame(const CFrame &pFrame,cv::Mat& frame);

    // 图片传输
    image_transport::ImageTransport imageTransport_;
    static image_transport::Publisher imagePublisherLeft_;
    static image_transport::Publisher imagePublisherRight_;
private:

    CamOption option_; 
    static CamParam param_;
    ICameraPtr cameraSptrL_;
    ICameraPtr cameraSptrR_;

    IStreamSourcePtr streamPtrL_ ;
    IStreamSourcePtr streamPtrR_ ;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private;

    //参数
    std::string cameraTopicLeft_;
    std::string cameraTopicRight_;

    std::vector<cv::Mat> images_to_pub_;
};

