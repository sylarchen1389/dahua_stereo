#include"common.h" 
#include"CamHandlerNode.h"
#include<Media/ImageConvert.h>
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Infra/PrintLog.h"
#include "Memory/SharedPtr.h"
#include <boost/bind/bind.hpp>



image_transport::Publisher CamHandlerNode::imagePublisherLeft_;
image_transport::Publisher CamHandlerNode::imagePublisherRight_;
CamParam CamHandlerNode::param_;


void CamHandlerNode::CamInit(){
    static volatile int keepRunning = 1; 
    bool bRetL,bRetR;

    /*connet start */
    CSystem &systemObj = CSystem::getInstance();
    int count = 10;
    while (count --)
    {
        /*检测摄像头列表*/
        TVector<ICameraPtr> vCameraPtrList;
        bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);

        if (!isDiscoverySuccess)
        {
            printf("discovery device fail.\n");
            continue;
        }
       
        if (vCameraPtrList.size() == 0)
        {
            printf("no devices found.\n");
            continue;
        }else{            
	        //print camera info (index,Type,vendor name, model,serial number,DeviceUserID,IP Address)
	        // 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）
            dahua::displayDeviceInfo(vCameraPtrList);
            int cameraIndexL = dahua::leftRaw ;
            int cameraIndexR = dahua::rightRaw ;
            std::cout<<"connect left: "<<cameraIndexL <<std::endl
                <<"connect right: "<<cameraIndexR<<std::endl;
            if(vCameraPtrList.size()<=1)
                continue;
            cameraSptrL_ = vCameraPtrList[cameraIndexL];
            cameraSptrR_ = vCameraPtrList[cameraIndexR];
          
           /* GigE相机时，连接前设置相机Ip与网卡处于同一网段上 */
	        if( ICamera::typeGige == cameraSptrL_->getType())
	        {
	        	if(dahua::autoSetCameraIP(cameraSptrL_) != 0)
	        	{
	        		printf("set left camera Ip failed.\n");
                    continue;
	        	}
	        }

            if( ICamera::typeGige == cameraSptrR_->getType())
	        {
	        	if(dahua::autoSetCameraIP(cameraSptrR_) != 0)
	        	{
	        		printf("set right camera Ip failed.\n");
                    continue;
	        	}
	        }

            /* 连接相机 */
            if (!(cameraSptrL_->connect()&& cameraSptrR_ ->connect()))
            {
                printf("connect cameral failed.\n");
                continue;
	        }

            break;
        }
    }
    /*connet finish */
    
    // 创建AcquisitionControl对象
    IAcquisitionControlPtr sptrAcquisitionControlLeft = systemObj.createAcquisitionControl(cameraSptrL_);
	if (NULL == sptrAcquisitionControlLeft.get())
	{
		printf("create Left AcquisitionControl object fail.\n");
        cameraSptrL_->disConnect();
        cameraSptrR_->disConnect();
        exit(1);
	}

    IAcquisitionControlPtr sptrAcquisitionControlRight = systemObj.createAcquisitionControl(cameraSptrR_);
	if (NULL == sptrAcquisitionControlRight.get())
	{
		printf("create Right AcquisitionControl object fail.\n");
        cameraSptrL_->disConnect();
        cameraSptrR_->disConnect();
        exit(1);
	}

    // 配置 摄像头 取图
    /*设置分辨率*/
    dahua::setResolution(cameraSptrL_,option_.nWidth, option_.nHeight);
    dahua::setResolution(cameraSptrR_,option_.nWidth, option_.nHeight);
    /*ROI*/
    bool success;
    success = dahua::setROI(cameraSptrL_, option_.roiX,option_.roiY,option_.roiWidth,option_.roiHeight);
    std::cout<<"[INFO]set left roi: "<<success<<std::endl;
    success = dahua::setROI(cameraSptrR_, option_.roiX,option_.roiY,option_.roiWidth,option_.roiHeight);
    std::cout<<"[INFO]set right roi: "<<success<<std::endl;
    /*设置主动曝光*/
    dahua::setExposureTime(cameraSptrL_, option_.exposureTime,false);
    dahua::setExposureTime(cameraSptrR_, option_.exposureTime,false);
    /*白平衡*/
    dahua::setBalanceRatio(cameraSptrL_, option_.balanceRatoioRed,option_.balanceRatoioGreen,option_.balanceRatoioBlue);    
    dahua::setBalanceRatio(cameraSptrR_, option_.balanceRatoioRed,option_.balanceRatoioGreen,option_.balanceRatoioBlue);
    /*伽马*/
    dahua::setGamma(cameraSptrL_, option_.dGamma);
    dahua::setGamma(cameraSptrR_, option_.dGamma);


    // 设置外部触发配置
    bRetL = dahua::setLineTriggerConf(sptrAcquisitionControlLeft);
    bRetR = dahua::setLineTriggerConf(sptrAcquisitionControlRight);
    if(!bRetL||!bRetR){
        printf("set Line trigger config fail.\n");
        cameraSptrL_->disConnect();
        cameraSptrR_->disConnect();
        exit(1);
    }


  

    // 创建流对象
    streamPtrL_ = systemObj.createStreamSource(cameraSptrL_);
    if (NULL == streamPtrL_.get())
	{
		printf("create left stream obj  fail.\n");
		cameraSptrL_->disConnect();
        cameraSptrR_->disConnect();
        exit(1);
	}

    streamPtrR_ = systemObj.createStreamSource(cameraSptrR_);
    if (NULL == streamPtrR_.get())
	{
		printf("create right stream obj  fail.\n");
		cameraSptrL_->disConnect();
        cameraSptrR_->disConnect();
        exit(1);
	}


    // 注册回调函数
    bRetL = streamPtrL_->attachGrabbing(&CamHandlerNode::onGetLeftFrame);
    bRetR = streamPtrR_->attachGrabbing(&CamHandlerNode::onGetRightFrame);
    if(!bRetL||!bRetR){
        printf("attach Grabbing fail.\n");
        cameraSptrL_->disConnect();
        cameraSptrR_->disConnect();
        exit(1);
    }
    
    // 开始拉流
    bRetL = streamPtrL_->startGrabbing();
    bRetR = streamPtrR_->startGrabbing();
    if(!bRetL||!bRetR){
        printf("StartGrabbing  fail.\n");
        cameraSptrL_->disConnect();
        cameraSptrR_->disConnect();
        exit(1);
    }
}

CamHandlerNode::CamHandlerNode(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private)
:nh_(nh),nh_private(nh_private),imageTransport_(nh)
{
    #pragma region parm
    nh_private.param("useROI",option_.useROI,true);
    nh_private.param("roiWidth",option_.roiWidth,2592);
    nh_private.param("roiHeight",option_.roiHeight,1038);
    nh_private.param("roiX",option_.roiX,0);
    nh_private.param("roiY",option_.roiY,600);
    nh_private.param("nWidth",option_.nWidth,2592);
    nh_private.param("nHeight",option_.nHeight,2048);
    nh_private.param("autoExposure",option_.autoExposure,true);
    nh_private.param("exposureTime",option_.exposureTime,10000.0);
    nh_private.param("setBalanceRatoio",option_.setBalanceRatoio,true);
    nh_private.param("balanceRatoioRed",option_.balanceRatoioRed,1.5);
    nh_private.param("balanceRatoioGreen",option_.balanceRatoioBlue,1.5);
    nh_private.param("balanceRatoioBlue",option_.balanceRatoioGreen,1.0);
    nh_private.param("setGamma",option_.setGamma,true);
    nh_private.param("dGamma",option_.dGamma,0.6);
    nh_private.param("cameraTopicLeft",cameraTopicLeft_,std::string("/camera/leftRaw"));
    nh_private.param("cameraTopicRight",cameraTopicRight_,std::string("/camera/rightRaw"));
    nh_private.param("pubWidth",param_.imageWidth,2592);
    nh_private.param("pubHeight",param_.imageHeight,1038);
#pragma endregion  
#pragma region camParam
    param_.cameraMatrixL = (cv::Mat_<double>(3, 3) <<
        1799.98852787378,0,1323.68935404431,
        0,1799.10577181371,1023.97977890216,
        0,0,1);
    param_.distCoeffL = (cv::Mat_<double>(5, 1) << 
        -0.147055131065462,0.150449210882043,-0.00133156154198220,0.00211672773185521,-0.0421072966690359);

    param_.cameraMatrixR = (cv::Mat_<double>(3, 3) << 
        1800.14279727335,0,1307.88868461770,
        0,1799.39390950049,1040.29964105710,
        0,0,1);
    param_.distCoeffR = (cv::Mat_<double>(5, 1) << 
        -0.148803965819541,0.152691806471188,-0.00124043605469856,0.00278208589264770,-0.0448779491567325);

    param_.T = (cv::Mat_<double>(3, 1) << -7.96016216343397,0.0646698136337603,-0.0383012834848667);//T平移向量
    //Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec旋转向量
    param_.R = (cv::Mat_<double>(3,3)<<
        0.999920586961890,-0.0123262085575864,-0.00262380494413116,
        0.0123436584137887,0.999901087092108,0.00674167108252923,
        0.00254044617216446,-0.00677352305792141,0.999973832416944);//R 旋转矩阵
    param_.imageHeight = 1038;
    param_.imageWidth = 2592;

    double cxl = 1323.68935404431;
    double cxr = 1307.88868461770;

    float baseline = 7.96016216343397; 
    float force = 1799.98852787378;

    // t_mat.at<float>(0, 0) = 1;
	// t_mat.at<float>(0, 2) = 0; //水平平移量
	// t_mat.at<float>(1, 1) = 1;
	// t_mat.at<float>(1, 2) = 0; //竖直平移量

    /*立体矫正*/
    cv::Size originSize(2592,2048);
    cv::stereoRectify(param_.cameraMatrixL, param_.distCoeffL, param_.cameraMatrixR,param_.distCoeffR, 
        originSize, param_.R, param_.T, param_.Rl, param_.Rr, param_.Pl, param_.Pr, param_.Q, 
        cv::CALIB_ZERO_DISPARITY,0, originSize);
    initUndistortRectifyMap(param_.cameraMatrixL, param_.distCoeffL, param_.Rl, param_.Pl, 
        originSize, CV_32FC1, param_.mapLx, param_.mapLy);
    initUndistortRectifyMap(param_.cameraMatrixR, param_.distCoeffR, param_.Rr, param_.Pr, 
        originSize, CV_32FC1, param_.mapRx, param_.mapRy);
    
#pragma endregion

    imagePublisherLeft_ =  imageTransport_.advertise(cameraTopicLeft_,1); 
    imagePublisherRight_ = imageTransport_.advertise(cameraTopicRight_,1);

    std::cout<<"autoExposure: ##########################################" <<option_.autoExposure << std::endl;


    CamInit();
}


CamHandlerNode::~CamHandlerNode(){
    streamPtrL_->detachGrabbing( &CamHandlerNode::onGetLeftFrame);
    streamPtrR_->detachGrabbing( &CamHandlerNode::onGetRightFrame);
	if(!(streamPtrL_->stopGrabbing()&&streamPtrR_->stopGrabbing()))
	{
		printf("StopGrabbing  fail.\n");
	}

    if (!(cameraSptrL_->disConnect()&&cameraSptrR_->disConnect()))
	{
		printf("disConnect camera fail.\n");
	}
    printf("disConnect successfully thread ID :%d\n", CThread::getCurrentThreadID());
}

void CamHandlerNode::onGetLeftFrame(const CFrame& pFrame){
    double lastClockTime,t_start,t_end;
    t_start = clock();
    // printf("\r\n");
    ros::Time time = ros::Time::now();

    // 判断帧的有效性
	bool isValid = pFrame.valid();
	if (!isValid)
	{
		printf("frame is invalid!\n");
		return;
	}

	
	// std::cout<<"-------------------------------------------------------------<<<"<<std::endl;

    cv::Mat frame;
    CamHandlerNode::convertFrame(pFrame,frame);
    // ###
    // cv::remap(frame,frame,param_.mapLx,param_.mapLy,cv::INPAINT_TELEA);
    // frame = frame(cv::Rect(0, 600, 2592, 1038));
    // cv::resize(frame,frame,cv::Size(param_.imageWidth,param_.imageHeight));
    // ###
    // t_end = clock();
    // std::cout<<"[Time]left pub convert time:"<<(t_end-t_start)/CLOCKS_PER_SEC<<std::endl;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
    // t_end = clock();
    // std::cout<<"[Time]left pub msg time:"<<(t_end-t_start)/CLOCKS_PER_SEC<<std::endl;
    msg->header.stamp = time;
    imagePublisherLeft_.publish(msg);

    t_end = clock();
    // std::cout<<"[Time]left pub total time:"<<(t_end-t_start)/CLOCKS_PER_SEC<<std::endl;
    printf("[Time]left pub total time:%lf\n",(t_end-t_start)/CLOCKS_PER_SEC);
    // printf("[INFO]get frame %lld successfully thread ID :%d\n", pFrame.getBlockId(), CThread::getCurrentThreadID());
    // std::cout<<"[INFO]Press ctrl+z to stop"<<std::endl;

	return;
}

void CamHandlerNode::onGetRightFrame(const CFrame& pFrame){
    double lastClockTime,t_start,t_end;
    t_start = clock();
    // printf("\r\n");
    ros::Time time = ros::Time::now();

    // 判断帧的有效性
	bool isValid = pFrame.valid();
	if (!isValid)
	{
		printf("frame is invalid!\n");
		return;
	}

	
	// std::cout<<"------------------------------------------------------------->>>"<<std::endl;

    cv::Mat frame;
    CamHandlerNode::convertFrame(pFrame,frame);
    // ###
    // cv::remap(frame,frame,param_.mapRx,param_.mapRy,cv::INPAINT_TELEA);
    // frame = frame(cv::Rect(0, 600, 2592, 1038));
    // cv::resize(frame,frame,cv::Size(param_.imageWidth,param_.imageHeight));
    // ###
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
    msg->header.stamp = time;
    imagePublisherRight_.publish(msg);

    t_end = clock();
    // std::cout<<"[Time]right pub total time:"<<(t_end-t_start)/CLOCKS_PER_SEC<<std::endl;
      printf("[Time]right pub total time:%lf\n",(t_end-t_start)/CLOCKS_PER_SEC);
    // printf("[INFO]get frame %lld successfully thread ID :%d\n", pFrame.getBlockId(), CThread::getCurrentThreadID());
    // std::cout<<"[INFO]Press ctrl+z to stop"<<std::endl;

	return;
}


void CamHandlerNode::convertFrame(const CFrame &pFrame,cv::Mat& frame){
    IMGCNV_SOpenParam conv_param;
	conv_param.width = pFrame.getImageWidth();
	conv_param.height = pFrame.getImageHeight();
	if (conv_param.width%4 != 0){
		conv_param.paddingX = 4 - conv_param.width%4;
	}else
	{
		conv_param.paddingX = 0;
	}
	
	if (conv_param.height%2 != 0){
		conv_param.paddingY = 2 - conv_param.height%2;
	}else
	{
		conv_param.paddingY = 0;
	}
	//转换图片格式
	int nRgbBufferSize = 0;
	nRgbBufferSize = conv_param.height*conv_param.width*3;
	uint8_t* imgBuff =   (uint8_t*)pFrame.getImage();
	uint8_t* img_out = (uint8_t*)malloc(nRgbBufferSize);
	conv_param.dataSize =pFrame.getImageSize(); 
	conv_param.pixelForamt = pFrame.getImagePixelFormat();
	IMGCNV_EErr status =  IMGCNV_ConvertToBGR24_Ex(imgBuff,&conv_param,img_out,&nRgbBufferSize,IMGCNV_DEMOSAIC_BILINEAR);
	
	
	if (IMGCNV_SUCCESS != status)
	{
		/* 释放内存 */
		printf("IMGCNV_ConvertToBGRA24_Ex failed.\n");
		free(imgBuff);
		free(img_out);
		return;
	}
	// std::cout<<"[Time]Time stamp:"<<pFrame.getImageTimeStamp()<<std::endl;
    printf("[Time]Time stamp:%ld\n",pFrame.getImageTimeStamp());
	// std::cout<<"[Size]Source image size: "<<pFrame.getImageSize()<<std::endl;
	// std::cout<<"[Size]CV image size: "<<nRgbBufferSize<<std::endl;
	
	cv::Mat Image(conv_param.height,conv_param.width,CV_8UC3,img_out);
    Image.copyTo(frame);
    free(img_out);
}
