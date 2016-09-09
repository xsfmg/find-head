/********************************************************************
Copyritht:BMI
Author:chuangzeng
Date:2016-04-26
Description:this hpp  include procedures call interface
**********************************************************************/
#pragma once
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <Kinect.h>// Kinect Header files
using namespace cv;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class Kinect
{
public:
	/***************输入麦克风到kinect坐标以及座位分割阈值******************/
	 const float	micposionX=0;
	 const float	micposionY=0;
	 const float	micposionZ=0;
	 const float	Headthreshold = 1000; 
	/**********************************************************************/
	static const int    cDepthWidth = 512;
	static const int    cDepthHeight = 424;
	struct Person
	{
		float x;
		float y;
		float z;
	}HPoint;
	std::vector<Person>HeadPoint;
	struct Head_Area
	{
		int x;
		int y;
		int width;
		int height;
	}pHead;
	std::vector<Head_Area>HeadArea;
	Kinect();
	~Kinect();
	HRESULT			InitKinect();					//initialise Kinect				
	bool			HeadProcess2();					//HeadPosion function
	bool			HeadProcess();					//not in my program, just use it to debug 
private:
	IKinectSensor*          m_pKinectSensor;		// Current Kinect
	IDepthFrameReader*      m_pDepthFrameReader;	// Color reader
	IDepthFrame*			pDepthFrame ;		//depth frame
	ICoordinateMapper*		pCoordinateMapper;		//Mapper
	RGBQUAD*                m_pDepthRGBX;			//color RGB
	UINT16*					pBuffer;			//define 16 pbuffer
	Mat						headpro;				//head image 
	int				PersonNum;
	bool			result;		//Returns the result of execution 
	bool			Update();	//refresh frame into processing
	bool			ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);//Depth processing
	bool			get_DepthContour(cv::Mat src);		//get depth contour
	bool			Getpoint(int x, int y, int width, int height, vector<UINT16> depthBuffer);
	bool			Head_Point();
	bool			Mapcamera2mic(float*	kinectHposionX, float* kinectHposionY, float* kinectHposionZ);
};