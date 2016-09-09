/********************************************************************
Copyritht:BMI
Author:chuangzeng
Date:2016-04-26
Description:this cpp  include program and algorithm about Head positioning
**********************************************************************/
#include "mykinect2.h"
Kinect::Kinect()
{
	m_pKinectSensor = NULL;
	m_pDepthFrameReader = NULL;
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];// create heap storage for color pixel data in RGBX format
	pDepthFrame = NULL;
	pBuffer = NULL;
}

Kinect::~Kinect()
{
	if (m_pDepthRGBX)
	{
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}

	SafeRelease(m_pDepthFrameReader);// done with color frame reader

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();// close the Kinect Sensor
	}
	SafeRelease(m_pKinectSensor);
}

HRESULT	Kinect::InitKinect()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		SafeRelease(pDepthFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found! \n");
		return E_FAIL;
	}

	return hr;
}

bool Kinect::HeadProcess2()
{

	while (1)
	{
		if (result = Update())
		{
			std::cout << "ERROR:Update() failed!" << std::endl;
		}
		HeadPoint.clear();
		if (waitKey(1) >= 0)//按下任意键退出
		{
			break;
		}
	}
	return 0;


}
bool Kinect::Update()
{
	if (!m_pDepthFrameReader)
	{
		return 0;
	}
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr))
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		if (SUCCEEDED(hr))
		{
			hr=m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))
		{
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if (SUCCEEDED(hr))
		{

			result=ProcessDepth(pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
		}
		if (!result)
		{

			result=get_DepthContour(headpro);
		}
		else
		{
			std::cerr << "ERROR:ProcessDepth operation failed" << std::endl;
		}
		if (!result)
		{
			
			result = Head_Point();
		}
		else
		{
			std::cerr << "ERROR:Head_Point operation failed" << std::endl;
		}
		
		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
	return 0;
}

/***************************************************************************************
						The algorithm of finding head posion
***************************************************************************************/

bool Kinect::ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;
			//std::cout << *pBuffer << std::endl;
			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			if (depth < Headthreshold)
			{
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (-depth * 256 / 4900 + 256) : 255);
			    pRGBX->rgbRed = intensity;
			    pRGBX->rgbGreen = intensity;
			    pRGBX->rgbBlue = intensity;
		   }
			else
			{
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? 255: 255);
				pRGBX->rgbRed = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue = intensity;
			}
			++pRGBX;
			++pBuffer;
		}

		// Draw the data with OpenCV
		Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
		Mat show = DepthImage.clone();
		headpro = show;
		//imshow("DepthImage", show);
		
	}
	return 0;
}

//get depthMat contours
bool Kinect::get_DepthContour(cv::Mat depthMat)
{
	Mat SdepthMat;  //单通道的深度图
	cv::Mat dst, canny_output;
	cv::vector<cv::Vec4i>hierarchy;
	std::vector<std::vector<cv::Point> > contours;
	if (!depthMat.data)
	{
		std::cout << "read data error!" << std::endl;
		return 0;
	}
	cvtColor(depthMat,SdepthMat , CV_BGR2GRAY);
	cv::medianBlur(SdepthMat, SdepthMat, 3);
	//blur(SdepthMat, SdepthMat, Size(5, 5), Point(-1, -1));
	dilate(SdepthMat, SdepthMat, Mat(5, 5, CV_8U), Point(-1, -1),3);
	cv::Canny(SdepthMat, canny_output, 80, 255, 3);
	//imshow("canny",canny_output);
	cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	const int minarea = 10;
	double maxarea = 0;
	double maxAreaIdx = 0;
	for (unsigned int i = 0; i<contours.size(); i++)
	{
		double tmparea = fabs(contourArea(contours[i]));
		if (tmparea>maxarea)
		{
			maxarea = tmparea;
			maxAreaIdx = i;
			continue;
		}
		if (tmparea < minarea)
		{
			contours.erase(contours.begin() + i);
			continue;
		}
		
	}
	// Draw contours 
	for (unsigned int i = 0; i< contours.size(); i++)
	{
		cv::Rect Rect = boundingRect(contours[i]);
		pHead.x = Rect.x;
		pHead.y = Rect.y;
		pHead.width = Rect.width;
		pHead.height = Rect.height;
		HeadArea.push_back(pHead);
		/*
		for (int i = 0; i < HeadArea.size(); i++)
		{
			std::cout << HeadArea[i].x << std::endl;
		}
		*/

		rectangle(depthMat, Rect.tl(), Rect.br(), cv::Scalar(255, 0, 0), 2, 8, 0);

	}
	imshow("headContour",depthMat);
	PersonNum = contours.size();

	//std::cout << PersonNum << std::endl;
	return 0;
}

bool Kinect::Head_Point()
{
	std::vector<UINT16> depthBuffer(cDepthWidth* cDepthHeight);
	pDepthFrame->CopyFrameDataToArray(cDepthWidth* cDepthHeight, &depthBuffer[0]);
	for (int i = 0; i < PersonNum; i++)
	{
		std::cout << "第" << i+1 << "个人" << std::endl;
		Getpoint(HeadArea[i].x, HeadArea[i].y, HeadArea[i].width, HeadArea[i].height,depthBuffer);
		Mapcamera2mic(&HeadPoint[i].x, &HeadPoint[i].y, &HeadPoint[i].z);
		std::cout << "X:" << HeadPoint[i].x << "	Y:" << HeadPoint[i].y << "	Z:" << HeadPoint[i].z<< std::endl;

	}
	return 0;
}
bool Kinect::Getpoint(int x, int y, int width, int height, vector<UINT16> depthBuffer)
{
	CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
	std::vector<float> depthZ;
	std::vector<float> depthX;
	std::vector<float> depthY;
	for (int Ystart = y; Ystart<= y+height; Ystart++)
	{
		for (int Xstart = x; Xstart <=x+width; Xstart++)
		{
		DepthSpacePoint depthSpacePoint = { static_cast<float>(Xstart ), static_cast<float>(Ystart) };
		UINT16 depth = depthBuffer[Ystart*cDepthWidth + Xstart];
		
		//整理人头区域中的有效点，并且进行深度坐标系和相机坐标系的转换
		if (depth > 0)
		{
			pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
			HPoint.x = cameraSpacePoint.X;
			HPoint.y = cameraSpacePoint.Y;
			HPoint.z = cameraSpacePoint.Z;
			depthZ.push_back(HPoint.z);
			depthY.push_back(HPoint.y);
			depthX.push_back(HPoint.x);
		}
		else
		continue;

		}
	}
	std::vector<float>::iterator minZ= min_element(depthZ.begin(), depthZ.end());
	int num=std::distance(std::begin(depthZ), minZ);
	HPoint.x = depthX[num];
	HPoint.y = depthY[num];
	HPoint.z= depthZ[num];
	HeadPoint.push_back(HPoint);
	depthZ.clear();
	depthY.clear();
	depthX.clear();
	return 0;
}

bool Kinect::HeadProcess()
{
	Mat depthMat2(cDepthHeight, cDepthWidth, CV_8UC1);
	string dst_img_name = "E://数据输出//depth//";  //深度图像存储地址
	char chari[1000];
	int n = 0;
	while (1)
	{
		Update();
		sprintf_s(chari, "%04d", n);
		dst_img_name += "depth_";
		dst_img_name += chari;
		dst_img_name += ".PNG";
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(dst_img_name, headpro, compression_params);
		dst_img_name = "E://数据输出//depth//";
		//n++;
		if (waitKey(1) >= 0)//按下任意键退出
		{
			break;
		}
	}
	return 0;


}
bool Kinect::Mapcamera2mic(float* kinectHposionX, float* kinectHposionY, float* kinectHposionZ)
{
	*kinectHposionX = *kinectHposionX - micposionX;
	*kinectHposionY = *kinectHposionY - micposionY;
	*kinectHposionZ = *kinectHposionZ - micposionZ;
	return 0;
}