#include "mykinect2.h"
#include <Winsock2.h>
using namespace std;
#define TCP_PORT 4567
#define MAX_BUF 256 // 定义最大缓冲区大小   
// 接收数据模块   
int Receive(SOCKET s, char *szBuf, int len)
{
	int cnt, rc;
	cnt = len;

	// 循环发送数据   
	while (cnt > 0)
	{
		rc = recv(s, szBuf, cnt, 0);// 接收数据   
		if (rc == SOCKET_ERROR)		// 当结束错误时，返回   
		{
			cout << "RECV_ERROR..." << endl;
			return -1;
		}
		if (rc == 0)
			return len - cnt; // 当无数据可接收时，返回   
		szBuf += rc;
		cnt -= rc;
	}
	return len;
}

// 发送数据模块   
int Send(SOCKET s, char *szBuf, int len)
{
	int cnt, rc;
	cnt = len;

	// 循环发送数据   
	while (cnt>0)
	{
		rc = send(s, szBuf, cnt, 0); // 发送数据   
		if (rc == SOCKET_ERROR)    // 当发送数据发生错误时，返回   
		{
			cout << "SEND_ERROR..." << endl;
			return -1;
		}
		if (rc == 0)
			return len - cnt;
		szBuf += rc;
		cnt -= rc;
	}
	return len;
}
//主函数
int main()
{
	Kinect kinect;
	kinect.InitKinect();
	kinect.HeadProcess2();



	// 初始化 Windows Socket DLL
	WSADATA wsaData;
	if (WSAStartup(0x0202, &wsaData))
	{
		cout << "Initialize socket failed." << endl;
		return -1;
	}

	while (TRUE)
	{
		// 初始化 SOCKET
		SOCKET	sServer = socket(AF_INET, SOCK_STREAM, 0);
		if (sServer == INVALID_SOCKET)
		{
			printf(" Failed socket() \n");
			return 0;
		}

		sockaddr_in local;
		local.sin_family = AF_INET;
		local.sin_port = htons(TCP_PORT);  // 连接端口号   
		local.sin_addr.S_un.S_addr = inet_addr("192.168.1.101");  // 连接的服务器IP地址   

		if (connect(sServer, (sockaddr*)&local, sizeof(local)) == -1)   // 连接服务器   
		{
			cout << "Socket connect failed." << endl;
		}

		char szSendMsg[MAX_BUF];
		char szRespMsg[MAX_BUF];

		memset(szSendMsg, 0, MAX_BUF);
		memset(szRespMsg, 0, MAX_BUF);

		cout << "Enter the message to send:";   // 输入向服务器发送的信息   

		cin >> szSendMsg;

		Send(sServer, szSendMsg, MAX_BUF);		// 向服务器发送数据   
		Receive(sServer, szRespMsg, MAX_BUF);	// 接收服务器端返回的数据   

		// 显示回应数据   
		cout << "Server return..." << szRespMsg << endl;
		closesocket(sServer);
	}
	WSACleanup();
	return 0;

}
