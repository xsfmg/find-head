#include "mykinect2.h"
#include <Winsock2.h>
using namespace std;
#define TCP_PORT 4567
#define MAX_BUF 256 // ������󻺳�����С   
// ��������ģ��   
int Receive(SOCKET s, char *szBuf, int len)
{
	int cnt, rc;
	cnt = len;

	// ѭ����������   
	while (cnt > 0)
	{
		rc = recv(s, szBuf, cnt, 0);// ��������   
		if (rc == SOCKET_ERROR)		// ����������ʱ������   
		{
			cout << "RECV_ERROR..." << endl;
			return -1;
		}
		if (rc == 0)
			return len - cnt; // �������ݿɽ���ʱ������   
		szBuf += rc;
		cnt -= rc;
	}
	return len;
}

// ��������ģ��   
int Send(SOCKET s, char *szBuf, int len)
{
	int cnt, rc;
	cnt = len;

	// ѭ����������   
	while (cnt>0)
	{
		rc = send(s, szBuf, cnt, 0); // ��������   
		if (rc == SOCKET_ERROR)    // ���������ݷ�������ʱ������   
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
//������
int main()
{
	Kinect kinect;
	kinect.InitKinect();
	kinect.HeadProcess2();



	// ��ʼ�� Windows Socket DLL
	WSADATA wsaData;
	if (WSAStartup(0x0202, &wsaData))
	{
		cout << "Initialize socket failed." << endl;
		return -1;
	}

	while (TRUE)
	{
		// ��ʼ�� SOCKET
		SOCKET	sServer = socket(AF_INET, SOCK_STREAM, 0);
		if (sServer == INVALID_SOCKET)
		{
			printf(" Failed socket() \n");
			return 0;
		}

		sockaddr_in local;
		local.sin_family = AF_INET;
		local.sin_port = htons(TCP_PORT);  // ���Ӷ˿ں�   
		local.sin_addr.S_un.S_addr = inet_addr("192.168.1.101");  // ���ӵķ�����IP��ַ   

		if (connect(sServer, (sockaddr*)&local, sizeof(local)) == -1)   // ���ӷ�����   
		{
			cout << "Socket connect failed." << endl;
		}

		char szSendMsg[MAX_BUF];
		char szRespMsg[MAX_BUF];

		memset(szSendMsg, 0, MAX_BUF);
		memset(szRespMsg, 0, MAX_BUF);

		cout << "Enter the message to send:";   // ��������������͵���Ϣ   

		cin >> szSendMsg;

		Send(sServer, szSendMsg, MAX_BUF);		// ���������������   
		Receive(sServer, szRespMsg, MAX_BUF);	// ���շ������˷��ص�����   

		// ��ʾ��Ӧ����   
		cout << "Server return..." << szRespMsg << endl;
		closesocket(sServer);
	}
	WSACleanup();
	return 0;

}
