#include <stdio.h>  
#include <unistd.h>  
 
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>

#include <sys/time.h>

#include <dirent.h>

//add include to dp server
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <stdarg.h>
#include <stdbool.h>

#define SERVER_PORT 5000
#define BUFFER_SIZE 256
///////

#define SETBITSPEED(opt, s)		\
do					\
{					\
	cfsetispeed(&(opt), B##s);	\
	cfsetospeed(&(opt), B##s);	\
} while(0)

#define CMD_PREFIX	0xeeee
#define CMD_SUFFIX	0xdddddddd

//char *dev[]={"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};
char file_dev[10][128];

//#define g_countTTYUSB	4 //sizeof(dev[0])
int g_fd[10] = {0};			//For serial device
unsigned char g_ifdebug = 0;
int g_countTTYUSB = 0;

///upd data
struct sockaddr_in g_server_addr;
int g_server_socket_fd;

union PARA
{
	char c[4];
	long int d;
} para[5];
union CMD
{
	char c[4];
	long int d;
	struct {
		char flag[2];
		char cmd;
		char rev;
	};
} cmd;
long int timeout = 0;
unsigned char g_isidle[10] = {0};

char buff[24];
char buff_child[1024];

void do_run_cmd(int fd, char buff[24]){
	write(fd, buff, 24);
}

void fill_cmd_head(union CMD *cmd){
	cmd->flag[0] = 0xee;
	cmd->flag[1] = 0xee;
	cmd->rev = 0;
}

void fill_data_tail(union PARA *para){
	para->d = CMD_SUFFIX;
}

void run_cmd(int fd, union CMD *cmd, union PARA para[5]){
	int j;
	fill_cmd_head(cmd);

	fill_data_tail(&para[4]);


	memcpy(buff, cmd->c, 4);
	memcpy(buff+4, para, 20);

	if(g_ifdebug) {
		printf("run_cmd:\n");
		for(j = 0; j < 24; j++)
			printf("%x,", buff[j]);
	
		printf("\n");
	}

	do_run_cmd(fd, buff);
}

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	/* 五个参量 fd打开文件 speed设置波特率 bit数据位设置   neent奇偶校验位 stop停止位 */

	struct termios newtio,oldtio;

	if ( tcgetattr( fd,&oldtio) != 0) { 
		perror("SetupSerial 1");
		return -1;
	}

	bzero( &newtio, sizeof( newtio ) );

	newtio.c_cflag |= CLOCAL | CREAD; 
	newtio.c_cflag &= ~CSIZE; 

	switch( nBits )
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}

	switch( nEvent )
	{
		case 'O':
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E': 
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N': 
			newtio.c_cflag &= ~PARENB;
			break;
	}

	SETBITSPEED(newtio, 9600);

	if( nStop == 1 )
		newtio.c_cflag &= ~CSTOPB;
	else if ( nStop == 2 )
		newtio.c_cflag |= CSTOPB;

	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	tcflush(fd,TCIOFLUSH);

	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}

/*
	   struct termios options;  // 串口配置结构体
	   tcgetattr(fd,&options); //获取当前设置
	   bzero(&options,sizeof(options));

	   options.c_cflag  |= B9600 | CLOCAL | CREAD; // 设置波特率，本地连接，接收使能
	   options.c_cflag &= ~CSIZE; //屏蔽数据位
	   options.c_cflag  |= CS8; // 数据位为 8 ，CS7 for 7 
	   options.c_cflag &= ~CSTOPB; // 一位停止位， 两位停止为 |= CSTOPB
	   options.c_cflag &= ~PARENB;  // 无校验
	   //options.c_cflag |= PARENB; // 有校验
	   //options.c_cflag &= ~PARODD;// 偶校验
	   //options.c_cflag |= PARODD; // 奇校验

	   options.c_cc[VTIME] = 0; // 等待时间，单位百毫秒 （读）。后有详细说明
	   options.c_cc[VMIN] = 0; // 最小字节数 （读）。后有详细说明
	   tcflush(fd, TCIOFLUSH); // TCIFLUSH刷清输入队列。
	   				//TCOFLUSH刷清输出队列。 
					//TCIOFLUSH刷清输入、输出队列。
	   tcsetattr(fd, TCSANOW, &options); // TCSANOW立即生效；
	   					//TCSADRAIN：Wait until everything has been transmitted；
						//TCSAFLUSH：Flush input and output buffers and make the change

*/

	if(g_ifdebug) printf("set_opt: done!\n");
	return 0;
}
int open_port(char const *dev)
{
	/* fd 打开串口 comport表示第几个串口 */
	int fd = open(dev, O_RDWR|		//O_RDWR 读写方式打开；
			   O_NOCTTY|		//O_NOCTTY 不允许进程管理串口（不太理解，一般都选上）；
			   O_NDELAY);		//O_NDELAY 非阻塞（默认为阻塞，打开后也可以使用fcntl()重新设置）

	if (-1 == fd){
		perror("Can't Open Serial Port");
		return(-1);
	}
	else 
		if(g_ifdebug) printf("open %s\n", dev);

	if(fcntl(fd, F_SETFL, 0) < 0)
		printf("fcntl failed!\n");
	else
		if(g_ifdebug) printf("fcntl=%d\n", fcntl(fd, F_SETFL,0));

	if(isatty(STDIN_FILENO)==0)
		printf("standard input is not a terminal device\n");
	else
		if(g_ifdebug) printf("isatty success!\n");

	if(g_ifdebug) printf("fd-open=%d\n",fd);

	return fd;
}
void usage() {

	printf("\n\r\
./control [<-m -x -y -z>] [-l <format>] [-t <format>]\n\r\
  -m: command: A,B,C,V,v,	\n\r\
  -x: oppsited x distance	\n\r\
  -y: oppsited y distance	\n\r\
  -z: oppsited z distance	\n\r\
\n\r\
  -l <format>: long command\n\r\
        format: <command>,<x>,<y>,<z>	\n\r\
  -t <format>: long command with timeout	\n\r\
        format:<timeout>,<command>,<x>,<y>,<z>	\n\r\
  		timeout in millisecond	\n\r\
\n");
}
int isidle(int j) {
	union CMD t_cmd;
	union PARA t_para[5];

	int count = 0;
	int nread = 0;
	char tmp_tv[100] = {0},tmp[100] = {0};
	int i = 0;

	memset(&t_cmd, 0, sizeof(union CMD));
	memset(t_para, 0, 5 * sizeof(union PARA));
	memset(buff_child, 0, 1024);

	struct timeval tv;  
	gettimeofday(&tv, 0);

	sprintf(tmp_tv, "copy %lu:%lu",tv.tv_sec, tv.tv_usec);  	//for string searching!

	if (g_ifdebug)
		printf("isidle: checking string: %s\n", tmp_tv);

	t_cmd.cmd = 'C';
	t_para[0].d = tv.tv_sec;
	t_para[1].d = tv.tv_usec;

	run_cmd(g_fd[j], &t_cmd, t_para);
	fsync(g_fd[j]);
	count = 0;

	for( i = 0; i < timeout * 100; i++) {  //unit of timeout is millisecond
		usleep(10);
		nread = read(g_fd[j], tmp, 100);
		if(nread) {
			memcpy(buff_child + count, tmp, nread);
			count += nread;
		}

		if (g_ifdebug)
			printf("received from arduino(%d) %s\n",count, buff_child);
	
		if( strstr(buff_child, tmp_tv) != NULL) {
			if(g_ifdebug) printf("g_isidle[%d] = 1\n", j);
			g_isidle[j] = 1;
			return 1;
		}
	}
	return 0;
}
void listDir(char *path)
{
	DIR *pDir;
	struct dirent *ent;
	char childpath[512];

	pDir = opendir(path);
	memset(childpath, 0, sizeof(childpath));

	while((ent = readdir(pDir)) != NULL) //read pDir
	{
		//if type is NOT DT_DIR which is a file, print name directly
		//this is also the exit of recursive
		if (strstr(ent->d_name, "Arduino") != NULL) {
			sprintf( file_dev[g_countTTYUSB], "%s/%s", path, ent->d_name);
			g_countTTYUSB++;
		}
	}
}
void udpInit()
{
	//<1>create the connector of socke
	bzero(&g_server_addr,sizeof(g_server_addr));
	g_server_addr.sin_family = AF_INET;
	g_server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	g_server_addr.sin_port = htons(SERVER_PORT);

	//<2>create socket
	g_server_socket_fd = socket(AF_INET,SOCK_DGRAM, 0);
	if(g_server_socket_fd == -1)
	{
		perror("Create Socket Failed:");
		exit(1);
	}

	//<3>bind upd connector
	if(-1 == (bind(g_server_socket_fd, (struct sockaddr*)&g_server_addr, sizeof(g_server_addr))))
	{
		perror("Server Bind Failed:");
		exit(1);
	}
}

void udpserver()
{
	int n;			//receiver data length
	//<1>data transmit
	//[1]degfine a address to get the address of client
	struct sockaddr_in client_addr;
	socklen_t client_addr_length = sizeof(client_addr);
	//[2]receive data
	char buffer[BUFFER_SIZE];
	bzero(buffer,BUFFER_SIZE);


	n = recvfrom(g_server_socket_fd, buffer, BUFFER_SIZE, 0,
			(struct sockaddr*)&client_addr, &client_addr_length);
	if(n > 0)
	{
		buffer[n] = 0;
		sscanf(buffer, "%ld,%c,%ld,%ld,%ld,%ld",
			&timeout, &cmd.cmd, &para[0].d, &para[1].d, &para[2].d, &para[3].d);
		printf("opt with optopt: %ld,%c,%ld,%ld,%ld,%ld\n",
			timeout, cmd.cmd, para[0].d, para[1].d, para[2].d, para[3].d);
		sendto(g_server_socket_fd, "Success", 7, 0,
			(struct sockaddr*)&client_addr, sizeof(client_addr));
	}

	return;
}

int main(int argc, char **argv)  
{  
	int ch;  
	int i, j;
	bool udp_enter = false;

	memset(&cmd, 0, 4);
	memset(para, 0, 20);
	memset(file_dev, 0, 128 * 10);

	///////////////////////
	// Command options
	//
	opterr = 0;  
	while ((ch = getopt(argc, argv, "dum:x:y:z:c:l:t:")) != -1)
	{  
		switch(ch)  
		{  
			case 'm':  
				cmd.cmd = optarg[0];
				break;	
			case 'x':  
				para[0].d = atoi(optarg);
				break;	
			case 'y':  
				para[1].d = atoi(optarg);
				break;	
			case 'z':  
				para[2].d = atoi(optarg);
				break;
			case 'c':
				para[3].d = atoi(optarg);
				break;
			case 'l':  
				sscanf(optarg, "%c,%ld,%ld,%ld,%ld",
						&cmd.cmd, &para[0].d, &para[1].d, &para[2].d, &para[3].d);
				break;	
			case 't':  
				sscanf(optarg, "%ld,%c,%ld,%ld,%ld,%ld",
						&timeout, &cmd.cmd, &para[0].d, &para[1].d, &para[2].d, &para[3].d);
				break;	
			case 'u':
				udpInit();
				udp_enter = true;
				break;
			case 'd':
				g_ifdebug = 1;
				break;
			case '?':
			default:
				usage();
				exit(0);
		}  
		printf("opt [%c] with optopt: %s\n", ch, optarg);	
	}

	listDir("/dev");

	for(i = 0; i < g_countTTYUSB; i++) {
		//////////////////////////
		// Open serial port
		// By default open ttyUSB0
		//
		if((g_fd[i] = open_port(file_dev[i])) < 0){
			perror("open_port error");
			return -1;
		}

		//////////////////////////
		// Set serial port
		//
		if((set_opt(g_fd[i], 9600, 8, 'N', 1)) < 0){
			perror("set_opt error");
			return -2;
		}
	}


	if(timeout > 0) {
		for(j = 0; j < g_countTTYUSB; j++) {
			isidle(j);
		}

	} else
		for (j = 0; j < g_countTTYUSB; j++)
			g_isidle[j] = 1;
	do {
		if (udp_enter)
		{
			memset(&cmd, 0, 4);
			memset(para, 0, 20);
			memset(file_dev, 0, 128 * 10);
			udpserver();
		}
		for(j = 0; j < g_countTTYUSB; j++) {
			if (g_isidle[j])
				run_cmd(g_fd[j], &cmd, para);
		}
	} while(udp_enter);

	for(j = 0; j < g_countTTYUSB; j++)
		close(g_fd[j]);

	return 0;
}

