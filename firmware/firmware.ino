#include <EEPROM.h>                     

#define MAX_QUEUE	512
#define CMD_SIZE	(5 * 4)

byte pin_x=3;              //x方向电机驱动脉冲端口
byte pin_y=5;             //y方向电机机驱动脉冲端口
byte pin_z=9;            //z方向电机机驱动脉冲端口


byte dir_x=4;            //x方向电机方向端口
byte dir_y=6;           //y方向电机方向端口
byte dir_z=10;          //z方向电机方向端口

byte enable_x=7;          //x方向电机使能端口
byte enable_y=8;         //y方向电机使能端口
byte enable_z=11;       //z方向电机使能端口

//用EEPROM寄存器保存坐标地址
byte address_x_HIGH=1;
byte address_x_LOW=2;
byte address_y_HIGH=3;
byte address_y_LOW=4;
byte address_z_HIGH=5;
byte address_z_LOW=6;

//EEPROM寄存器保存的高低位数
int value_x_high;
int value_x_low;
int value_y_high;
int value_y_low;
int value_z_high;
int value_z_low;
int value_time_0;
int value_time_1;
int value_time_2;
int value_time_3;

//电机参数
int objpos_x,objpos_y,objpos_z;           //x，y，z方向目标位置坐标
int curpos_x,curpos_y,curpos_z;          //x，y，z方向当前目标坐标
int distance_x,distance_y,distance_z;   //x，y，z方向所需移动位移

//位移与角度转化比
int DISSWANG_X=95;                      //50轨道
int DISSWANG_Y=75;                      //45轨道
int DISSWANG_Z=95;                     //50轨道
int pulse_rate_x=4000;                //x方向电机旋转一圈所需脉冲数
int pulse_rate_y=400;                //y方向电机旋转一圈所需脉冲数
int pulse_rate_z=4000;              //z方向电机旋转一圈所需脉冲数

int delay_lowlevenum_xlow=250;          //x方向电机低速低电平脉冲延迟数
int delay_lowlevenum_xmid=150;          //x方向电机低速中电平脉冲延迟数
int delay_lowlevenum_xhig=100;          //x方向电机低速高电平脉冲延迟数
int delay_lowlevenum_ylow=150;          //y方向电机低速低电平脉冲延迟数
int delay_lowlevenum_ymid=100;          //y方向电机低速中电平脉冲延迟数
int delay_lowlevenum_yhig=75;          //y方向电机低速高电平脉冲延迟数
int delay_lowlevenum_zlow=250;          //z方向电机低速低电平脉冲延迟数
int delay_lowlevenum_zmid=150;          //z方向电机低速中电平脉冲延迟数
int delay_lowlevenum_zhig=100;          //z方向电机低速高电平脉冲延迟数


//串口数据
int string_head = 0;                                //数据头指针地址
int string_end = 0;                                 //数据尾指针地址      
unsigned char * comdata;                           //读取串口数据                                

//获取机器运行的时间长度, 单位毫秒
unsigned long motor_time;
unsigned long motor_time_head;
unsigned long motor_time_end;
byte address_time_0=7;
byte address_time_1=8;
byte address_time_2=9;
byte address_time_3=10;
char * time_data;


union PARA
{
	unsigned char c[4];  
	long int d;
};
union PARA para[4];
union CMD
{
	unsigned char c[4];  
	long int d;
	struct cmddata
	{
		unsigned char flag[2];
		unsigned char mode;
		unsigned char getzero;
	}data;  
}cmd;

bool g_processFlag = false;

void setup() {
	// put your setup code here, to run once:
	//---===引脚模式设置===---// 
	pinMode(pin_x,OUTPUT);
	pinMode(pin_y,OUTPUT);
	pinMode(pin_z,OUTPUT);
	pinMode(dir_x,OUTPUT);
	pinMode(dir_y,OUTPUT);
	pinMode(dir_z,OUTPUT);
	pinMode(enable_x,OUTPUT);
	pinMode(enable_y,OUTPUT);
	pinMode(enable_z,OUTPUT);
	//---===各参数清零===---//
	motor_time=0;
	motor_time_head=0;
	motor_time_end=0;
	string_head=0;                                
	string_end=0; 

	//---===分配空间===---//
	comdata = (unsigned char*)malloc(sizeof(char)*800);
	time_data = (char*)malloc(sizeof(long));

	memset(comdata, 0, 800);
	memset(time_data, 0, 1);


	//---===读取EEPROM中参数赋给当前坐标===---//
	value_x_high=EEPROM.read(address_x_HIGH);
	value_x_low=EEPROM.read(address_x_LOW);
	value_y_high=EEPROM.read(address_y_HIGH);
	value_y_low=EEPROM.read(address_y_LOW);
	value_z_high=EEPROM.read(address_z_HIGH);
	value_z_low=EEPROM.read(address_z_LOW);

	if((value_x_high==255)||(value_y_high==255)||(value_z_high=255))
	{
		//初始化各地址初始值
		EEPROM.write(address_x_HIGH, 0);
		EEPROM.write(address_x_LOW, 0);
		EEPROM.write(address_y_HIGH, 0);
		EEPROM.write(address_y_LOW, 0);
		EEPROM.write(address_z_HIGH, 0);
		EEPROM.write(address_z_LOW, 0);
		curpos_x=0;
		curpos_y=0;
		curpos_z=0;
	}
	else
	{
		curpos_x=value_x_high*256+value_x_low;
		curpos_y=value_y_high*256+value_y_low;
		curpos_z=value_z_high*256+value_z_low;
	}

	//---===使能端有效===---//
	pinMode(enable_x,LOW);
	pinMode(enable_y,LOW);
	pinMode(enable_z,LOW);

	Serial.begin(9600);           //打开串口
}

int g_inputCount = 0;

void parseData(unsigned char *comdata)
{
	bool found_head = false;

	//if(comdata[string_head % 512]==0xff && comdata[(string_head+1) % 512]==0xff)
	Serial.print(cmd.data.flag[0],HEX);
	Serial.print(", ");
	Serial.print(cmd.data.flag[1],HEX);

	while( (string_head + 1) % MAX_QUEUE != string_end )
	{
		if(comdata[string_end] == 0xff && comdata[string_end + 1] == 0xff){
			found_head = true; 
			break;
		} else {
			string_end ++;
		}
	}

	if(found_head) {
		memcpy(cmd.c, comdata + string_end, 4);
		string_end = (string_end + 4) % MAX_QUEUE;

		for(int x=0;x<4;x++)
		{
			memcpy(para[x].c, comdata + string_end, sizeof(para[x].c));
			string_end = (string_end + sizeof(para[x].c) ) % MAX_QUEUE;
		}

		found_head = false;
	}
	return;
}

void printGlobalCount(int count)
{
	Serial.print("\r\n(");
	Serial.print(count, DEC);
	Serial.print(')');
}

void printChar(unsigned char c)
{
	Serial.print(c, HEX);
	Serial.print(',');
}
void stop_motor()               //暂停
{
    pinMode(pin_x,LOW);
    pinMode(pin_y,LOW);
    pinMode(pin_z,LOW);
    pinMode(enable_x,HIGH);
    pinMode(enable_y,HIGH);
    pinMode(enable_z,HIGH);
}

/*
 * 获得电机当前频率;
 * char sort：电机类别
 * int  n:需要的脉冲数; 
 * int & tf:频率;    
*/

void GetFreData(char sort,unsigned int n,int & tf)
{
      int pulse_rate = 1;        //脉冲比值
      int delay_lowlevenum_low = 0;          //电机低速低电平脉冲延迟数
      int delay_lowlevenum_mid = 0;          //电机中速低电平脉冲延迟数
      int delay_lowlevenum_hig = 0;          //电机高速低电平脉冲延迟数
      switch(sort)
      {
        case 'x':                //x方向电机
             pulse_rate = pulse_rate_x;
             delay_lowlevenum_low = delay_lowlevenum_xlow;
             delay_lowlevenum_mid = delay_lowlevenum_xmid;
             delay_lowlevenum_hig = delay_lowlevenum_xhig;
             break;
        case 'y':             //y方向电机
             pulse_rate=pulse_rate_y;
             delay_lowlevenum_low = delay_lowlevenum_ylow;
             delay_lowlevenum_mid = delay_lowlevenum_ymid;
             delay_lowlevenum_hig = delay_lowlevenum_yhig;
             break;
        case 'z':             //z方向电机
             pulse_rate = pulse_rate_y;
             delay_lowlevenum_low = delay_lowlevenum_zlow;
             delay_lowlevenum_mid = delay_lowlevenum_zmid;
             delay_lowlevenum_hig = delay_lowlevenum_zhig;
             break;
        default:
             Serial.println("ERROR:motor sort error！！！");
             break;
      }
  
      unsigned int x = n / pulse_rate;
      if(x > 6)
            tf = delay_lowlevenum_hig;
      else if(x > 2)
            tf = delay_lowlevenum_mid;
      else
            tf = delay_lowlevenum_low;
}

/*
 * 电机驱动;
 * char sort：电机类别
 * int & n:需要的脉冲数; 
 * int & tf:频率;
 * int & count:低电平延迟数
 * boolean & flag:计数标志位
*/
void motor(char sort,unsigned int & n,int & tf,int & count,boolean & flag)
{
    int pin = 3;           //脉冲端口
    if(n==0)
    {
        flag=true;
    }
    switch(sort)
    {
      case 'x':                //x方向电机
           pin=pin_x;
           break;
      case 'y':              //y方向电机
           pin=pin_y;
           break;
      case 'z':             //z方向电机
           pin=pin_z;
           break;
      default:
           Serial.println("ERROR:motor sort error！！！");
           break;
    }
    
    if(flag==false)
    {
        if(count==tf)
        {
            digitalWrite(pin,HIGH);
            count=0;
            n--;
            //获得下一脉冲所需脉冲下的电机低电平延迟数
            GetFreData(sort,n,tf);
        }
        else
        {
            digitalWrite(pin,LOW);
            count++;
        }
    }
}

/*
 * 运动规划；
 * int s_x:x方向所需运动位移；
 * int s_y:y方向所需运动位移；
 * int s_z:z方向所需运动位移；
*/
void dmotor(int s_x,int s_y,int s_z)
{
     //定义基本变量
     int tf_x,tf_y,tf_z;
     int s_x_abs=0,s_y_abs=0,s_z_abs=0;  
     int count_x=0,count_y=0,count_z=0;                  //x，y，z方向电机计数器
     boolean flag_x=false,flag_y=false,flag_z=false;                //x，y，z方向电机完成任务信号标志
     //使能端开启有效
     pinMode(enable_x,LOW);
     pinMode(enable_y,LOW);
     pinMode(enable_z,LOW);
    
     //判断x，y，z方向电机转动方向
     if(s_x<0)                           //x方向电机反转
     {
         digitalWrite(dir_x,LOW);
         s_x_abs=abs(s_x);
     }
     if(s_x>0)                          //x方向电机正转
     {
         digitalWrite(dir_x,HIGH);
         s_x_abs=abs(s_x);
     }
     
     if(s_y<0)                           //y方向电机反转
     {
         digitalWrite(dir_y,LOW);
         s_y_abs=abs(s_y);
     }
     if(s_y>0)                          //y方向电机正转
     {
         digitalWrite(dir_y,HIGH);
         s_y_abs=abs(s_y);
     }
     
     if(s_z<0)                           //x方向电机反转
     {
         digitalWrite(dir_z,LOW);
         s_z_abs=abs(s_z);
     }
     if(s_z>0)                          //x方向电机正转
     {
         digitalWrite(dir_z,HIGH);
         s_z_abs=abs(s_z);
     }

     //将距离转换为脉冲数
     unsigned int pulse_x=int(s_x_abs * (4000.0 / DISSWANG_X));
     unsigned int pulse_y=int(s_y_abs * (400.0 / DISSWANG_Y));
     unsigned int pulse_z=int(s_z_abs * (4000.0 / DISSWANG_Z));
     Serial.println(pulse_x);
     Serial.println(pulse_y);
     Serial.println(pulse_z);
     //分别获得x,y,z方向频率
     GetFreData('x',pulse_x,tf_x);
     GetFreData('y',pulse_y,tf_y);
     GetFreData('z',pulse_z,tf_z);

     motor_time_end=millis();                       //开始记录时间
     while(flag_x==false||flag_y==false||flag_z==false)
     {
          motor('x',pulse_x,tf_x,count_x,flag_x);
          motor('y',pulse_y,tf_y,count_y,flag_y);
          motor('z',pulse_z,tf_z,count_z,flag_z);
     }
     motor_time_head=millis();                    //记录运行结束时间
     motor_time=motor_time_head-motor_time_end;  //获得运行时间

      //将当前位置保存至EEPROM中
      value_time_0=curpos_x%256;
      value_time_1=(curpos_x/256)%256;
      value_time_2=(curpos_x/256/256)%256;
      value_time_3=(curpos_x/256/256/256)%256;

      EEPROM.write(address_time_0, value_time_0);
      EEPROM.write(address_time_1, value_time_1);
      EEPROM.write(address_time_2, value_time_2);
      EEPROM.write(address_time_3, value_time_3);
      
     //将目标位置赋值给当前位置
     curpos_x=objpos_x;
     curpos_y=objpos_y;
     curpos_z=objpos_z;

     //将当前位置保存至EEPROM中
    value_x_high=curpos_x/256;
    value_x_low=curpos_x%256;
    value_y_high=curpos_x/256;
    value_y_low=curpos_x%256;
    value_z_high=curpos_x/256;
    value_z_low=curpos_x%256;

    EEPROM.write(address_x_HIGH, value_x_high);
    EEPROM.write(address_x_LOW, value_x_low);
    EEPROM.write(address_y_HIGH, value_y_high);
    EEPROM.write(address_y_LOW, value_y_low);
    EEPROM.write(address_z_HIGH, value_z_high);
    EEPROM.write(address_z_LOW, value_z_low);
     
     //标志位复位
     flag_x=false;
     flag_y=false;
     flag_z=false;

}

void processMotor()
{
	switch(cmd.data.mode)
	{
		case 'A':                       //move:输入的x,y,z是绝对坐标
			objpos_x=para[0].d;
			objpos_y=para[1].d;
			objpos_y=para[2].d;
			distance_x=objpos_x-curpos_x;                 //所需运行位移
			distance_y=objpos_y-curpos_y;                //所需运行位移
			distance_z=objpos_z-curpos_z;               //所需运行位移

			dmotor(distance_x,distance_y,distance_z);       //电机驱动

			Serial.println("Here !!!!!");
			break;
		case 'B':                     //move:输入的x,y,z是相对坐标
			distance_x=para[0].d;
			distance_y=para[1].d;
			distance_z=para[2].d;
			objpos_x=distance_x+curpos_x;                 //所需运行位移
			objpos_y=distance_y+curpos_y;                //所需运行位移
			objpos_z=distance_z+curpos_z;               //所需运行位移

			dmotor(distance_x,distance_y,distance_z);       //电机驱动
			break;
		case 'C':                     //stop:电机停止
			void stop_motor(); 
			break;
		case 'D':                     //getTimedata:获得上次运行的时间
			value_time_0=EEPROM.read(address_time_0);
			time_data[0] = char(value_time_0);
			value_time_1=EEPROM.read(address_time_1);
			time_data[1] = char(value_time_1);
			value_time_2=EEPROM.read(address_time_2);
			time_data[2] = char(value_time_2);
			value_time_3=EEPROM.read(address_time_3);
			time_data[3] = char(value_time_3);
//			Serial.println(time_data);
			memset(time_data, 0, 4);
			break;       
	}
}

//get COM data
void GetCOM_Data()
{
	//获取串口字符赋给comdata字符串，并获得结束位置 
	while (Serial.available() > 0)  
	{
		printGlobalCount( ++g_inputCount );

		if(string_head - string_end == -1 || string_head - string_end == 511)
		{
			Serial.println("ERROR:OVERFLOW!!!");
			string_head = string_end = 0;

			return;
		}

		comdata[string_head] = Serial.read();
		printChar( comdata[string_head] );

		if(comdata[string_head] == 0 &&
			comdata[string_head-1] == 0 &&
			comdata[string_head-2] == 0 &&
			comdata[string_head-3] == 0 )

			g_processFlag = true;

		string_head++;
		string_head = (string_head) % MAX_QUEUE;

	}
	//提取20位有用信号字符,并获得起点位置

	while (string_head != string_end && g_processFlag)
	{
		int n = 0;

		parseData(comdata);

		processMotor();
		Serial.print("\r\n  * ");
		Serial.print(cmd.data.mode);
		Serial.print(", ");
		for(n = 0; n<4; n++){
			Serial.print(para[n].d);
			Serial.print(", ");
		}

//		string_end = (string_end + CMD_SIZE)&MAX_QUEUE;
	}

	g_processFlag = false;
}
void loop() {

	// put your main code here, to run repeatedly:
	GetCOM_Data();

	//根据获得的运动模式指令，确定运行方式
}
