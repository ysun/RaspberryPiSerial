#include <EEPROM.h>			

#define MAX_QUEUE	512
#define CMD_SIZE	(5 * 4)

#define PIN_PULS 5
#define PIN_DIR 6
#define PIN_ENABLE 8

#define PIN_INTER 1  //Equal to pin 3; 0 means pin 2 on the board
#define PIN_INTER_RIGHT 10 
#define PIN_INTER_LEFT 9

#define TRAN_RATION 75

#define PULSE_RATE 400

#define ADDRESS_LOW 2
#define ADDRESS_HIGH 1

//EEPROM寄存器保存的高低位数
int value_high;
int value_low;
int value_time_0;
int value_time_1;
int value_time_2;
int value_time_3;

//电机参数
int objpos_coor;	   //target pos
int curpos_coor;	  //curret pos
int distance_coor;	 //distance of moving


boolean inter_right = false;
boolean inter_left = false;

//pointer of serial buffer
int string_head = 0;
int string_end = 0;
unsigned char * comdata;

//run duration of last running
unsigned long motor_time;
unsigned long motor_time_head;
unsigned long motor_time_end;

//register for time calculation
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

void stop_motor()
{
    digitalWrite(PIN_PULS,LOW);
    digitalWrite(PIN_ENABLE,HIGH);
    Serial.println("stop_motor:");
}

//function for interrupt
void blink()
{
	
	int val_right = digitalRead(PIN_INTER_RIGHT);
	int val_left = digitalRead(PIN_INTER_LEFT);
	
	if(val_right)
	{
		stop_motor();
		Serial.println("interrupt right");
		inter_right = true;
	} else {
		inter_right = false;
	}

	if(val_left)
	{
		stop_motor();
		Serial.println("interrupt left");
		inter_left = true;
	} else {
		inter_left = false;
	}

	return;
}

void setup() {
	// put your setup code here, to run once:
	//---===引脚模式设置===---// 
	pinMode(PIN_PULS,OUTPUT);
	pinMode(PIN_DIR,OUTPUT);
	pinMode(PIN_ENABLE,OUTPUT);

	pinMode(PIN_INTER_RIGHT,OUTPUT);
	pinMode(PIN_INTER_LEFT,OUTPUT);

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
	value_high=EEPROM.read(ADDRESS_HIGH);
	value_low=EEPROM.read(ADDRESS_LOW);

	if(value_high==255)
	{
		//初始化各地址初始值
		EEPROM.write(ADDRESS_HIGH, 0);
		EEPROM.write(ADDRESS_LOW, 0);
		curpos_coor=0;
	}
	else
	{
		curpos_coor=value_high*256+value_low;
	}

	//---===使能端有效===---//
	digitalWrite(PIN_ENABLE,LOW);

	//---===限位开关置低===---//
	digitalWrite(PIN_INTER_RIGHT,LOW);
	digitalWrite(PIN_INTER_LEFT,LOW);

	//中断开启
	attachInterrupt(PIN_INTER, blink, CHANGE);

	Serial.begin(9600);	      //打开串口

	inter_left = false;
	inter_right = false;
}

int g_inputCount = 0;

void printData(unsigned char *comdata)
{
	int i = 0;
	Serial.print("\r\n");

	for(; i < 20; i++) {
		Serial.print(comdata[string_end + i], HEX);
		Serial.print(" ");
	}
	Serial.print("\r\n");

}
void parseData(unsigned char *comdata)
{
	bool found_head = false;

	printData(comdata);

	//while( (string_head + 1) % MAX_QUEUE != string_end )
	while( string_head != string_end )
	{
		if(comdata[string_end] == 0xee && comdata[string_end + 1] == 0xee){
			found_head = true; 
			break;
		} else {
			string_end ++;
			string_end = string_end % MAX_QUEUE;
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

int speedUp(int countStep){
      int i = 0;
      float tdelay = 350;
      for(i = 0; i < countStep; i++)
      {
	    Serial.print("tdelay:");
	    Serial.println(tdelay);

	    for(int j = 0; j < 100; j++){
		    digitalWrite(PIN_PULS, LOW);
		    delayMicroseconds(int(tdelay));
		    digitalWrite(PIN_PULS, HIGH);
		    delayMicroseconds(int(tdelay));
	    }
	    tdelay = tdelay - 6.25;
      }
      return tdelay;
}
/*
 * 电机驱动;
 * int & n:需要的脉冲数; 
 * int	tf:频率;
*/
unsigned long do_run(unsigned long steps, unsigned long during_micro_second, bool direct_left_default)
{
	unsigned long i = 0;

	digitalWrite(PIN_DIR, direct_left_default);

	for(i = 0; i < steps && (!inter_left && !inter_right); i++)
	{
		if (inter_left || inter_right)
			break;
			
		digitalWrite(PIN_PULS, LOW);
		delayMicroseconds(during_micro_second);
		digitalWrite(PIN_PULS, HIGH);
		delayMicroseconds(during_micro_second);
	}

	return i;
}

bool dir_left_or_right(long obj_distance) {
	if(obj_distance > 0) // right
		return false;
	else
		return true; // left
}

/*
 * 运动规划；
 * int distance_x:x方向所需运动位移；
 * int distance_y:y方向所需运动位移；
 * int distance_z:z方向所需运动位移；
*/
void motor_run(long distance)
{
	unsigned long pulse_total = int(abs(distance) * (PULSE_RATE / (TRAN_RATION * 1.0)));
	//Enable motor, LOW means enabling.
	digitalWrite(PIN_ENABLE,LOW);
     
	//保存此过程所需脉冲数
	//motor_time_end=millis();			    //开始记录时间
     
	do_run(pulse_total, 200, dir_left_or_right(distance));
     
/*
      //将当前位置保存至EEPROM中
      value_time_0 = motor_time%256;
      value_time_1 = (motor_time/256)%256;
      value_time_2 = (motor_time/256/256)%256;
      value_time_3 = (motor_time/256/256/256)%256;
      EEPROM.write(address_time_0, value_time_0);
      EEPROM.write(address_time_1, value_time_1);
      EEPROM.write(address_time_2, value_time_2);
      EEPROM.write(address_time_3, value_time_3);

     //将当前位置保存至EEPROM中
     value_high = curpos_coor/256;
     value_low	= curpos_coor%256;

     EEPROM.write(ADDRESS_HIGH, value_high);
     EEPROM.write(ADDRESS_LOW, value_low);
*/
}

void processMotor()
{
	int objpos_x = 0, objpos_y = 0, objpos_z = 0;		//目标位置坐标
	int curpos_x = 0, curpos_y = 0, curpos_z = 0;	       //当前目标坐标
	int distance_x = 0, distance_y = 0, distance_z = 0;   //所需移动位移
	
	inter_left = inter_right = false;
	switch(cmd.data.mode)
	{
		case 'A':			//move:输入的x,y,z是绝对坐标
			objpos_x=para[0].d;
			objpos_y=para[1].d;
			objpos_y=para[2].d;
			distance_x=objpos_x-curpos_x;		      //所需运行位移
			distance_y=objpos_y-curpos_y;		     //所需运行位移
			distance_z=objpos_z-curpos_z;		    //所需运行位移

			motor_run(distance_y);	//电机驱动

			Serial.println("Here A !!!!!");
			break;
		case 'B':		      //move:输入的x,y,z是相对坐标
			Serial.println("Here B moving!!!");

			distance_x=para[0].d;
			distance_y=para[1].d;
			distance_z=para[2].d;
			objpos_x=distance_x+curpos_x;		      //所需运行位移
			objpos_y=distance_y+curpos_y;		     //所需运行位移
			objpos_z=distance_z+curpos_z;		    //所需运行位移

			motor_run(distance_y);	//电机驱动

			Serial.println("Finished B moving!!!");
			break;
		case 'C':		      //stop:电机停止
			void stop_motor(); 
			break;
		case 'D':		      //getTimedata:获得上次运行的时间
			value_time_0=EEPROM.read(address_time_0);
			time_data[0] = char(value_time_0);
			value_time_1=EEPROM.read(address_time_1);
			time_data[1] = char(value_time_1);
			value_time_2=EEPROM.read(address_time_2);
			time_data[2] = char(value_time_2);
			value_time_3=EEPROM.read(address_time_3);
			time_data[3] = char(value_time_3);
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

		if(comdata[string_head] == 0xdd &&
			comdata[string_head-1] == 0xdd &&
			comdata[string_head-2] == 0xdd &&
			comdata[string_head-3] == 0xdd )

			g_processFlag = true;

		string_head++;
		string_head = (string_head) % MAX_QUEUE;

	}
}
void processData()
{
	//提取20位有用信号字符,并获得起点位置
	while (string_head != string_end && g_processFlag)
	{
		parseData(comdata);

		processMotor();
	}

	g_processFlag = false;

}
void loop() {

	// put your main code here, to run repeatedly:
	GetCOM_Data();
	processData();

	digitalWrite(PIN_ENABLE,LOW);

/*
	motor_run(600);
	delay(1000);
	motor_run(-600);
*/
}
