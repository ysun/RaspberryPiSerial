#include <EEPROM.h>			
#define AXIS 1 //0:x, 1:y, 2:z

#define MAX_QUEUE	512
#define CMD_SIZE	(5 * 4)
#define CMD_DATA_LEN	CMD_SIZE * 40

#define PIN_PULS 5
#define PIN_DIR 6
#define PIN_ENABLE 8

#define PIN_INTER_RIGHT 1  //Equal to pin 3; 0 means pin 2 on the board
#define PIN_INTER_LEFT 0

#define TRAN_RATION 75

#define PULSE_RATE 400
#define PULSE_DELAY_DEFAULT 100

#define ADDRESS_LOW 2
#define ADDRESS_HIGH 1

int g_inputCount = 0;
bool g_processFlag = false;

bool g_afterSetup = false;

int g_pulseDelay = PULSE_DELAY_DEFAULT;

bool inter_right = false;
bool inter_left = false;

//pointer of serial buffer
int string_head = 0;
int string_end = 0;

unsigned char * comdata;

//run duration of last running
unsigned long motor_time;
unsigned long motor_time_head;
unsigned long motor_time_end;

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
} cmd;
unsigned long do_run(unsigned long steps, unsigned long during_micro_second, bool direct_left_default)
{
	unsigned long i = 0;

	digitalWrite(PIN_DIR, direct_left_default);

	inter_left = inter_right = false; //just for interrupt!

	for(i = 0; i < steps; i++)
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

void motor_run(long distance)
{
	unsigned long pulse_total = int(abs(distance) * (PULSE_RATE / (TRAN_RATION * 1.0)));
	//Enable motor, LOW means enabling.
	digitalWrite(PIN_ENABLE,LOW);
     
	//保存此过程所需脉冲数
	//motor_time_end=millis();			    //开始记录时间
     
	do_run(pulse_total, g_pulseDelay, dir_left_or_right(distance));
     
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


void stop_motor()
{
    digitalWrite(PIN_PULS,LOW);
    digitalWrite(PIN_ENABLE,HIGH);
}

//function for interrupt
void linterrupt()
{
	if(!g_afterSetup) return;
	
//	motor_run(100);
	stop_motor();

	motor_run(5);

	inter_left = true;
	return;
}
void rinterrupt()
{
	if(!g_afterSetup) return;
	
//	motor_run(-100);
	stop_motor();
	
	motor_run(-5);

	inter_right = true;

	return;
}


void setup() {
	// put your setup code here, to run once:

	//---===引脚模式设置===---// 
	pinMode(PIN_PULS, OUTPUT);
	pinMode(PIN_DIR, OUTPUT);
	pinMode(PIN_ENABLE, OUTPUT);

	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);

	delay(100);

	digitalWrite(2,LOW);
	digitalWrite(3,LOW);

	//---===分配空间===---//
	comdata = (unsigned char *) malloc(CMD_DATA_LEN * sizeof(char));

	memset(comdata, 0, 800);

	//---===使能端有效===---//
	digitalWrite(PIN_ENABLE,LOW);

	//中断开启
	attachInterrupt(PIN_INTER_LEFT, linterrupt, RISING);
	attachInterrupt(PIN_INTER_RIGHT, rinterrupt, RISING);

	Serial.begin(9600);	      //打开串口

	inter_left = inter_right = false;
	g_afterSetup = true;
}


void printData(unsigned char *comdata)
{
	int i = 0;
	Serial.print("\r\n");

	for(; i < CMD_SIZE; i++) {
		Serial.print(comdata[string_end + i], HEX);
		Serial.print(" ");
	}
	Serial.print("\r\n");

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

int speedUp(int countStep){   //chaning speed every 100 steps!
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

void processMotor()
{
	long distance = 0;

	switch(cmd.data.mode)
	{
		case 'A':			//move:输入的x,y,z是绝对坐标
			Serial.println("Here A !!!!!");
			distance = para[AXIS].d;
/*
			distance_y=objpos_x-curpos_x;		     //所需运行位移
			distance_y=objpos_y-curpos_y;		     //所需运行位移
			distance_z=objpos_z-curpos_z;		    //所需运行位移
*/
			motor_run(distance);	//电机驱动

			break;
		case 'B':		      //move:输入的x,y,z是相对坐标
			Serial.println("Here B moving!!!");

			distance = para[AXIS].d;
			motor_run(distance);	//电机驱动

			Serial.println("Finished B moving!!!");
			break;
		case 'V':
			Serial.println("Here V!");
			g_pulseDelay = para[0].d;
			
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

	delay(10);
}
