#include <EEPROM.h>                     

#define MAX_QUEUE	512
#define CMD_SIZE	(5 * 4)

#define PIN_PULS 5
#define PIN_DIR 6
#define PIN_ENABLE 8

#define PIN_INTER 3          
#define PIN_INTER_RIGHT 10 
#define PIN_INTER_LEFT 9

#define TRAN_RATION_X 95
#define TRAN_RATION_Y 75
#define TRAN_RATION_Z 95
#define DELAY_TF_X 200
#define DELAY_TF_Y 800
#define DELAY_TF_Z 200

#define PULSE_RATE_X 800
#define PULSE_RATE_Y 400
#define PULSE_RATE_Z 800

#define ADDRESS_LOW 2
#define ADDRESS_HIGH 1

#define ACC_TIME 160000            //acclebrate 10ms
#define MAX_DELAY 2000

#define SORT 'y'
//EEPROM寄存器保存的高低位数
int value_high;
int value_low;
int value_time_0;
int value_time_1;
int value_time_2;
int value_time_3;

//电机参数
int objpos_coor;           //target pos
int curpos_coor;          //curret pos
int distance_coor;       //distance of moving


boolean inter_stop = false;

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
     stop_motor();
     
     inter_stop = true;
//     int val_right = digitalRead(PIN_INTER_RIGHT);
//     int val_left = digitalRead(PIN_INTER_LEFT);
//     if(val_right)
//     {
//         digitalWrite(PIN_DIR, HIGH);
//         return;
//     }
//     if(val_left)
//     {
//         digitalWrite(PIN_DIR, LOW);
//         return;
//     }
     Serial.println(" blink:");
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
	attachInterrupt(1,blink,RISING);

	Serial.begin(9600);           //打开串口
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

	while( (string_head + 1) % MAX_QUEUE != string_end )
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
/*
 * 电机加速启动
 * unsigned int & n;电机所需脉冲数
 * int tf;电机所需延迟时间
*/
void Accelerate_Launch(unsigned int & n, int tf)
{
      int d;
      int acc_n;
      acc_n=ACC_TIME * 2 / (tf + MAX_DELAY);
      d = (MAX_DELAY - tf) / (acc_n - 1);
      for(int i = MAX_DELAY; i > tf;)
      {
            digitalWrite(PIN_PULS, LOW);
            delayMicroseconds(i);
            digitalWrite(PIN_PULS, HIGH);
            delayMicroseconds(i);
            i = i - d;
            n--;
      }     
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
 * int  tf:频率;
*/
void motor(unsigned int  & n, int tf)
{
    //电机加速启动
    //Accelerate_Launch(n, tf);
    //电机转动
    for(; n > 0 && inter_stop == false; n--)
    {
          digitalWrite(PIN_PULS, LOW);
          delayMicroseconds(tf);
          digitalWrite(PIN_PULS, HIGH);
          delayMicroseconds(tf);
    }
}

/*
 * 运动规划；
 * int distance_x:x方向所需运动位移；
 * int distance_y:y方向所需运动位移；
 * int distance_z:z方向所需运动位移；
*/
void dmotor(int distance_x, int distance_y, int distance_z)
{
     //定义基本变量
     int delay_tf;
     int tran_ration;
     int pulse_rate;
     int distance_abs;  
     char sort;

     
     sort = SORT;
     //根据类型确定基本变量
     switch(sort)
     {
          case 'x':
               distance_coor = distance_x;
               delay_tf = DELAY_TF_X;
               tran_ration = TRAN_RATION_X;
               pulse_rate = PULSE_RATE_X;
               break;
          case 'y':
	       Serial.println("I'm Y AXIS");
               distance_coor = distance_y;
               delay_tf = DELAY_TF_Y;
               tran_ration = TRAN_RATION_Y;
               pulse_rate = PULSE_RATE_Y;
               break;
          case 'z':
               distance_coor = distance_z;
               delay_tf = DELAY_TF_Z;
               tran_ration = TRAN_RATION_Z;
               pulse_rate = PULSE_RATE_Z;
               break;
          default:
           Serial.println("ERROR:motor sort error！！！");
           stop_motor();
           break;
     }

     //使能端开启有效
     digitalWrite(PIN_ENABLE,LOW);
     
      //判断向电机转动方向
     if(distance_coor < 0)                           //x方向电机反转 同方向左转
     {
         int val = digitalRead(PIN_INTER_RIGHT);
         if(val)
         {
             stop_motor();
             return;
         }
         digitalWrite(PIN_DIR,LOW);
         distance_abs = abs(distance_coor);
     }
     else
     {
         int val = digitalRead(PIN_INTER_LEFT);
         if(val)
         {
             stop_motor();
             return;
         }
         digitalWrite(PIN_DIR,HIGH);
         distance_abs = abs(distance_coor);
     }    
    

     //将距离转换为脉冲数
     unsigned int puls_add = int(distance_abs * ( pulse_rate / (tran_ration * 1.0)));

     
     Serial.println(puls_add, DEC);
     
     //保存此过程所需脉冲数
     unsigned int keep_add = puls_add;

     motor_time_end=millis();                       //开始记录时间
     
     Serial.print("puls_add:");
     Serial.println(puls_add, DEC);
     Serial.print("delay_tf:");
     Serial.println(delay_tf, DEC);

     motor(puls_add, delay_tf); 
     
     motor_time_head=millis();                    //记录运行结束时间
     motor_time=motor_time_head-motor_time_end;  //获得运行时间

     //求出当前位置
     curpos_coor = curpos_coor + distance_coor * (1 - puls_add / keep_add);
     
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
     value_low  = curpos_coor%256;

     EEPROM.write(ADDRESS_HIGH, value_high);
     EEPROM.write(ADDRESS_LOW, value_low);
}

void processMotor()
{
	int objpos_x = 0, objpos_y = 0, objpos_z = 0;           //目标位置坐标
	int curpos_x = 0, curpos_y = 0, curpos_z = 0;          //当前目标坐标
	int distance_x = 0, distance_y = 0, distance_z = 0;   //所需移动位移
        
        inter_stop = false;
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

			Serial.println("Here A !!!!!");
			break;
		case 'B':                     //move:输入的x,y,z是相对坐标
			Serial.println("Here B moving!!!");

			distance_x=para[0].d;
			distance_y=para[1].d;
			distance_z=para[2].d;
			objpos_x=distance_x+curpos_x;                 //所需运行位移
			objpos_y=distance_y+curpos_y;                //所需运行位移
			objpos_z=distance_z+curpos_z;               //所需运行位移

			dmotor(distance_x,distance_y,distance_z);       //电机驱动

			Serial.println("Finished B moving!!!");
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
	//提取20位有用信号字符,并获得起点位置

	while (string_head != string_end && g_processFlag)
	{
		parseData(comdata);

		processMotor();

//		string_end = (string_end + CMD_SIZE)&MAX_QUEUE;
	}

	g_processFlag = false;
}
void loop() {

	// put your main code here, to run repeatedly:
//	GetCOM_Data();
        digitalWrite(PIN_ENABLE,LOW);
        digitalWrite(PIN_DIR,LOW);
	int endduring = speedUp(16);

	Serial.print("endduring:");
	Serial.println(endduring, DEC);

      for(int i = 0; i < 1600; i++)
      {
            digitalWrite(PIN_PULS, LOW);
            delayMicroseconds(endduring);
            digitalWrite(PIN_PULS, HIGH);
            delayMicroseconds(endduring);
      }

        digitalWrite(PIN_DIR,HIGH);
	endduring = speedUp(16);

	Serial.print("endduring:");
	Serial.println(endduring, DEC);

      for(int i = 0; i < 1600; i++)
      {
            digitalWrite(PIN_PULS, LOW);
            delayMicroseconds(endduring);
            digitalWrite(PIN_PULS, HIGH);
            delayMicroseconds(endduring);
      }


        while(1);

//	dmotor(500, 500, 500);       //电机驱动

	//根据获得的运动模式指令，确定运行方式
}
