#include <EEPROM.h>			
//0:x axis, 1:y axis, 2:z axis, 3:w cloud platform
#define AXIS 3

#define MAX_QUEUE	512
#define CMD_SIZE	(6 * 4)
#define CMD_DATA_LEN	CMD_SIZE * 40

#define PIN_PULS 5
#define PIN_DIR 6
#define PIN_ENABLE 7

#define PIN_INTER_RIGHT 1  //Equal to pin 3; 0 means pin 2 on the board
#define PIN_INTER_LEFT 0


#define PULSE_RATE 400

#define BACKSPACE_STEPS 1200
#define ANTI_SHAKE_STEPS 10

#if AXIS == 3		//For clond-platform
	#define TRAN_RATION 45			//360/(2000/400)
	#define PULSE_DELAY_DEFAULT 1500		//steps is 2000/r
#elif AXIS == 1
	#define TRAN_RATION 75
	#define PULSE_DELAY_DEFAULT 500
#else
	#define TRAN_RATION 95
	#define PULSE_DELAY_DEFAULT 400
#endif

#define PULSE_DELAY_MAX 500
#define PULSE_DELAY_MIN 50

#define ADDRESS_LOW 2
#define ADDRESS_HIGH 1

#define MASK_POS_LOW 0xFF
#define MASK_POS_HIGH 0xFF00
#define REG_POS_LOW 0
#define REG_POS_HIGH 1

#define REG_LEN_LOW 2
#define REG_LEN_HIGH 3

#define GET_POS_LOW(pos)	pos & MASK_POS_LOW
#define GET_POS_HIGH(pos)	(pos & MASK_POS_HIGH) >> 8

void linterrupt();
void rinterrupt();
void updatePos(long pos);
long getCurPos();
void motor_run(long distance);
void printLog(const char str[]);

int g_inputCount = 0;
bool g_processFlag = false;

bool g_afterSetup = false;

int g_pulseDelay = PULSE_DELAY_DEFAULT;
bool g_needUpdatePos = true;

long g_position = 0;

bool inter_right = false;
bool inter_left = false;
bool g_reachend = false;

bool g_inInterruptLeft = false;
bool g_inInterruptRight = false;

//pointer of serial buffer
int string_head = 0;
int string_end = 0;

long g_targetPos = 0;
unsigned char * comdata;

//run duration of last running
unsigned long motor_time;
unsigned long motor_time_head;
unsigned long motor_time_end;
void stop_motor();

union PARA
{
	unsigned char c[4];  
	long int d;
};
union PARA para[5];

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

long getCurPos() {
	return g_position;
}
long getCurPosReg() {
	return EEPROM.read(REG_POS_LOW) + (EEPROM.read(REG_POS_HIGH) << 8);
}
unsigned long do_run(unsigned long steps, unsigned long during_micro_second, bool direct_right_default)
{
	unsigned long i = 0;

	digitalWrite(PIN_DIR, !direct_right_default);

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
	return (direct_right_default ? i : -i);
}

unsigned long do_run_interrupt(unsigned long steps, unsigned long during_micro_second, bool direct_right_default)
{
	unsigned long i = 0;

	digitalWrite(PIN_DIR, !direct_right_default);

	for(i = 0; i < steps; i++)
	{
		digitalWrite(PIN_PULS, LOW);
		delayMicroseconds(during_micro_second*10);
		digitalWrite(PIN_PULS, HIGH);
		delayMicroseconds(during_micro_second*10);
	}
	return (direct_right_default ? i : -i);
}

bool dir_left_or_right(long obj_distance) {
	if(obj_distance > 0) // right
		return true;
	else
		return false; // left
}
long m_to_pulse(long distance) {
	return long(round(distance * (PULSE_RATE / (TRAN_RATION * 1.0))));
}

long pulse_to_m(long pulse) {
	return long(round(((pulse * 1.0) / (PULSE_RATE * 1.0)) * TRAN_RATION));
}

void printCurPos() {
	Serial.print("addtional:");
	Serial.print(g_targetPos);
	Serial.print(" cur pos:");
	Serial.print(g_position);
	Serial.print(" (");
	Serial.print(EEPROM.read(REG_POS_HIGH));
	Serial.print(" : ");
	Serial.print(EEPROM.read(REG_POS_LOW));
	Serial.print(" )");
}

void motor_run(long distance)
{
	if(AXIS == 3) {
		if(distance >= 180)
			distance =180;
		if(distance <= -180)
			distance = -180;
	}
	unsigned long pulse_total = long(abs(distance) * (long(PULSE_RATE) / (TRAN_RATION * 1.0)));

	unsigned long pulse_actual = 0;
	
	//Enable motor, LOW means enabling.
	digitalWrite(PIN_ENABLE,LOW);

	g_needUpdatePos = true; //by defalut need update position after moving expect for interrupt triggered!
	g_reachend = false;
     
	pulse_actual = do_run(pulse_total, g_pulseDelay, dir_left_or_right(distance));

	if(g_needUpdatePos) {
		updatePos( getCurPosReg() + pulse_actual);
	}

	printCurPos();

	if(pulse_actual == 0 && !(inter_left || inter_right)) {
		printLog("Motro moving check ERROR, run again!");
		motor_run(distance);
	}
}

void motor_run_interrupt(long distance)
{
	unsigned long pulse_total = long(abs(distance) * (long(PULSE_RATE) / (TRAN_RATION * 1.0)));

	unsigned long pulse_actual = 0;

	//Enable motor, LOW means enabling.
	digitalWrite(PIN_ENABLE,LOW);

	pulse_actual = do_run_interrupt(pulse_total, g_pulseDelay, dir_left_or_right(distance));

	if(g_needUpdatePos) {
		updatePos( getCurPosReg() + pulse_actual);
	}
}
void stop_motor()
{
	digitalWrite(PIN_PULS,LOW);
}

//function for interrupt
void linterrupt()
{
	noInterrupts();
	if(!g_afterSetup || g_inInterruptLeft) return;

	g_inInterruptLeft = true;
	
	g_position = 0;
	EEPROM.write(REG_POS_LOW, 0);
	EEPROM.write(REG_POS_HIGH, 0);
	g_needUpdatePos = false;

	inter_left = true;

	motor_run_interrupt(5);

	g_inInterruptLeft = false;
	interrupts();
}
void rinterrupt()
{
	noInterrupts();
	if(!g_afterSetup || g_inInterruptRight) return;

	g_inInterruptRight = true;

	g_reachend = true;

	inter_right = true;

	motor_run_interrupt(-5);

	g_inInterruptRight = false;
	interrupts();
}
void updatePos(long pos) {
	g_position = pos;

	EEPROM.write(REG_POS_LOW, GET_POS_LOW(pos));
	EEPROM.write(REG_POS_HIGH, GET_POS_HIGH(pos));
}

void reset() {
	if(AXIS == 2) {
		motor_run(-50);
		delay(100);
	}
	if(AXIS != 3)
		motor_run(-1900);
}

void setup() {
	// put your setup code here, to run once:

	//---===引脚模式设置===---// 
	pinMode(PIN_PULS, OUTPUT);
	pinMode(PIN_DIR, OUTPUT);
	pinMode(PIN_ENABLE, OUTPUT);

	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	digitalWrite(2,HIGH);
	digitalWrite(3,HIGH);

	//---===分配空间===---//
	comdata = (unsigned char *) malloc(CMD_DATA_LEN * sizeof(char));

	memset(comdata, 0, 800);

	//---===使能端有效===---//
	digitalWrite(PIN_ENABLE,LOW);

	//中断开启
	attachInterrupt(PIN_INTER_LEFT, linterrupt, LOW);
	attachInterrupt(PIN_INTER_RIGHT, rinterrupt, LOW);

	//Have to put serial initial after interrupt initializaion.
	Serial.begin(9600);	      //打开串口

	inter_left = inter_right = false;
	g_afterSetup = true;
}
void printLog(const char str[])
{
	Serial.print("\r\n[LOG]");
	Serial.println(str);
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
		if(comdata[string_end] == 0xee && comdata[(string_end + 1) % MAX_QUEUE] == 0xee
				&& comdata[(string_end + 20) % MAX_QUEUE] == 0xdd
				&& comdata[(string_end + 21) % MAX_QUEUE] == 0xdd
				&& comdata[(string_end + 22) % MAX_QUEUE] == 0xdd
				&& comdata[(string_end + 23) % MAX_QUEUE] == 0xdd){
			found_head = true; 
			break;
		} else {
			string_end ++;
			string_end = string_end % MAX_QUEUE;
		}
	}

	if(found_head) {
		int x = 0;
		for(x = 0; x < 4; x++){
			cmd.c[x] = comdata[string_end];
			comdata[string_end] = 0;
			string_end = (string_end + 1) % MAX_QUEUE;
		}

		for(x = 0; x < 5; x++)
		{
			int y = 0;
			for(y = 0; y < 4; y++){
				para[x].c[y] = comdata[string_end];
				comdata[string_end] = 0;
				string_end = (string_end + 1) % MAX_QUEUE;
			}
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
			Serial.print("Here A ! para.d: pulse:");

			g_targetPos = para[AXIS].d;

			distance = para[AXIS].d - pulse_to_m(getCurPosReg());
			Serial.print(distance);

			motor_run(distance);	//电机驱动

			Serial.println("Finished A moving!!!");
			break;
		case 'B':		      //move:输入的x,y,z是相对坐标
			Serial.println("Here B moving!!!");

			g_targetPos = para[AXIS].d + pulse_to_m(getCurPosReg());

			distance = para[AXIS].d;
			motor_run(distance);	//电机驱动

			Serial.println("Finished B moving!!!");
			break;
		case 'C':
			Serial.print("copy ");
			Serial.print(para[0].d);
			Serial.print(":");
			Serial.print(para[1].d);

			break;
		case 'V':
			if (AXIS == 1) {
				Serial.println("Here V!");
				g_pulseDelay = para[0].d;
			}
			
			break;

		case 'v':
			if (AXIS == 1) {
				g_pulseDelay += para[0].d;
				if(g_pulseDelay > PULSE_DELAY_MAX) {
					g_pulseDelay = PULSE_DELAY_MAX;
				}
				else if (g_pulseDelay < PULSE_DELAY_MIN)
					g_pulseDelay = PULSE_DELAY_MIN;

				Serial.print("Here v! g_pulseDelay:");
				Serial.println(g_pulseDelay);
			}
			break;
		case 'R':
			g_targetPos = 0;
			g_pulseDelay = PULSE_DELAY_MAX;
			reset();

			break;

	}
}

//get COM data
void GetCOM_Data()
{
	//receive Serial port data
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
			comdata[(string_head-1 + MAX_QUEUE) % MAX_QUEUE] == 0xdd &&
			comdata[(string_head-2 + MAX_QUEUE) % MAX_QUEUE] == 0xdd &&
			comdata[(string_head-3 + MAX_QUEUE) % MAX_QUEUE] == 0xdd )

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
	GetCOM_Data();
	processData();
	delay(10);
}
