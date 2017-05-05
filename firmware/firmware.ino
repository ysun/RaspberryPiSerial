#include <EEPROM.h>			
//0:x axis, 1:y axis, 2:z axis, 3:w cloud platform
#define AXIS 3

#define MAX_QUEUE	512
#define CMD_SIZE	(6 * 4)
#define CMD_DATA_LEN	CMD_SIZE * 40

//add
#define PIN_PULS 5
#define PIN_DIR 6
#define PIN_ENABLE 7

#define PIN_INTER_RIGHT 1  //Equal to pin 3; 0 means pin 2 on the board
#define PIN_INTER_LEFT 0


#define PULSE_RATE 400

#define BACKSPACE_STEPS 1200
#define ANTI_SHAKE_STEPS 10

#if AXIS == 3		//For clond-platform
	#define TRAN_RATION 22.5			//360/(6400/400)
	#define PULSE_DELAY_DEFAULT 1000		//steps is 4000/r
#elif AXIS == 1
	#define TRAN_RATION 75
	#define PULSE_DELAY_DEFAULT 300
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

#define INTER_DELAY_RATION 3.0

#define GET_POS_LOW(pos)	pos & MASK_POS_LOW
#define GET_POS_HIGH(pos)	(pos & MASK_POS_HIGH) >> 8

void linterrupt();
void rinterrupt();
void updatePos(long pos);
long getCurPos();
void motor_run(long distance);
void printLog(const char str[]);
long pulse_to_m(long pulse);
unsigned long speedAdjust(unsigned long pulsDelay, unsigned int steps, bool DIRECT);

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

	i = speedAdjust(during_micro_second, steps, true);

	unsigned long amendsteps = steps - i;

	for(; i < amendsteps; i++)
	{
		if (inter_left || inter_right)
			break;
			
		digitalWrite(PIN_PULS, LOW);
		delayMicroseconds(during_micro_second);
		digitalWrite(PIN_PULS, HIGH);
		delayMicroseconds(during_micro_second);
	}

	i =i + speedAdjust(during_micro_second, steps, false);

	return (direct_right_default ? i : -i);
}

unsigned long do_run_interrupt(unsigned long steps, unsigned long during_micro_second, bool direct_right_default)
{
	unsigned long i = 0;

	digitalWrite(PIN_DIR, !direct_right_default);

	for(i = 0; i < steps; i++)
	{
		digitalWrite(PIN_PULS, LOW);
		delayMicroseconds(during_micro_second * INTER_DELAY_RATION);
		digitalWrite(PIN_PULS, HIGH);
		delayMicroseconds(during_micro_second * INTER_DELAY_RATION);
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
	Serial.print(pulse_to_m(g_targetPos));
	Serial.print("(");
	Serial.print(g_targetPos);
	Serial.print(")     ");

	Serial.print("cur pos:");
	Serial.print(pulse_to_m(g_position));
	Serial.print("(");
	Serial.print(g_position);
	Serial.print(")");
}
unsigned long do_run_risk(unsigned long steps, unsigned long during_micro_second, bool direct_right_default)
{
	unsigned long i = 0;

	digitalWrite(PIN_DIR, !direct_right_default);

	for(i = 0; i < steps; i++)
	{
		if (inter_left || inter_right)
			break;

		digitalWrite(PIN_PULS, LOW);
		delayMicroseconds(during_micro_second * INTER_DELAY_RATION);
		digitalWrite(PIN_PULS, HIGH);
		delayMicroseconds(during_micro_second * INTER_DELAY_RATION);
	}
	return (direct_right_default ? i : -i);
}


void motor_run(long distance)
{
	if(AXIS == 3) {
		if(pulse_to_m(g_targetPos) >= 180) {
			distance =180 - pulse_to_m(g_position);
			Serial.println("left overflow!!!");
		}
		if(pulse_to_m(g_targetPos) <= -180) {
			distance = -180 - pulse_to_m(g_position);
			Serial.println("right overflow!!!");
		}
	}
	unsigned long pulse_total = abs(m_to_pulse(distance));

	unsigned long pulse_actual = 0;

	if(pulse_total == 0) return;
	
	//Enable motor, LOW means enabling.
	digitalWrite(PIN_ENABLE,LOW);

	g_needUpdatePos = true; //by defalut need update position after moving expect for interrupt triggered!
	g_reachend = false;

	Serial.print("pulse_total: ");
	Serial.print(pulse_total);
	Serial.print("  distance: ");
	Serial.println(distance);


	//add movement area
	long targetCoor = pulse_to_m(g_targetPos);
	unsigned long extrem_total = 0;
	unsigned long normalPulse = pulse_total;
	if((AXIS != 3) && (targetCoor > 980 || targetCoor < 0)) {
		targetCoor = (targetCoor > 950) ? (targetCoor - 900) :(80 - targetCoor);
		extrem_total = m_to_pulse(targetCoor);
		normalPulse = pulse_total - extrem_total;
	}

	pulse_actual = do_run(normalPulse, g_pulseDelay, dir_left_or_right(distance));
	pulse_actual = pulse_actual + do_run_risk(extrem_total,  g_pulseDelay, dir_left_or_right(distance));
	//add movement area

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
	g_targetPos = 0;

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
	if(AXIS != 3) {
		Serial.println("Here R moving!!!");
		int distance = -1100;
		g_targetPos = m_to_pulse(distance + pulse_to_m(getCurPosReg()));
		Serial.print("g_targetPos");
		Serial.println(pulse_to_m(g_targetPos));

		motor_run(distance);	//电机驱动

		Serial.println("Finished R moving!!!");
	}
	else {
		g_position = 0;
		g_targetPos = 0;

		EEPROM.write(REG_POS_LOW, 0);
		EEPROM.write(REG_POS_HIGH, 0);
		g_needUpdatePos = false;
	}
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
unsigned long speedAdjust(unsigned long pulsDelay, unsigned int steps, bool DIRECT) {
	int i = 0, j = 0, i_time = 0, j_time = 5;
	float minRatio = 1.0;
	float disRatio = 0.0;
	float maxRatio = INTER_DELAY_RATION;
	unsigned long tf = pulsDelay;
	unsigned long count = 0;

	unsigned int delaySteps = (steps > 200) ? 100 : int(0.5 * steps);
	i_time = delaySteps / j_time;
	disRatio = (DIRECT ? (minRatio - maxRatio):(maxRatio - minRatio)) / i_time;

	float ratio =DIRECT ? maxRatio:minRatio;

	for(i = 0; i < i_time; i++) {
		ratio = ratio + disRatio;
		tf = long(pulsDelay * ratio);

		for(j = 0; j < j_time; j++) {
			if (inter_left || inter_right)
				return count;

			digitalWrite(PIN_PULS, LOW);
			delayMicroseconds(tf);
			digitalWrite(PIN_PULS, HIGH);
			delayMicroseconds(tf);
			count++;
		}
	}

	return count;
}
void processMotor()
{
	long distance = 0;
	switch(cmd.data.mode)
	{
		case 'A':			//move:输入的x,y,z是绝对坐标
			Serial.print("Here A ! para.d: pulse:");

			g_targetPos = m_to_pulse(para[AXIS].d);

			distance = para[AXIS].d - pulse_to_m(getCurPosReg());

			motor_run(distance);	//电机驱动

			Serial.println("Finished A moving!!!");
			break;
		case 'B':		      //move:输入的x,y,z是相对坐标
			Serial.println("Here B moving!!!");

			g_targetPos = m_to_pulse(para[AXIS].d + pulse_to_m(getCurPosReg()));
			Serial.print("g_targetPos");
			Serial.println(pulse_to_m(g_targetPos));

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
