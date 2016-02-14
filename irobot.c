#if defined(WIN32) || defined(WIN64)
#define _CRT_SECURE_NO_WARNINGS		
#endif

#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include "/usr/include/flycapture/C/FlyCapture2_C.h"

// Definitions of iRobot Create OpenInterface Command Numbers
// See the Create OpenInterface manual for a complete list
 
//                 Create Command              // Arguments
const char         Start = 128;
const char         Stop = 173;
const char         SafeMode = 131;
const char         FullMode = 132;
const char         Clean = 135;
const char         Drive = 137;                // 4:   [Vel. Hi] [Vel Low] [Rad. Hi] [Rad. Low]
const char         DriveDirect = 145;          // 4:   [Right Hi] [Right Low] [Left Hi] [Left Low]
//const char         Demo = 136;                 // 2:    Run Demo x
const char         Sensors = 142;              // 1:    Sensor Packet ID
//const char         CoverandDock = 143;         // 1:    Return to Charger
const char         SensorStream = 148;         // x+1: [# of packets requested] IDs of requested packets to stream
const char         QueryList = 149;            // x+1: [# of packets requested] IDs of requested packets to stream
const char         StreamPause = 150;          // 1:    0 = stop stream, 1 = start stream
//const char         PlaySong = 141;
//const char         Song = 140;

                /* iRobot Create Sensor Packet IDs */
const char         LeftEncoderCounts = 43;
const char         RightEncoderCounts = 44;
//const char         Distance = 19;
//const char         Angle = 20;

const int          PACKET_SIZE = 8; 			// XG packet size
const int          C_PACKET_SIZE = 9;			// irobot packet size
const float        MM_PER_COUNTER = 0.4446;		// mm travel per counter
const int          COUNTER_PER_RANGLE = 410;    // counters per 90 angle

// -- Variable shared among threads
struct timeval startTime;
int SPEED_LEFT =  100;							// slow speed in case of turning
int SPEED_RIGHT = 100;							// slow speed in case of turning
int SPEED_LEFT_STRAIGHT = 200;					// fast speed in case of straight
int SPEED_RIGHT_STRAIGHT = 200;					// fast speed in case of straight
float pgrElapsedTime;							// WRITE * READ IN REAL-TIME (Use api)
float xgElapsedTime;							// WRITE * READ IN REAL-TIME
float xgAngleData;								// WRITE * READ IN REAL-TIME
float encElapsedTime;							// WRITE * READ IN REAL-TIME
unsigned short encLeftCnt;						// WRITE * READ IN REAL-TIME
unsigned short encRightCnt;						// WRITE * READ IN REAL-TIME
// -- shared variable end


fc2Context setPGR();
int setXG();
int setIRobot();
void showInstruction();
int rcvCommand();

void start(fc2Context context, int fdXG, int fdIrobot);
void quit(int fd, fc2Context contextPGR);		// stop OI
void clean(int fd);
void drive(int fd);
void forward(int fd);
void forwardDistance(int fd, int distance);
void reverse(int fd);
void reverseDistance(int fd, int distance);
void left(int fd);
void leftAngle(int fd, int angle);
void right(int fd);
void rightAngle(int fd, int angle);
void stop(int fd);		// stop driving
void stopLeft(int fd);
void stopRight(int fd);
void zigzag(int fd, int length, int width, int req_num_length);

void *receivePGRCapture(void *v_context);
void stopPGRCapture(fc2Context context);
void *receiveCensorXG(void *fd);
void requestCensorEnc(int fd);
void *receiveCensorEnc(void *fd);
void stopCensorEnc(int fd);

void main()
{
	fc2Context contextPGR;			// Context handler of PGR camera
	int fdXG;						// Serial handler of XG1010
	int fdIrobot;					// Serial handler of irobot

	// set handlers
	contextPGR = setPGR();
	fdXG = setXG();
	fdIrobot = setIRobot();

	// for zigzag argument
	int length;
	int width;
	int numlength;

	showInstruction();

	while(1)
	{
		//listen command
		int cmdRcvd;
		cmdRcvd = rcvCommand();

		switch(cmdRcvd)
		{
			case 1:
				start(contextPGR, fdXG, fdIrobot);
				break;
			case 2:
				clean(fdIrobot);
				break;
			case 3:
				drive(fdIrobot);
				showInstruction();
				break;
			case 4:
				printf("[Q] Please enter length / width / # of length : ");
				scanf("%d %d %d", &length, &width, &numlength);
				zigzag(fdIrobot, length, width, numlength);
				stop(fdIrobot);
				break;
			case 5:
				quit(fdIrobot, contextPGR);
				return;
			default:
				break;
		}
	}
	
	return;
}

fc2Context setPGR()
{
	fc2Error error;
    fc2Context context;
    fc2PGRGuid guid;
    unsigned int numCameras = 0;

    error = fc2CreateContext( &context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2CreateContext: %d\n", error );
        exit(-1);
    }

    error = fc2GetNumOfCameras( context, &numCameras );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2GetNumOfCameras: %d\n", error );
        exit(-1);
    }

    if ( numCameras == 0 )
    {
        // No cameras detected
        printf( "[-] No PGR cameras detected.\n");
        exit(-1);
    }

    // Get the 0th camera
    error = fc2GetCameraFromIndex( context, 0, &guid );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2GetCameraFromIndex: %d\n", error );
        exit(-1);
    }

    error = fc2Connect( context, &guid );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2Connect: %d\n", error );

    	// DestoryContext
	    error = fc2DestroyContext( context );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "Error in fc2DestroyContext: %d\n", error );
	    }

        exit(-1);
    }

	return context;
}

int setXG()
{
	int fd;
	struct termios serialio;
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(fd < 0)
	{
		printf("[-] Check the connection of XG consor.\n");
		perror("/dev/ttyUSB0");
		exit(-1);
	}

	memset( &serialio, 0, sizeof(serialio) );
	serialio.c_cflag = B115200;   // baud - 115200 
	serialio.c_cflag |= CS8;      // data bit - 8bit 
	serialio.c_cflag |= CLOCAL;   // use local comm port 
	serialio.c_cflag |= CREAD;    // read & write
	serialio.c_iflag = 0;         // no parity bit
	serialio.c_oflag = 0;
	serialio.c_lflag = 0;
	serialio.c_cc[VTIME] = 0; 
	serialio.c_cc[VMIN] = 1; 
	tcflush (fd, TCIFLUSH ); 			// flush mode mline
	tcsetattr(fd, TCSANOW, &serialio );   // port attr setting

	return fd;
}

int setIRobot()
{
	int fd;

	struct termios serialio;
	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(fd < 0)
	{
		printf("[-] Check the connection of iRobot.\n");
		perror("/dev/ttyUSB1");
		exit(-1);
	}

	memset( &serialio, 0, sizeof(serialio) );
	serialio.c_cflag = B115200;   // baud - 115200 
	serialio.c_cflag |= CS8;      // data bit - 8bit 
	serialio.c_cflag |= CLOCAL;   // use local comm port 
	serialio.c_cflag |= CREAD;    // read & write
	serialio.c_iflag = 0;         // no parity bit
	serialio.c_oflag = 0;
	serialio.c_lflag = 0;
	serialio.c_cc[VTIME] = 0; 
	serialio.c_cc[VMIN] = 1; 
	tcflush (fd, TCIFLUSH ); 			// flush mode mline
	tcsetattr(fd, TCSANOW, &serialio );   // port attr setting

	return fd;
}

void showInstruction()
{
	printf("===============================\n");
	printf("1. To start, Type 1\n");
	printf("2. To clean, Type 2\n");
	printf("3. To move around, Type 3\n");
	printf("4. To zigzag, Type 4\n");
	printf("5. To quit and clear, Type 5\n\n");
	printf("[Info] Please make sure to start by pressing 1 first.\n");
	printf("[Info] Make sure you quit and clear previous data by re-executing and pressing 5 afterwards\n");
	printf("[Info] Please check connection status before start.\n");
	printf("[Info] ttyUSB0 : Gyro (XG1010) | ttyUSB1 : iRobot Create 2\n");
	printf("===============================\n");
}

int rcvCommand()
{
	int command;
	printf("Enter command : ");
	scanf("%d", &command);
	return command;
}

// Start (Starting robot and make 3 threads)
// Send irobot start and safe mode
// [Thread 1] PGR camera capture
// [Thread 2] XG1010 angle listening
// [Thread 3] Encoder counter listening
void start(fc2Context context, int fdXG, int fdIrobot)
{
	char buf[1];

	pthread_t p_thread[3];
	int thr_id[3];

	sprintf(buf, "%c", Start);
	printf("[+] Send msg : %d\n", buf[0]);
	write(fdIrobot, buf, 1);

	sprintf(buf, "%c", SafeMode);
	printf("[+] Send msg : %d\n", buf[0]);
	write(fdIrobot, buf, 1);

	gettimeofday(&startTime, NULL);

	// camera captures by creating a thread & record all the info in 3 threads
	thr_id[0] = pthread_create(&p_thread[0], NULL, receivePGRCapture, (void *)&context);
	if(thr_id[0] < 0)
	{
		perror("Camera thread create error : ");
		exit(0);
	}

	// Listen to gyro angls by creating a thread
	thr_id[1] = pthread_create(&p_thread[1], NULL, receiveCensorXG, (void *)&fdXG);
	if(thr_id[1] < 0)
	{
		perror("Gyro thread create error : ");
		exit(0);
	}

	requestCensorEnc(fdIrobot);

	// Listen to encoders by creating a thread
	thr_id[2] = pthread_create(&p_thread[2], NULL, receiveCensorEnc, (void *)&fdIrobot);
	if(thr_id[2] < 0)
	{
		perror("iRobot thread create error : ");
		exit(0);
	}

}

void quit(int fd, fc2Context context)
{
	stopPGRCapture(context);
	stop(fd); // stop driving
	stopCensorEnc(fd);

	printf("\n[+] PGR / iRobot Sensor clear! Goodbye..\n");
}

void clean(int fd)
{
	char buf[10];

	sprintf(buf, "%c", Clean);
	printf("[+] Send msg : %d (Clean)\n", buf[0]);
	printf("[+] If you want to pause cleaning, Request Clean again.\n");
	write(fd, buf, 1);
}

void drive(int fd)
{
	char dir;

	initscr();
	raw();
	noecho();
	//keypad(stdscr, TRUE);

	printf("-- Moving Instruction --\n");
	printf("[Up]-Forward [Down]-Backward [Right]-Right [Left]-Left [Space]-Pause [Enter]-exit\n");

	while(1)
	{
	    dir = getch();
	    //printf("[-] Input code : %c\n", dir);

	    switch(dir) { // the real value
	        case 'A':
	            // code for arrow up
	        	forward(fd);
	            break;
	        case 'B':
	            // code for arrow down
	        	reverse(fd);
	            break;
	        case 'C':
	            // code for arrow right
	        	right(fd);
	            break;
	        case 'D':
	            // code for arrow left
	        	left(fd);
	            break;
	        case ' ':
	        	// code for spacebar
	        	stop(fd);
	        	break;
	        case 0xA: // enter key
	        	endwin();
	        	return;
	        default:
	        	//printf("%c %d\n", dir, dir);
	        	//printf("[-] Input code : %c\n", dir);
				//printf("[-] Invlid input. See the instruction below\n");
				//printf("[Up]-Forward\t[Down]-Backward\t[Right]-Right\t[Left]-Left\t[s] - Stop\n");
	        	continue;
		}
	}
	
}

void forward(int fd) // forward straight
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c",
		DriveDirect,
		(char)((SPEED_RIGHT_STRAIGHT>>8)&0xFF), (char)(SPEED_RIGHT_STRAIGHT&0xFF),
		(char)((SPEED_LEFT_STRAIGHT>>8)&0xFF), (char)(SPEED_LEFT_STRAIGHT&0xFF));

	printf("[+] Send msg : (Forward straight)\n");
	write(fd, buf, 5);
}

void forwardDistance(int fd, int distance) // forward for distnace
{
	char buf[5];

	// For sync issue, we define values for storing encode counters.
	// These values should be written once per loop and treated locally!
	unsigned short encLeftNow;
	unsigned short encRightNow;

	// Define previous encoder variables to be recorded.
	unsigned short encLeftPrev;
	unsigned short encRightPrev;

	// counters traveled for left/right encoders
	unsigned short leftCntTraveled;
	unsigned short rightCntTraveled;

	// counters needed to reach distance requested
	int cntNeeded;  

	sprintf(buf, "%c%c%c%c%c",
		DriveDirect,
		(char)((SPEED_RIGHT_STRAIGHT>>8)&0xFF), (char)(SPEED_RIGHT_STRAIGHT&0xFF),
		(char)((SPEED_LEFT_STRAIGHT>>8)&0xFF), (char)(SPEED_LEFT_STRAIGHT&0xFF));

	printf("[+] Send msg : (Forward for %d mm)\n", distance);
	write(fd, buf, 5);

	encLeftPrev = encLeftCnt;
	encRightPrev = encRightCnt;
	leftCntTraveled = 0;
	rightCntTraveled = 0;

	// Check left / right counter until reach distance
	// Use MM_PER_COUNTER
	cntNeeded = (int)((float)distance/MM_PER_COUNTER);
	printf("cntNeeded : %d\n\n", cntNeeded);

	while(1)
	{
		// no more use of global value
		encLeftNow = encLeftCnt;
		encRightNow = encRightCnt;

		// plus acc.
		if(encLeftPrev > encLeftNow)
			leftCntTraveled += (0xFFFF - (encLeftPrev - encLeftNow));
		else
			leftCntTraveled += (encLeftNow - encLeftPrev);

		encLeftPrev = encLeftNow;

		// plus acc.
		if(encRightPrev > encRightNow)
			rightCntTraveled += (0xFFFF - (encRightPrev - encRightNow));
		else
			rightCntTraveled += (encRightNow - encRightPrev);

		encRightPrev = encRightNow;

		if(leftCntTraveled >= cntNeeded && rightCntTraveled >= cntNeeded)
		{
			printf("[+] Stop forward. Counts to travel : +%d +%d\n", leftCntTraveled, rightCntTraveled);
			stop(fd);
			break;
		}
		usleep( 5 * 1000 );
	}
}

void reverse(int fd) // backward straight
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-SPEED_RIGHT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT_STRAIGHT)&0xFF), 
			(char)(((-SPEED_LEFT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_LEFT_STRAIGHT)&0xFF));

	printf("[+] Send msg : (Backward straight)\n");
	write(fd, buf, 5);
}

void reverseDistance(int fd, int distance) // backward for distance 
{
	char buf[5];

	// For sync issue, we define values for storing encode counters.
	// These values should be written once per loop and treated locally!
	unsigned short encLeftNow;
	unsigned short encRightNow;

	// Define previous encoder variables to be recorded.
	unsigned short encLeftPrev;
	unsigned short encRightPrev;

	// counters traveled for left/right encoders
	unsigned short leftCntTraveled;
	unsigned short rightCntTraveled;

	// counters needed to reach distance requested
	int cntNeeded;  

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-SPEED_RIGHT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT_STRAIGHT)&0xFF), 
			(char)(((-SPEED_LEFT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_LEFT_STRAIGHT)&0xFF));

	printf("[+] Send msg : (Backward for %d mm)\n", distance);
	write(fd, buf, 5);

	encLeftPrev = encLeftCnt;
	encRightPrev = encRightCnt;
	leftCntTraveled = 0;
	rightCntTraveled = 0;

	// Check left / right counter until reach distance
	// Use MM_PER_COUNTER
	cntNeeded = (int)((float)distance/MM_PER_COUNTER);
	printf("cntNeeded : %d\n\n", cntNeeded);

	while(1)
	{
		// no more use of global value
		encLeftNow = encLeftCnt;
		encRightNow = encRightCnt;

		// minus acc.
		if(encLeftPrev < encLeftNow)
			leftCntTraveled += (0xFFFF - (encLeftNow - encLeftPrev)); 
		else
			leftCntTraveled += (encLeftPrev - encLeftNow); 

		encLeftPrev = encLeftNow;

		// minus acc.
		if(encRightPrev < encRightNow)
			rightCntTraveled += (0xFFFF - (encRightNow - encRightPrev));
		else
			rightCntTraveled += (encRightPrev - encRightNow);

		encRightPrev = encRightNow;

		if(leftCntTraveled >= cntNeeded && rightCntTraveled >= cntNeeded)
		{
			printf("[+] Stop backward. Counts to travel : -%d -%d\n", leftCntTraveled, rightCntTraveled);
			stop(fd);
			break;
		}
		usleep( 5 * 1000 );
	}
}

void left(int fd)
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF), 
		(char)(((-SPEED_LEFT)>>8)&0xFF), (char)((-SPEED_LEFT)&0xFF));

	printf("[+] Send msg : (Left)\n");
	write(fd, buf, 5);
}

void leftAngle(int fd, int angle)
{
	char buf[5];

	// For sync issue, we define values for storing encode counters.
	// These values should be written once per loop and treated locally!
	unsigned short encLeftNow;
	unsigned short encRightNow;

	// Define previous encoder variables to be recorded.
	unsigned short encLeftPrev;
	unsigned short encRightPrev;

	// counters traveled for left/right encoders
	unsigned short leftCntTraveled;
	unsigned short rightCntTraveled;

	// counters needed to reach distance requested
	int cntNeeded;  

	int numOfRAngles;

	sprintf(buf, "%c%c%c%c%c", 
		Drive,
		(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF), 
		(char)(0), (char)(1)); // counter clockwise

	printf("[+] Send msg : (Left for %d degree)\n", angle);
	write(fd, buf, 5);

	encLeftPrev = encLeftCnt;
	encRightPrev = encRightCnt;
	leftCntTraveled = 0;
	rightCntTraveled = 0;
	// How many right angles to turn?
	numOfRAngles = angle/90;

	// Check left / right counter until reach distance
	// Use COUNTER_PER_RANGLE
	cntNeeded = COUNTER_PER_RANGLE * numOfRAngles;
	printf("cntNeeded : %d\n\n", cntNeeded);

	while(1)
	{
		// no more use of global value
		encLeftNow = encLeftCnt;
		encRightNow = encRightCnt;

		// minus acc.
		if(encLeftPrev < encLeftNow)
		{
			leftCntTraveled += (0xFFFF - (encLeftNow - encLeftPrev));
			printf("Rollover..  encLeftNow : %u encLeftPrev : %u\t", encLeftNow, encLeftPrev);
		} 
		else
			leftCntTraveled += (encLeftPrev - encLeftNow); 

		encLeftPrev = encLeftNow;

		// plus acc.
		if(encRightPrev > encRightNow)
		{
			rightCntTraveled += (0xFFFF - (encRightPrev - encRightNow));
			printf("Rollover..  encRightNow : %u encRightPrev : %u\t", encRightNow, encRightPrev); 
		}
		else
			rightCntTraveled += (encRightNow - encRightPrev); 

		encRightPrev = encRightNow;

		if(leftCntTraveled >= cntNeeded || rightCntTraveled >= cntNeeded)
		{
			printf("[+] Stop left turn. Counts to travel : -%d %d\n", leftCntTraveled, rightCntTraveled);
			stop(fd);
			break;
		}
		usleep( 5 * 1000 );
	}
}

void right(int fd)
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)(((-SPEED_RIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT)&0xFF), 
			(char)((SPEED_LEFT>>8)&0xFF), (char)(SPEED_LEFT&0xFF));

	printf("[+] Send msg : (Right)\n");
	write(fd, buf, 5);
}

void rightAngle(int fd, int angle)
{
	char buf[5];

	// For sync issue, we define values for storing encode counters.
	// These values should be written once per loop and treated locally!
	unsigned short encLeftNow;
	unsigned short encRightNow;

	// Define previous encoder variables to be recorded.
	unsigned short encLeftPrev;
	unsigned short encRightPrev;

	// counters traveled for left/right encoders
	unsigned short leftCntTraveled;
	unsigned short rightCntTraveled;

	// counters needed to reach distance requested
	int cntNeeded;  

	int numOfRAngles;

	sprintf(buf, "%c%c%c%c%c", 
		Drive, 
		(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF), 
		(char)(0xFF), (char)(0xFF)); //clockwise

	printf("[+] Send msg :(Right)\n");
	write(fd, buf, 5);

	encLeftPrev = encLeftCnt;
	encRightPrev = encRightCnt;
	leftCntTraveled = 0;
	rightCntTraveled = 0;
	// How many right angles to turn?
	numOfRAngles = angle/90;

	// Check left / right counter until reach distance
	// Use COUNTER_PER_RANGLE
	cntNeeded = COUNTER_PER_RANGLE * numOfRAngles;
	printf("cntNeeded : %d\n\n", cntNeeded);

	while(1)
	{
		// no more use of global value
		encLeftNow = encLeftCnt;
		encRightNow = encRightCnt;

		// plus acc.
		if(encLeftPrev > encLeftNow)
		{
			leftCntTraveled += (0xFFFF - (encLeftPrev - encLeftNow));
			printf("Rollover..  encLeftNow : %u encLeftPrev : %u\t", encLeftNow, encLeftPrev); 
		}
		else
			leftCntTraveled += (encLeftNow - encLeftPrev); // after-before = plus

		encLeftPrev = encLeftNow;

		// minus acc.
		if(encRightPrev < encRightNow)
		{
			rightCntTraveled += (0xFFFF - (encRightNow - encRightPrev));
			printf("Rollover..  encRightNow : %u encRightPrev : %u\t", encRightNow, encRightPrev); 
		}
		else
			rightCntTraveled += (encRightPrev - encRightNow);

		encRightPrev = encRightNow;

		if(leftCntTraveled >= cntNeeded || rightCntTraveled >= cntNeeded)
		{
			printf("[+] Stop right turn. Counts to travel : %d -%d\n", leftCntTraveled, rightCntTraveled);
			stop(fd);
			break;
		}
		usleep( 5 * 1000 );
	}
}

void stop(int fd)
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)(0),  (char)(0),  
		(char)(0),  (char)(0));

	printf("[+] Send msg : %d%d%d%d%d (Stop Driving)\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

	write(fd, buf, 5);
}

void stopLeft(int fd)
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF),  
		(char)(0),  (char)(0));

	printf("[+] Send msg : %d%d%d%d%d (Stop left motor only)\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

	write(fd, buf, 5);
}

void stopRight(int fd)
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)(0),  (char)(0),  
		(char)((SPEED_LEFT>>8)&0xFF), (char)(SPEED_LEFT&0xFF));

	printf("[+] Send msg : %d%d%d%d%d (Stop right motor only)\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

	write(fd, buf, 5);
}

void zigzag(int fd, int length, int width, int req_num_length)
{
	int num_length = 0;

	if(req_num_length < 1)
	{
		printf("[-] Requested number of length is not enough!\n");
	}

	while(1)
	{
		forwardDistance(fd, length);
		num_length++;
		usleep( 300 * 1000 );

		// check num_length per every cycle after forward(length)
		if(num_length == req_num_length)
		{
			printf("[+] Traveled %d driving of length. Zigzag over.\n", num_length);
			break;
		}

		// if num_length is odd, turn left -> go 'width' -> turn left
		if(num_length%2 == 1)
		{
			leftAngle(fd, 90);
			usleep( 300 * 1000 );
			forwardDistance(fd, width);
			usleep( 300 * 1000 );
			leftAngle(fd, 90);
			usleep( 300 * 1000 );
		}
		// if num_length is even, turn right -> go 'width' -> turn right
		else
		{
			rightAngle(fd, 90);
			usleep( 300 * 1000 );
			forwardDistance(fd, width);
			usleep( 300 * 1000 );
			rightAngle(fd, 90);
			usleep( 300 * 1000 );
		}
	}
}

// Thread for capturing images from PGR camera
void *receivePGRCapture(void *v_context)
{
	fc2Context context = *(fc2Context *)v_context;
	fc2Error error;
    fc2Image rawImage;
    fc2Image convertedImage;
    char filePath[10];
    char writeLine[100];
    int imageCnt = 0;
    int fdTxt; // file descriptor for writing file

    // Considering sync, we define temp data retrieved from gyro and encoder(irobot).
    // This value is assgined from that of global value "right after retrieving buffer data"
    float pgrElapsedTimeSync;
    float xgElapsedTimeSync;
    float xgAngleDataSync;
    float encElapsedTimeSync;
    unsigned short encLeftCntSync;
    unsigned short encRightCntSync;

    // ready for writing 
    fdTxt = open("./result/result.txt", O_WRONLY | O_CREAT, 0644);
	if(fdTxt < 0)
	{
		perror("./result/pgr.txt");
		exit(-1);
	}

    sprintf(writeLine, "TimeImg\tImage #\tTimeGyro\tdegree\tTimeEnc\tleftEnc\trightEnc\n");
    write(fdTxt, writeLine, strlen(writeLine));

    SetTimeStamping( context, TRUE );

    error = fc2StartCapture( context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2StartCapture: %d\n", error );

    	// Disconnect
	    error = fc2Disconnect( context );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "Error in fc2Disconnect: %d\n", error );
	    }

		// DestoryContext
	    error = fc2DestroyContext( context );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "Error in fc2DestroyContext: %d\n", error );
	    }

        exit(-1);
    }

	printf("[+] Start Recording.. \n");

    // If externally allocated memory is to be used for the converted image,
    // simply assigning the pData member of the fc2Image structure is
    // insufficient. fc2SetImageData() should be called in order to populate
    // the fc2Image structure correctly. This can be done at this point,
    // assuming that the memory has already been allocated.

    while(1)
    {
	    error = fc2CreateImage( &rawImage );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2CreateImage: %d\n", error );
	        stopPGRCapture (context);
	        exit(-1);
	    }

	    error = fc2CreateImage( &convertedImage );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2CreateImage: %d\n", error );
	        stopPGRCapture (context);
	        exit(-1);
	    }

        // Retrieve the image
        error = fc2RetrieveBuffer( context, &rawImage );
        if ( error != FC2_ERROR_OK )
        {
            printf( "[-] Error in retrieveBuffer: %d\n", error);
	        stopPGRCapture (context);
            exit(-1);
        }

        else if ( error == FC2_ERROR_OK )
    	{
    		// For the sync of data, store the data from gyro and irobot right after image retrieval
    		xgElapsedTimeSync = xgElapsedTime;
    		xgAngleDataSync = xgAngleData;
    		encElapsedTimeSync = encElapsedTime;
    		encLeftCntSync = encLeftCnt;
    		encRightCntSync = encRightCnt;

	        // Convert the final image to RGB
	        error = fc2ConvertImageTo(FC2_PIXEL_FORMAT_BGR, &rawImage, &convertedImage);
	        if ( error != FC2_ERROR_OK )
	        {
	            printf( "[-] Error in fc2ConvertImageTo: %d\n", error );
	        	stopPGRCapture (context);
	            exit(-1);
	        }

	        imageCnt++;

	  		// check and save the recording time of the retrieved data
	  		// WARNING ** there can be gap between pgr capture time and the other capture time
	  		// 		   ** because only pgr records the exact time from api when it records
	  		// 		   ** so, sync of this data doesn't have to be considered.
            fc2TimeStamp ts = fc2GetImageTimeStamp( &rawImage );
			pgrElapsedTime = (float)ts.microSeconds / 1000.0		// millisecconds
			pgrElapsedTimeSync = pgrElapsedTime;

			// Record saved image info
			// It would be best if we lock when we write...
	        sprintf(writeLine, "%.4f, %d, %.4f, %d, %.4f, %u, %u\n", 
	        	pgrElapsedTimeSync, imageCnt,							// record pgr data capture
	        	xgElapsedTimeSync, (int)xgAngleDataSync,				// record gyro data capture
	        	encElapsedTimeSync, encLeftCntSync, encRightCntSync);		// record irobot data capture
	        write(fdTxt, writeLine, strlen(writeLine));

	        // Save it to jpeg
	        sprintf(filePath, "./result/%d.jpeg", imageCnt);

			error = fc2SaveImage( &convertedImage, filePath, FC2_JPEG );
			if ( error != FC2_ERROR_OK )
			{
				printf( "[-] Error in saving image %d.jpeg: %d\n", imageCnt, error );
				printf( "[-] Please check write permissions.\n");
	        	stopPGRCapture (context);
				exit(-1);
			}
	        //printf("[+] [%f] Saving the last image to %d.png \n", pgrElapsedTime, imageCnt);

	    }

	    error = fc2DestroyImage( &rawImage );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2DestroyImage: %d\n", error );
	        stopPGRCapture (context);
	        exit(-1);
	    }

	    error = fc2DestroyImage( &convertedImage );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2DestroyImage: %d\n", error );
	        stopPGRCapture (context);
	        exit(-1);
	    }
    }
}

// for the safe use, set of disconnection from flycapture camera is called when error or exit
void stopPGRCapture(fc2Context context)
{
	fc2Error error;

	// Stop capture
    error = fc2StopCapture( context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "Error in fc2StopCapture: %d\n", error );
    }

	// Disconnect
    error = fc2Disconnect( context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "Error in fc2Disconnect: %d\n", error );
    }

	// DestoryContext
    error = fc2DestroyContext( context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "Error in fc2DestroyContext: %d\n", error );
    }

	printf("[+] PGR Camera working clear..")
}

void *receiveCensorXG(void *v_fd)
{
	int fd = *(int *)v_fd;
	short header;
	short rate_int;
	short angle_int;
	float rate_float;
	float angle_float;
	short check_sum;
	struct timeval xgEndTime;
	unsigned char data_packet[PACKET_SIZE];

	while(1)
	{
		if(PACKET_SIZE != read(fd, data_packet, PACKET_SIZE))
		{
			//printf("Not Valid Packet size\n");
			continue;
		}

		// Verify data packet header 
		memcpy(&header, data_packet, sizeof(short));
		if(header != (short)0xFFFF)
		{
			continue;
		}

		// Copy values from data string 
		memcpy(&rate_int, data_packet+2, sizeof(short));
		memcpy(&angle_int, data_packet+4, sizeof(short));
		memcpy(&check_sum, data_packet+6, sizeof(short));

		// Verify checksum
		if(check_sum != (short)(0xFFFF + rate_int + angle_int))
		{
			continue;
		}

		gettimeofday(&xgEndTime, NULL)
		xgElapsedTime = ((double)(xgEndTime.tv_sec)+(doube)(xgEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)startTime.tv_usec)/1000000.0)
	 	xgAngleData = angle_int;

		usleep( 15 * 1000 );
	}
}

// request stream for left / right encoding
void requestCensorEnc(int fd)
{
	char buf[4];

	// request censor stream for two bytes (LeftCnt / RightCnt)
	sprintf(buf, "%c%c%c%c", SensorStream, 2, LeftEncoderCounts, RightEncoderCounts);
	write(fd, buf, 4);
}

// Thread for receiving left right censor data
void *receiveCensorEnc(void *v_fd)
{
	int fd = *(int *)v_fd;
	unsigned short leften;
	unsigned short righten;
	struct timeval encEndTime;
	unsigned char data_packet[C_PACKET_SIZE];

	while(1)
	{
		// The data received should be 9 bytes
		// [1 hdr][1 nbytes][1 pktID1][2 rcvdata][1 pktID2][2 rcvdata][1 chksum]
		// [19][6][43][xxxx][44][xxxx][xxx]
		if(C_PACKET_SIZE != read(fd, data_packet, C_PACKET_SIZE))
		{
			//printf("Not valid packet size\n");
			continue;
		}

		// 9 bytes detected. check header and bytes
		if(data_packet[0] == 19 && data_packet[1] == 6)
		{
			// check packet ID 1
			if(data_packet[2] != 43 || data_packet[5] != 44)
			{
				continue;
			}
			leften = (data_packet[3] << 8) | data_packet[4];
			righten = (data_packet[6] << 8) | data_packet[7];

			gettimeofday(&encEndTime, NULL)
			encElapsedTime = ((double)(encEndTime.tv_sec)+(doube)(encEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)startTime.tv_usec)/1000000.0)

			encLeftCnt = leften;
			encRightCnt = righten;
			//printf("[+] [%f sec] Left/Right : [%u]\t[%u]\n", encElapsedTime, leften, righten);
		}

		usleep( 15 * 1000 );
	}
}

void stopCensorEnc(int fd)
{
	char buf[2];

	sprintf(buf, "%c%c", StreamPause, 0); // pause stream
	printf("[+] Send msg : %d%d (Pause Stream)\n", buf[0], buf[1]);
	write(fd, buf, 2);

	sprintf(buf, "%c", Stop); // stop OI
	printf("[+] Send msg : %d\n (Stop OI)", buf[0]);
	write(fd, buf, 1);
}