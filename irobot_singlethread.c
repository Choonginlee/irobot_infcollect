#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include "/usr/include/flycapture/C/FlyCapture2_C.h"

//				Create Command					// Arguments
const char		Start = 128;
const char		Stop = 173;
const char		SafeMode = 131;
const char		FullMode = 132;
const char		Clean = 135;
const char		DriveDirect = 145;				// 4:   [Right Hi] [Right Low] [Left Hi] [Left Low]
const char		Sensors = 142;					// 1:    Sensor Packet ID
const char		SensorStream = 148;         // x+1: [# of packets requested] IDs of requested packets to stream
const char		StreamPause = 150;

//				iRobot Create 2 Packet IDs		//
const char		LeftEncoderCounts = 43;
const char		RightEncoderCounts = 44;
const char		Distance = 19;
const char		Angle = 20;

const int		GYRO_PACKET_SIZE = 8; 			// gyro packet size
const int		IROBOT_PACKET_SIZE = 9;			// irobot packet size
const float		MM_PER_COUNTER = 0.4446;		// mm travel per counter
const int		COUNTER_PER_RANGLE = 410;		// counters per 90 angle

// Global variable for system
struct timeval startTime;
int SPEED_LEFT =  100;							// slow speed in case of turning
int SPEED_RIGHT = 100;							// slow speed in case of turning
int SPEED_LEFT_STRAIGHT = 200;					// fast speed in case of straight
int SPEED_RIGHT_STRAIGHT = 200;					// fast speed in case of straight
/*typedef struct {								// structure for handlers (pgr, xg, irobot)
	fc2Context context;
	int fdGyro;
	int fdIRobot;
}Handlers;		*/

fc2Context context;
int fdGyro;
int fdIRobot;							

// Global variable for record
float gyroElapsedTime;
int gyroAngleData;
float encElapsedTime;
unsigned short encLeftCnt;
unsigned short encRightCnt;
float pgrElapsedTime;
int pgrImageNumber;

void showInstruction();
int rcvCommand();

fc2Context setPGR();
int setGyro();
int setIRobot();
//void SetTimeStamping( fc2Context context, BOOL enableTimeStamp );

void drive();
void zigzag(int length, int width, int req_num_length);

void start();					// send start and safemode command
void quit();					// stop OI
void pauseDrive();						// pauseDrive driving
void forward();
void reverse();
void left();
void right();
void forwardDistance(int distance);
void reverseDistance(int distance);
void leftAngle(int angle);
void rightAngle(int angle);

void *receiveRecord(void *status);
void retrieveGyro();
void retrieveEncoder();
void retrieveImage();

/* 
main
- Call set PGR XG IROBOT
- Receive command from users (drive / zigzag / quit)
*/
void main()
{
	int cmdRcvd;
	int length;
	int width;
	int numlength;

	//Handlers handler;
	context = setPGR();
	fdGyro = setGyro();
	fdIRobot = setIRobot();
	//printf("[+] gyro : %d irobot : %d\n", handler.fdGyro, handler.fdIRobot);
	showInstruction();

	while(1)
	{
		cmdRcvd = rcvCommand();

		switch(cmdRcvd)
		{
			case 1:
				start();
				drive();
				quit();
				break;
			case 2:
				return;
				start();
				printf("[Q] Please enter length / width / # of length : ");
				scanf("%d %d %d", &length, &width, &numlength);
				zigzag(length, width, numlength);
				quit();
				break;
			case 3:				
				quit();
				break;
			default:
				printf("[-] Wrong input! Exit. : ");
				return;
		}
	}

	return;
}

void showInstruction()
{
	printf("=========================================\n");
	printf("1. Realtime drive 			--> Type 1\n");
	printf("2. Zigzag 					--> Type 2\n");
	printf("3. Quit and Clear camera 	--> Type 3\n\n");
	printf("[Info] Make sure you clear previous data by pressing 3 after emergency stop.\n");
	printf("[Info] Please check connection status before start.\n");
	printf("[Info] ttyUSB0 : Gyro (XG1010) | ttyUSB1 : iRobot Create 2\n");
	printf("=========================================\n");
}

int rcvCommand()
{
	int command;
	printf("Enter command : ");
	scanf("%d", &command);
	return command;
}

void start()
{
	char buf[1];

	sprintf(buf, "%c", Start);
	printf("[+] Send msg : %d\n", buf[0]);
	write(fdIRobot, buf, 1);

	sprintf(buf, "%c", SafeMode);
	printf("[+] Send msg : %d\n", buf[0]);
	write(fdIRobot, buf, 1);
}

/*
drive
- Make thread for receiveRecord
- handle real-time key input
*/
void drive()
{
	//printf("[+] gyro : %d irobot : %d\n", handler->fdGyro, handler->fdIRobot);
	pthread_t p_thread;
	int thr_id;
	int status;

	char dir;

	thr_id = pthread_create(&p_thread, NULL, receiveRecord, (void *)&status);
	if(thr_id < 0)
	{
		perror("[-] Thread create error : ");
		exit(0);
	}

	initscr();
	raw();
	noecho();

	while(1)
	{
	    dir = getch();
	    //printf("[-] Input code : %c\n", dir);

	    switch(dir) { // the real value
	        case 'A':
	            // code for arrow up
	        	forward();
	            break;
	        case 'B':
	            // code for arrow down
	        	reverse();
	            break;
	        case 'C':
	            // code for arrow right
	        	right();
	            break;
	        case 'D':
	            // code for arrow left
	        	left();
	            break;
	        case ' ':
	        	// code for spacebar
	        	pauseDrive();
	        	break;
	        case 0xA: 
	        	// enter key
	        	endwin();
	        	return;
	        default:
	        	continue;
		}
	}
}

/*
real-time driving commands
- forward
- reverse
- left
- right
- pauseDrive
*/

void forward() 
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c",
		DriveDirect,
		(char)((SPEED_RIGHT_STRAIGHT>>8)&0xFF), (char)(SPEED_RIGHT_STRAIGHT&0xFF),
		(char)((SPEED_LEFT_STRAIGHT>>8)&0xFF), (char)(SPEED_LEFT_STRAIGHT&0xFF));

	printf("[+] Send msg : (Forward straight)\n");
	write(fdIRobot, buf, 5);
}

void reverse() 
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-SPEED_RIGHT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT_STRAIGHT)&0xFF), 
			(char)(((-SPEED_LEFT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_LEFT_STRAIGHT)&0xFF));

	printf("[+] Send msg : (Backward straight)\n");
	write(fdIRobot, buf, 5);
}

void left()
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF), 
		(char)(((-SPEED_LEFT)>>8)&0xFF), (char)((-SPEED_LEFT)&0xFF));

	printf("[+] Send msg : (Left)\n");
	write(fdIRobot, buf, 5);
}

void right()
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)(((-SPEED_RIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT)&0xFF), 
			(char)((SPEED_LEFT>>8)&0xFF), (char)(SPEED_LEFT&0xFF));

	printf("[+] Send msg : (Right)\n");
	write(fdIRobot, buf, 5);
}

void pauseDrive()
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)(0),  (char)(0),  
		(char)(0),  (char)(0));

	printf("[+] Send msg : %d%d%d%d%d (Pause Driving)\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

	write(fdIRobot, buf, 5);
}

/*
zigag
- Make thread for receiveRecord
- request length / width / # of length
*/
void zigzag(int length, int width, int req_num_length)
{

}

/*
designated driving commands
- forwardDistance
- reverseDistance
- leftAngle
- rightAngle
*/

void forwardDistance(int distance)
{

}

void reverseDistance(int distance)
{

}

void leftAngle(int angle)
{

}

void rightAngle(int angle)
{

}

/*
quit
- stop pgr capturing
- exit
*/
void quit()
{
	fc2Error error;
	// Stop capture
    error = fc2StopCapture( context );
    if ( error != FC2_ERROR_OK )
    {
        //printf( "Error in fc2StopCapture: %d\n", error );
    }

	// Disconnect
    error = fc2Disconnect( context );
    if ( error != FC2_ERROR_OK )
    {
        //printf( "Error in fc2Disconnect: %d\n", error );
    }

	// DestoryContext
    error = fc2DestroyContext( context );
    if ( error != FC2_ERROR_OK )
    {
        //printf( "Error in fc2DestroyContext: %d\n", error );
    }

	printf("[+] PGR Camera working clear..\n");
	printf("=====   GOOD BYE   =====\n");

    exit(0);
}


/*
receiveRecord
= Thread for record info from 3 devices
- Retrieve Gyro		[1]
- Retrieve Enc 		[2]
- Retrieve Image 	[3]
- Record [1-3]
- Retrieval method records data on global variable
*/
void *receiveRecord(void *status)
{
	fc2Error error;
    char filePath[10];
    char writeLine[100];
    int fdTxt; // file descriptor for writing file
    int imageCnt = 0;

    // base time set
    gettimeofday(&startTime, NULL);
    //SetTimeStamping( context, TRUE );

    // ready for writing 
    fdTxt = open("./result/result.txt", O_WRONLY | O_CREAT, 0644);
	if(fdTxt < 0)
	{
		perror("./result/pgr.txt");
		exit(0);
	}
    sprintf(writeLine, "TimeImg\tImage #\tTimeGyro\tdegree\tTimeEnc\tleftEnc\trightEnc\n");
    write(fdTxt, writeLine, strlen(writeLine));

    error = fc2StartCapture( context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2StartCapture: %d\n", error );

    	// Disconnect
	    error = fc2Disconnect( context );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2Disconnect: %d\n", error );
	    }

		// DestoryContext
	    error = fc2DestroyContext( context );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2DestroyContext: %d\n", error );
	    }

        exit(0);
    }

    // Start Recording
    while(1)
    {
    	//printf("[+] Enter retrieveEncoder \n");
    	retrieveEncoder();
    	//printf("[+] Enter retrieveGyro \n");
    	retrieveGyro();
   		imageCnt++;
   		pgrImageNumber = imageCnt;
    	//printf("[+] Enter retrieveImage \n");
    	retrieveImage();

		// Record saved image info
        sprintf(writeLine, "%.4f, %d, %.4f, %d, %.4f, %u, %u\n", 
        	pgrElapsedTime, pgrImageNumber,				// record pgr data capture
        	gyroElapsedTime, gyroAngleData,				// record gyro data capture
        	encElapsedTime, encLeftCnt, encRightCnt);	// record irobot data capture
        write(fdTxt, writeLine, strlen(writeLine));
    }


}

void retrieveGyro()
{
	short header;
	short rate_int;
	short angle_int;
	float rate_float;
	float angle_float;
	short check_sum;
	struct timeval gyroEndTime;
	unsigned char data_packet[GYRO_PACKET_SIZE];

	//printf("[+] gyro : %d irobot : %d\n", handler->fdGyro, handler->fdIRobot);

	while(1)
	{
		if(GYRO_PACKET_SIZE != read(fdGyro, data_packet, GYRO_PACKET_SIZE))
		{
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

		gettimeofday(&gyroEndTime, NULL);

		gyroElapsedTime = ((double)(gyroEndTime.tv_sec)+(double)(gyroEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)(startTime.tv_usec)/1000000.0);
	 	gyroAngleData = angle_int; // degree * 100 is angle_int

	 	return;
	}
}

void retrieveEncoder()
{
	char buf[4];
	unsigned short leften;
	unsigned short righten;
	struct timeval encEndTime;
	unsigned char data_packet[IROBOT_PACKET_SIZE];

	// request censor stream for two bytes (LeftCnt / RightCnt)
	sprintf(buf, "%c%c%c%c", SensorStream, 2, LeftEncoderCounts, RightEncoderCounts);
	write(fdIRobot, buf, 4);

	while(1)
	{
		// The data received should be 9 bytes
		// [1 hdr][1 nbytes][1 pktID1][2 rcvdata][1 pktID2][2 rcvdata][1 chksum]
		// [19][6][43][xxxx][44][xxxx][xxx]
		if(IROBOT_PACKET_SIZE != read(fdIRobot, data_packet, IROBOT_PACKET_SIZE))
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

			break;
		}
	}

	gettimeofday(&encEndTime, NULL);

	sprintf(buf, "%c%c", StreamPause, 0); // pause stream
	write(fdIRobot, buf, 2);

	encElapsedTime = ((double)(encEndTime.tv_sec)+(double)(encEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)(startTime.tv_usec)/1000000.0);
	encLeftCnt = leften;
	encRightCnt = righten;

	return;
}

void retrieveImage()
{
	char filePath[10];
	fc2Error error;
    fc2Image rawImage;
    fc2Image convertedImage;
	struct timeval pgrEndTime;

	error = fc2CreateImage( &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2CreateImage: %d\n", error );
        quit();
    }

    error = fc2CreateImage( &convertedImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2CreateImage: %d\n", error );
        quit();
    }

	gettimeofday(&pgrEndTime, NULL);
	
    // Retrieve the image
    error = fc2RetrieveBuffer( context, &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in retrieveBuffer: %d\n", error);
        quit();
    }

    else if ( error == FC2_ERROR_OK )
	{
        // Convert the final image to RGB
        error = fc2ConvertImageTo(FC2_PIXEL_FORMAT_BGR, &rawImage, &convertedImage);
        if ( error != FC2_ERROR_OK )
        {
            printf( "[-] Error in fc2ConvertImageTo: %d\n", error );
        	quit();
        }

        // Save it to jpeg
        sprintf(filePath, "./result/%d.jpeg", pgrImageNumber);

		error = fc2SaveImage( &convertedImage, filePath, FC2_JPEG );
		if ( error != FC2_ERROR_OK )
		{
			printf( "[-] Error in saving image %d.jpeg: %d\n", pgrImageNumber, error );
			printf( "[-] Please check write permissions.\n");
        	quit();
		}
    }

    error = fc2DestroyImage( &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2DestroyImage: %d\n", error );
        quit();
    }

    error = fc2DestroyImage( &convertedImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2DestroyImage: %d\n", error );
        quit();
    }

    pgrElapsedTime = ((double)(pgrEndTime.tv_sec)+(double)(pgrEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)(startTime.tv_usec)/1000000.0);
    //fc2TimeStamp ts = fc2GetImageTimeStamp( &rawImage );
}

/* 
set PGR / XG / IROBOT
- Setting property for each device
*/

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
        exit(0);
    }

    error = fc2GetNumOfCameras( context, &numCameras );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2GetNumOfCameras: %d\n", error );
        exit(0);
    }

    if ( numCameras == 0 )
    {
        // No cameras detected
        printf( "[-] No PGR cameras detected.\n");
        exit(0);
    }

    // Get the 0th camera
    error = fc2GetCameraFromIndex( context, 0, &guid );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2GetCameraFromIndex: %d\n", error );
        exit(0);
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

        exit(0);
    }

	return context;
}

int setGyro()
{
	int fd;
	struct termios serialio;
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(fd < 0)
	{
		printf("[-] Check the connection of Gyro consor.\n");
		perror("/dev/ttyUSB0");
		exit(0);
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
		exit(0);
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

