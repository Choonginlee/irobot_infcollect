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

//				iRobot Create 2 Packet IDs		//
const char		LeftEncoderCounts = 43;
const char		RightEncoderCounts = 44;
const char		Distance = 19;
const char		Angle = 20;

const int		GYRO_PACKET_SIZE = 8; 			// gyro packet size
const int		IROBOT_PACKET_SIZE = 2;			// irobot packet size
const float		MM_PER_COUNTER = 0.4446;		// mm travel per counter
const int		COUNTER_PER_RANGLE = 410;		// counters per 90 angle

// Global variable for system
struct timeval startTime;
int SPEED_LEFT =  100;							// slow speed in case of turning
int SPEED_RIGHT = 100;							// slow speed in case of turning
int SPEED_LEFT_STRAIGHT = 200;					// fast speed in case of straight
int SPEED_RIGHT_STRAIGHT = 200;					// fast speed in case of straight
typedef struct {								// structure for handlers (pgr, xg, irobot)
	fc2Context context;
	int fdGyro;
	int fdIRobot;
}Handlers;									

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

void drive(Handlers handler);
void zigzag(Handlers handler, int length, int width, int req_num_length);

void quit(Handlers handler);					// stop OI
void pauseDrive(int fd);						// pauseDrive driving
void forward(int fd);
void reverse(int fd);
void left(int fd);
void right(int fd);
void forwardDistance(int fd, int distance);
void reverseDistance(int fd, int distance);
void leftAngle(int fd, int angle);
void rightAngle(int fd, int angle);

void *receiveRecord(void *v_handler);
void retrieveGyro(Handlers handler);
void retrieveEncoder(Handlers handler);
void retrieveImage(Handlers handler);

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

	Handlers handler;
	handler.context = setPGR();
	handler.fdGyro = setGyro();
	handler.fdIRobot = setIRobot();

	showInstruction();

	while(1)
	{
		cmdRcvd = rcvCommand();

		switch(cmdRcvd)
		{
			case 1:
				drive(handler);
				quit(handler);
				break;
			case 2:
				return;
				printf("[Q] Please enter length / width / # of length : ");
				scanf("%d %d %d", &length, &width, &numlength);
				zigzag(handler, length, width, numlength);
				quit(handler);
				break;
			case 3:
				quit(handler);
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

/*
drive
- Make thread for receiveRecord
- handle real-time key input
*/
void drive(Handlers handler)
{
	pthread_t p_thread;
	int thr_id;
	int status;

	char dir;

	thr_id = pthread_create(&p_thread, NULL, receiveRecord, (void *)&handler);
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
	        	forward(handler.fdIRobot);
	            break;
	        case 'B':
	            // code for arrow down
	        	reverse(handler.fdIRobot);
	            break;
	        case 'C':
	            // code for arrow right
	        	right(handler.fdIRobot);
	            break;
	        case 'D':
	            // code for arrow left
	        	left(handler.fdIRobot);
	            break;
	        case ' ':
	        	// code for spacebar
	        	pauseDrive(handler.fdIRobot);
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

void forward(int fd) 
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c",
		DriveDirect,
		(char)((SPEED_RIGHT_STRAIGHT>>8)&0xFF), (char)(SPEED_RIGHT_STRAIGHT&0xFF),
		(char)((SPEED_LEFT_STRAIGHT>>8)&0xFF), (char)(SPEED_LEFT_STRAIGHT&0xFF));

	printf("[+] Send msg : (Forward straight)\n");
	write(fd, buf, 5);
}

void reverse(int fd) 
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-SPEED_RIGHT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT_STRAIGHT)&0xFF), 
			(char)(((-SPEED_LEFT_STRAIGHT)>>8)&0xFF), (char)((-SPEED_LEFT_STRAIGHT)&0xFF));

	printf("[+] Send msg : (Backward straight)\n");
	write(fd, buf, 5);
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

void pauseDrive(int fd)
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)(0),  (char)(0),  
		(char)(0),  (char)(0));

	printf("[+] Send msg : %d%d%d%d%d (Pause Driving)\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

	write(fd, buf, 5);
}

/*
zigag
- Make thread for receiveRecord
- request length / width / # of length
*/
void zigzag(Handlers handler, int length, int width, int req_num_length)
{

}

/*
designated driving commands
- forwardDistance
- reverseDistance
- leftAngle
- rightAngle
*/

void forwardDistance(int fd, int distance)
{

}

void reverseDistance(int fd, int distance)
{

}

void leftAngle(int fd, int angle)
{

}

void rightAngle(int fd, int angle)
{

}

/*
quit
- stop pgr capturing
- exit
*/
void quit(Handlers handler)
{
	fc2Error error;

	// Stop capture
    error = fc2StopCapture( handler.context );
    if ( error != FC2_ERROR_OK )
    {
        //printf( "Error in fc2StopCapture: %d\n", error );
    }

	// Disconnect
    error = fc2Disconnect( handler.context );
    if ( error != FC2_ERROR_OK )
    {
        //printf( "Error in fc2Disconnect: %d\n", error );
    }

	// DestoryContext
    error = fc2DestroyContext( handler.context );
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
void *receiveRecord(void *v_handler)
{
	Handlers handler = *(Handlers *)v_handler;
    char filePath[10];
    char writeLine[100];
    int fdTxt; // file descriptor for writing file

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

    // Start Recording
    while(1)
    {
    	retrieveGyro(handler);
    	retrieveEncoder(handler);
    	retrieveImage(handler);

		// Record saved image info
        sprintf(writeLine, "%.4f, %d, %.4f, %d, %.4f, %u, %u\n", 
        	pgrElapsedTime, pgrImageNumber,				// record pgr data capture
        	gyroElapsedTime, gyroAngleData,				// record gyro data capture
        	encElapsedTime, encLeftCnt, encRightCnt);	// record irobot data capture
        write(fdTxt, writeLine, strlen(writeLine));
    }


}

void retrieveGyro(Handlers handler)
{
	short header;
	short rate_int;
	short angle_int;
	float rate_float;
	float angle_float;
	short check_sum;
	struct timeval gyroEndTime;
	unsigned char data_packet[GYRO_PACKET_SIZE];

	while(1)
	{
		if(GYRO_PACKET_SIZE != read(handler.fdGyro, data_packet, GYRO_PACKET_SIZE))
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

void retrieveEncoder(Handlers handler)
{
	char buf[2];
	unsigned short leften;
	unsigned short righten;
	struct timeval encEndTime;
	unsigned char data_packet[IROBOT_PACKET_SIZE];

	// request censor stream for two bytes (LeftCnt)
	sprintf(buf, "%c%c", Sensors, LeftEncoderCounts);
	write(handler.fdIRobot, buf, 2);
	while(1)
	{
		if( IROBOT_PACKET_SIZE != read(handler.fdIRobot, data_packet, IROBOT_PACKET_SIZE) )
			continue;

		leften = (data_packet[0] << 8) | data_packet[1];
		break;
	}

	// request censor stream for two bytes (RightCnt)
	sprintf(buf, "%c%c", Sensors, RightEncoderCounts);
	write(handler.fdIRobot, buf, 2);
	while(1)
	{
		if( IROBOT_PACKET_SIZE != read(handler.fdIRobot, data_packet, IROBOT_PACKET_SIZE) )
			continue;

		righten = (data_packet[0] << 8) | data_packet[1];
		break;
	}

	gettimeofday(&encEndTime, NULL);

	encElapsedTime = ((double)(encEndTime.tv_sec)+(double)(encEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)(startTime.tv_usec)/1000000.0);
	encLeftCnt = leften;
	encRightCnt = righten;

	return;
}

void retrieveImage(Handlers handler)
{
	char filePath[10];
	fc2Error error;
    fc2Image rawImage;
    fc2Image convertedImage;
	struct timeval pgrEndTime;

    int imageCnt = 0;

    error = fc2StartCapture( handler.context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2StartCapture: %d\n", error );

    	// Disconnect
	    error = fc2Disconnect( handler.context );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2Disconnect: %d\n", error );
	    }

		// DestoryContext
	    error = fc2DestroyContext( handler.context );
	    if ( error != FC2_ERROR_OK )
	    {
	        printf( "[-] Error in fc2DestroyContext: %d\n", error );
	    }

        exit(0);
    }

	error = fc2CreateImage( &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2CreateImage: %d\n", error );
        quit(handler);
    }

    error = fc2CreateImage( &convertedImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2CreateImage: %d\n", error );
        quit(handler);
    }

	gettimeofday(&pgrEndTime, NULL);
	
    // Retrieve the image
    error = fc2RetrieveBuffer( handler.context, &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in retrieveBuffer: %d\n", error);
        quit(handler);
    }

    else if ( error == FC2_ERROR_OK )
	{
        // Convert the final image to RGB
        error = fc2ConvertImageTo(FC2_PIXEL_FORMAT_BGR, &rawImage, &convertedImage);
        if ( error != FC2_ERROR_OK )
        {
            printf( "[-] Error in fc2ConvertImageTo: %d\n", error );
        	quit(handler);
        }
        imageCnt++;

        // Save it to jpeg
        sprintf(filePath, "./result/%d.jpeg", imageCnt);

		error = fc2SaveImage( &convertedImage, filePath, FC2_JPEG );
		if ( error != FC2_ERROR_OK )
		{
			printf( "[-] Error in saving image %d.jpeg: %d\n", imageCnt, error );
			printf( "[-] Please check write permissions.\n");
        	quit(handler);
		}
    }

    error = fc2DestroyImage( &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2DestroyImage: %d\n", error );
        quit(handler);
    }

    error = fc2DestroyImage( &convertedImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2DestroyImage: %d\n", error );
        quit(handler);
    }

    pgrElapsedTime = ((double)(pgrEndTime.tv_sec)+(double)(pgrEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)(startTime.tv_usec)/1000000.0);
    pgrImageNumber = imageCnt;
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

