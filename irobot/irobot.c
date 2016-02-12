#if defined(WIN32) || defined(WIN64)
#define _CRT_SECURE_NO_WARNINGS		
#endif

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
 
clock_t startTime;

int SPEED_LEFT =  200;
int SPEED_RIGHT = 200;

fc2Context setPGR();
int setXG();
int setIRobot();
void showInstruction();
int rcvCommand();

void start(fc2Context context, int fdXG, int fdIrobot);
void quit(int fd);		// stop OI
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
void zigzag(int fd, int length, int width, int req_num_length);

void *receivePGRCapture(void *v_context);
void GrabImages(fc2Context context);
void *receiveCensorXG(void *fd);
void requestCensorEnc(int fd);
void *receiveCensorEnc(void *fd);

void main()
{
	fc2Context contextPGR;			// Context handler of PGR camera
	int fdXG;						// Serial handler of XG1010
	int fdIrobot;					// Serial handler of irobot

	// set handlers
	contextPGR = setPGR();
	fdXG = setXG();
	fdIrobot = setIRobot();

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
				zigzag(fdIrobot, 1000, 500, 3);
				stop(fdIrobot);
				break;
			case 5:
				quit(fdIrobot);
				break;
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
	printf("5. To quit, Type 5\n\n");
	printf("[+] Please make sure to start by pressing 1 first.\n");
	printf("[+] Please check connection status before start.\n");
	printf("[+] ttyUSB0 : Gyro (XG1010) | ttyUSB1 : iRobot Create 2\n");
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
// It tries to listen left / right encoder
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

	startTime = clock();

	// camera captures by creating a thread
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

	// Listen to encoders by creating a thread
	thr_id[2] = pthread_create(&p_thread[2], NULL, receiveCensorEnc, (void *)&fdIrobot);
	if(thr_id[2] < 0)
	{
		perror("iRobot thread create error : ");
		exit(0);
	}

}

void quit(int fd)
{
	char buf[10];

	stop(fd);

	sprintf(buf, "%c", Stop);
	printf("[+] Send msg : %s\n", buf);
	write(fd, buf, 1);

	printf("Goodbye..\n");
}

void clean(int fd)
{
	char buf[10];

	sprintf(buf, "%c", Clean);
	printf("[+] Send msg : %s\n", buf);
	printf("[+] If you want to pause cleaning, Request Clean again.\n");
	write(fd, buf, 1);
}

void drive(int fd)
{
	char dir;

	initscr();
	cbreak();
	//keypad(stdscr, TRUE);

	printf("-- Moving Instruction --\n");
	printf("[Up]-Forward\t[Down]-Backward\t[Right]-Right\t[Left]-Left\t[s] - Stop\n");

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
	        default:
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
		(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF),
		(char)((SPEED_LEFT>>8)&0xFF), (char)(SPEED_LEFT&0xFF));

	printf("[+] Send msg : %s (Forward straight)\n", buf);
	write(fd, buf, 5);
}

void forwardDistance(int fd, int distance) // forward for distnace
{
	char buf[5];
	int waittime = 0;

	sprintf(buf, "%c%c%c%c%c",
		DriveDirect,
		(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF),
		(char)((SPEED_LEFT>>8)&0xFF), (char)(SPEED_LEFT&0xFF));

	printf("[+] Send msg : %s (Forward for %d mm)\n", buf, distance);
	write(fd, buf, 5);

	// Time = Distance (mm) / Velocity (mm)
	waittime = (int)(distance / SPEED_RIGHT);
	sleep(waittime);
}

void reverse(int fd) // backward straight
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-SPEED_RIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT)&0xFF), 
			(char)(((-SPEED_LEFT)>>8)&0xFF), (char)((-SPEED_LEFT)&0xFF));

	printf("[+] Send msg : %s (Backward straight)\n", buf);
	write(fd, buf, 5);
}

void reverseDistance(int fd, int distance) // backward for distance 
{
	char buf[5];
	int waittime = 0;

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-SPEED_RIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT)&0xFF), 
			(char)(((-SPEED_LEFT)>>8)&0xFF), (char)((-SPEED_LEFT)&0xFF));

	printf("[+] Send msg : %s (Backward for %d mm)\n", buf, distance);
	write(fd, buf, 5);

	// Time = Distance (mm) / Velocity (mm)
	waittime = (int)(distance / SPEED_RIGHT);
	sleep(waittime);
}

void left(int fd)
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF), 
			(char)(((-SPEED_LEFT)>>8)&0xFF), (char)((-SPEED_LEFT)&0xFF));

	printf("[+] Send msg : %s (Left)\n", buf);
	write(fd, buf, 5);
}

void leftAngle(int fd, int angle)
{
	char buf[5];
	int waittime = 0;

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)((SPEED_RIGHT>>8)&0xFF), (char)(SPEED_RIGHT&0xFF), 
			(char)(((-SPEED_LEFT)>>8)&0xFF), (char)((-SPEED_LEFT)&0xFF));

	printf("[+] Send msg : %s (Left for %d degree)\n", buf, angle);
	write(fd, buf, 5);

	// 200mm velocity : 90 degrees per sec
	waittime = (int)(angle / 90);
	sleep(waittime);
}

void right(int fd)
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)(((-SPEED_RIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT)&0xFF), 
			(char)((SPEED_LEFT>>8)&0xFF), (char)(SPEED_LEFT&0xFF));

	printf("[+] Send msg : %s (Right)\n", buf);
	write(fd, buf, 5);
}

void rightAngle(int fd, int angle)
{
	char buf[5];
	int waittime = 0;

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)(((-SPEED_RIGHT)>>8)&0xFF), (char)((-SPEED_RIGHT)&0xFF), 
			(char)((SPEED_LEFT>>8)&0xFF), (char)(SPEED_LEFT&0xFF));

	printf("[+] Send msg : %s (Right)\n", buf);
	write(fd, buf, 5);

	// 200mm velocity : 90 degrees per sec
	waittime = (int)(angle / 90);
	sleep(waittime);
}

void stop(int fd)
{
	char buf[5];

	sprintf(buf, "%c%c%c%c%c", 
		DriveDirect, 
		(char)(0),  (char)(0),  
		(char)(0),  (char)(0));

	printf("[+] Send msg : %s (Stop)\n", buf);

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
			forwardDistance(fd, width);
			leftAngle(fd, 90);
		}
		// if num_length is even, turn right -> go 'width' -> turn right
		else
		{
			rightAngle(fd, 90);
			forwardDistance(fd, width);
			rightAngle(fd, 90);
		}
	}
}

// Thread for capturing images from PGR camera
void *receivePGRCapture(void *v_context)
{
	fc2Context context = *(fc2Context *)v_context;
	fc2Error error;

    error = fc2StartCapture( context );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2StartCapture: %d\n", error );
        exit(-1);
    }

    GrabImages(context);   
}

void GrabImages(fc2Context context)
{
    fc2Error error;
    fc2Image rawImage;
    fc2Image convertedImage;
    char fileName[10];
    float elapsedTime;
    int imageCnt = 0;

    error = fc2CreateImage( &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2CreateImage: %d\n", error );
        exit(-1);
    }

    error = fc2CreateImage( &convertedImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2CreateImage: %d\n", error );
        exit(-1);
    }

    // If externally allocated memory is to be used for the converted image,
    // simply assigning the pData member of the fc2Image structure is
    // insufficient. fc2SetImageData() should be called in order to populate
    // the fc2Image structure correctly. This can be done at this point,
    // assuming that the memory has already been allocated.

    while(1)
    {
        // Retrieve the image
        error = fc2RetrieveBuffer( context, &rawImage );
        if ( error != FC2_ERROR_OK )
        {
            printf( "[-] Error in retrieveBuffer: %d\n", error);
            exit(-1);
        }
        else if ( error == FC2_ERROR_OK )
    	{
	        // Convert the final image to RGB
	        error = fc2ConvertImageTo(FC2_PIXEL_FORMAT_BGR, &rawImage, &convertedImage);
	        if ( error != FC2_ERROR_OK )
	        {
	            printf( "[-] Error in fc2ConvertImageTo: %d\n", error );
	            exit(-1);
	        }

	        imageCnt++;
			elapsedTime = (clock()-startTime)/100000.0;
	        // Save it to PNG
	        printf("[+] [%f] Saving the last image to %d.png \n", elapsedTime, imageCnt);

	        // file name change!

	        sprintf(fileName, "%d.png", imageCnt);
			error = fc2SaveImage( &convertedImage, fileName, FC2_PNG );
			if ( error != FC2_ERROR_OK )
			{
				printf( "[-] Error in saving image %d.png: %d\n", imageCnt, error );
				printf( "[-] Please check write permissions.\n");
				exit(-1);
			}			
	    }
    }

    error = fc2DestroyImage( &rawImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2DestroyImage: %d\n", error );
        exit(-1);
    }

    error = fc2DestroyImage( &convertedImage );
    if ( error != FC2_ERROR_OK )
    {
        printf( "[-] Error in fc2DestroyImage: %d\n", error );
        exit(-1);
    }
}

void *receiveCensorXG(void *v_fd)
{
	int fd = *(int *)v_fd;
	float elapsedTime;
	short header;
	short rate_int;
	short angle_int;
	float rate_float;
	float angle_float;
	short check_sum;
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

		// Apply scale factors
		rate_float = rate_int/100.0;
	 	angle_float = angle_int/100.0;
		
		printf("angle_float : %f [deg]\n", angle_float);
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
	unsigned char data_packet[C_PACKET_SIZE];
	float elapsedTime;

	requestCensorEnc(fd);

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
			leften = *(short *)&data_packet[3];
			righten = *(short *)&data_packet[6];

			elapsedTime = (clock()-startTime)/100000.0;
			// save the left encoder data
			printf("[+] [%f sec] Left/Right : [%u]\t[%u]\n", elapsedTime, leften, righten);
		}

		usleep( 15 * 1000 );
	}
}