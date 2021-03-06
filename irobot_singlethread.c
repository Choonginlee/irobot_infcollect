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

//              Create Command                  // Arguments
const char      Start = 128;
const char      Stop = 173;
const char      Reset = 7;
const char      SafeMode = 131;
const char      FullMode = 132;
const char      Clean = 135;
const char      DriveDirect = 145;              // 4:   [Right Hi] [Right Low] [Left Hi] [Left Low]
const char      Sensors = 142;                  // 1:    Sensor Packet ID
const char      SensorStream = 148;         // x+1: [# of packets requested] IDs of requested packets to stream
const char      StreamPause = 150;

//              iRobot Create 2 Packet IDs      //
const char      LeftEncoderCounts = 43;
const char      RightEncoderCounts = 44;
const char      Distance = 19;
const char      Angle = 20;

const int       GYRO_PACKET_SIZE = 8;           // gyro packet size
const int       IROBOT_PACKET_SIZE_STREAM = 9;  // irobot packet size
const int       IROBOT_PACKET_SIZE_SENSORS = 2; // irobot packet size
const float     MM_PER_COUNTER = 0.4446;        // mm travel per counter
const int       COUNTER_PER_RANGLE = 410;       // counters per 90 angle

// Global variable for system
struct timeval startTime;
int SPEED_LEFT =  100;                          // slow speed in case of turning
int SPEED_RIGHT = 100;                          // slow speed in case of turning
int SPEED_LEFT_STRAIGHT = 200;                  // fast speed in case of straight
int SPEED_RIGHT_STRAIGHT = 200;                 // fast speed in case of straight
int ROLLOVER_BOUNDARY = 65510;
/*typedef struct {                              // structure for handlers (pgr, xg, irobot)
    fc2Context context;
    int fdGyro;
    int fdIRobot;
}Handlers;      */
pthread_t p_thread;
int thr_id;
int status;

fc2Context context;
int fdGyro;
int fdIRobot;                           

// Global variable for record
float gyroElapsedTime;
int gyroAngleData;
float encLElapsedTime;
float encRElapsedTime;
unsigned short encLeftCnt;
unsigned short encRightCnt;
float pgrElapsedTime;
int pgrImageNumber;
unsigned short leftenPrev=0;        // this is for storing last value
unsigned short rightenPrev=0;       // this is for storing last value

void showInstruction();
int rcvCommand();

fc2Context setPGR();
int setGyro();
int setIRobot();
//void SetTimeStamping( fc2Context context, BOOL enableTimeStamp );

void drive();
void zigzag(int length, int width, int req_num_length);

void start();                   // send start and safemode command
void reset();
void clean();
void quit();                    // stop OI
void pauseDrive();                      // pauseDrive driving
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
    showInstruction();

    while(1)
    {
        cmdRcvd = rcvCommand();

        switch(cmdRcvd)
        {
            case 1:
                start();
                drive();
                quit(); // end process
                break;
            case 2:
                start();
                return;
                printf("[Q] Please enter length / width / # of length : ");
                scanf("%d %d %d", &length, &width, &numlength);
                zigzag(length, width, numlength);
                quit(); // end process
                break;
            case 3:             
                quit();
                break;
            case 4:
                reset();
                break;
            case 5:
                clean();
                break;
            case 6:
                start();
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
    printf("1. Realtime drive\t--> Type 1\n");
    printf("2. Zigzag\t\t--> Type 2\n");
    printf("3. Quit and Clear camera\t-> Type 3\n");
    printf("4. Reset robot system\t--> Type 4\n");
    printf("5. Clean\t\t--> Type 5\n\n");
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
    
    buf[0] = Start;
    printf("[+] Start char. %x\n", buf[0]);
    write(fdIRobot, buf, 1);

    buf[0] = FullMode;
    printf("[+] FullMode char. %x\n", buf[0]);
    write(fdIRobot, buf, 1);
    
    printf("[+] Please wait for iRobot to be stabilized..\n");
    usleep( 1000 * 1000 );
    
}

void reset()
{
    char buf[1];

    buf[0] = Reset;
    printf("[+] Send msg : %x (Reset robot) \n", buf[0]);
    write(fdIRobot, buf, 1);
}

/*
drive
- Make thread for receiveRecord
- handle real-time key input
*/
void drive()
{
    //printf("[+] gyro : %d irobot : %d\n", fdGyro, fdIRobot);
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

    buf[0] = (char)(DriveDirect);
    buf[1] = (char)((SPEED_RIGHT_STRAIGHT>>8)&0xFF);
    buf[2] = (char)(SPEED_RIGHT_STRAIGHT&0xFF);
    buf[3] = (char)((SPEED_LEFT_STRAIGHT>>8)&0xFF);
    buf[4] = (char)(SPEED_LEFT_STRAIGHT&0xFF);
    printf("[+] Send msg : (Forward straight)\n");
    write(fdIRobot, buf, 5);
}

void reverse() 
{
    char buf[5];

    buf[0] = (char)(DriveDirect);
    buf[1] = (char)(((-SPEED_RIGHT_STRAIGHT)>>8)&0xFF);
    buf[2] = (char)((-SPEED_RIGHT_STRAIGHT)&0xFF);
    buf[3] = (char)(((-SPEED_LEFT_STRAIGHT)>>8)&0xFF);
    buf[4] = (char)((-SPEED_LEFT_STRAIGHT)&0xFF);
    printf("[+] Send msg : (Backward straight)\n");
    write(fdIRobot, buf, 5);
}

void left()
{
    char buf[5];

    buf[0] = (char)(DriveDirect);
    buf[1] = (char)((SPEED_RIGHT>>8)&0xFF);
    buf[2] = (char)(SPEED_RIGHT&0xFF);
    buf[3] = (char)(((-SPEED_LEFT)>>8)&0xFF);
    buf[4] = (char)((-SPEED_LEFT)&0xFF);
    printf("[+] Send msg : (Left)\n");
    write(fdIRobot, buf, 5);
}

void right()
{
    char buf[5];
    buf[0] = (char)(DriveDirect);
    buf[1] = (char)(((-SPEED_RIGHT)>>8)&0xFF);
    buf[2] = (char)((-SPEED_RIGHT)&0xFF);
    buf[3] = (char)((SPEED_LEFT>>8)&0xFF);
    buf[4] = (char)(SPEED_LEFT&0xFF);
    printf("[+] Send msg : (Right)\n");
    write(fdIRobot, buf, 5);
}

void pauseDrive()
{
    char buf[5];
    buf[0] = (char)(DriveDirect);
    buf[1] = (char)(0);
    buf[2] = (char)(0);
    buf[3] = (char)(0);
    buf[4] = (char)(0);
    printf("[+] Send msg : (Pause Driving)\n");
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

void clean()
{
    char buf[1];

    buf[0] = Clean;
    printf("Clean char. %d\n", buf[0]);
    write(fdIRobot, buf, 1);
}

/*
quit
- stop pgr capturing
- exit
*/
void quit()
{
    fc2Error error;
    int rc;
    char buf_one[1];
    char buf_two[2];

    // stop drive
    pauseDrive();

    // pause stream
    buf_two[0] = StreamPause;
    buf_two[1] = 0;
    printf("[+] Send msg : %d%d (Pause Stream)\n", buf_two[0], buf_two[1]);
    write(fdIRobot, buf_two, 2);

    // stop OI
    buf_one[0] = Stop;
    printf("[+] Send msg : %d (Stop OI) \n", buf_one[0]);
    write(fdIRobot, buf_one, 1);

    printf("[+] iRobot working clear..\n");

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

    close(fdIRobot);
    close(fdGyro);
    printf("[+] Serial working clear..\n");

    /*
    rc = pthread_join(p_thread, (void **)&status);
    if(rc == 0)
    {
        printf("[+] Recording thread clear..\n");
    }
    */

    printf("=====   GOOD BYE   =====\n");

    exit(0);
}


/*
receiveRecord
= Thread for record info from 3 devices
- Retrieve Enc      [1]
- Retrieve Gyro     [2]
- Retrieve Image    [3]
- Record [1-3]
- Retrieval method records data on global variable
*/
void *receiveRecord(void *status)
{
    fc2Error error;
    char buf[4];
    char filePath[10];
    char writeLine[100];
    int fdTxt; // file descriptor for writing file
    int imageCnt = 0;

    // ready for writing 
    fdTxt = open("./result/result.txt", O_WRONLY | O_CREAT, 0644);
    if(fdTxt < 0)
    {
        perror("./result/pgr.txt");
        exit(0);
    }
    sprintf(writeLine, "TimeImg\tImage #\tTimeGyro\tdegree\tTimeLnc\tleftEnc\tTimeREnc\trightEnc\n");
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


    // base time set
    gettimeofday(&startTime, NULL);
    //SetTimeStamping( context, TRUE );

    printf("[+] Start Recording..\n");

    /********** Stream pause / resume ************** (METHOD 1. TOO SLOW)
    // request censor stream for two bytes (LeftCnt / RightCnt)
    buf[0] = (char)(SensorStream);
    buf[1] = (char)(2);
    buf[2] = (char)(LeftEncoderCounts);
    buf[3] = (char)(RightEncoderCounts);
    write(fdIRobot, buf, 4);

    printf("[+] Sent request SensorStream\n");
    //*************************************************/

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
        sprintf(writeLine, 
            "%.4f, %d, %.4f, %d, %.4f, %u, %.4f, %u\n", 
            pgrElapsedTime, pgrImageNumber,             // record pgr data capture
            gyroElapsedTime, gyroAngleData,             // record gyro data capture
            encLElapsedTime, encLeftCnt,                // record irobot data capture
            encRElapsedTime, encRightCnt);              // record irobot data capture
        write(fdTxt, writeLine, strlen(writeLine));
    }

}

void retrieveEncoder()
{
	char buf[2];
	unsigned short leften;
	unsigned short righten;
	int encdiff;
	struct timeval encLEndTime;
	struct timeval encREndTime;
	/********** Stream pause / resume ************** (METHOD 1. TOO SLOW)
	unsigned char data_packet[IROBOT_PACKET_SIZE_STREAM];
	//*************************************************/

	///********** Stream pause / resume ************** (METHOD 1. TOO SLOW)
	unsigned char data_packet[IROBOT_PACKET_SIZE_SENSORS];
	//*************************************************/

	/********** Stream pause / resume ************** (METHOD 1. TOO SLOW)
	//buf[0] = (char)(StreamPause);
	//buf[1] = (char)(1);
	//write(fdIRobot, buf, 2);

	// flush serial buffer before request
	tcflush(fdIRobot, TCIFLUSH);

	while(1)
	{
		// The data received should be 9 bytes
		// [1 hdr][1 nbytes][1 pktID1][2 rcvdata][1 pktID2][2 rcvdata][1 chksum]
		// [19][6][43][xxxx][44][xxxx][xxx]
		if(IROBOT_PACKET_SIZE_STREAM != read(fdIRobot, data_packet, IROBOT_PACKET_SIZE_STREAM))
		{
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
	gettimeofday(&encREndTime, NULL);

	//buf[0] = (char)(StreamPause);
	//buf[1] = (char)(0);
	//write(fdIRobot, buf, 2);

	*************************************************/

	///********** Single Request ************** (METHOD 2)
    // usleep( 100 * 1000 );
    while(1)
    {
        // get left encoder
        tcflush(fdIRobot, TCIFLUSH);
        memset (&data_packet, 0, sizeof(data_packet));
        buf[0] = Sensors;
        buf[1] = LeftEncoderCounts;
        write(fdIRobot, buf, 2);

        if(IROBOT_PACKET_SIZE_SENSORS != read(fdIRobot, data_packet, IROBOT_PACKET_SIZE_SENSORS))
        {
            usleep( 15 * 1000 );
            continue;
        }
        leften = (data_packet[0] << 8) | data_packet[1];
        gettimeofday(&encLEndTime, NULL);

        // get right encoder
        tcflush(fdIRobot, TCIFLUSH);
        memset (&data_packet, 0, sizeof(data_packet));
        buf[1] = RightEncoderCounts;
        write(fdIRobot, buf, 2);

        if(IROBOT_PACKET_SIZE_SENSORS != read(fdIRobot, data_packet, IROBOT_PACKET_SIZE_SENSORS))
        {
            usleep( 15 * 1000 );
            continue;
        }
        righten = (data_packet[0] << 8) | data_packet[1];
        gettimeofday(&encREndTime, NULL);

        break;
    }

	//*************************************************/

	encLElapsedTime = ((double)(encLEndTime.tv_sec)+(double)(encLEndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)(startTime.tv_usec)/1000000.0);
	encRElapsedTime = ((double)(encREndTime.tv_sec)+(double)(encREndTime.tv_usec)/1000000.0) - ((double)(startTime.tv_sec)+(double)(startTime.tv_usec)/1000000.0);
	encLeftCnt = leften;
	encRightCnt = righten;

	return;
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

    // flush serial buffer before request
    tcflush(fdGyro, TCIFLUSH);

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
    fc2VideoMode videomode;
    fc2FrameRate framerate;
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

    // videomode = (fc2VideoMode)FC2_VIDEOMODE_640x480Y16;
    // framerate = (fc2FrameRate)FC2_NUM_FRAMERATES;

    // error = fc2SetVideoModeAndFrameRate( context, (fc2VideoMode)FC2_VIDEOMODE_640x480Y16, (fc2FrameRate)FC2_NUM_FRAMERATES );
    // if ( error != FC2_ERROR_OK )
    // {
    //     printf( "[-] Error in fc2SetVideoModeAndFrameRate: %d\n", error );

    //     // DestoryContext
    //     error = fc2DestroyContext( context );
    //     if ( error != FC2_ERROR_OK )
    //     {
    //         printf( "Error in fc2DestroyContext: %d\n", error );
    //     }

    //     exit(0);
    // }
    return context;
}

int setGyro()
{
    int fd;
    struct termios serialio;
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(fd < 0)
    {
        printf("[-] Check the connection of Gyro censor.\n");
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

    cfmakeraw(&serialio);

    tcflush (fd, TCIFLUSH );            // flush mode mline
    tcsetattr(fd, TCSANOW, &serialio );   // port attr setting

    return fd;
}

int setIRobot()
{
	int fd;

	struct termios serialio;
	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);

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
	serialio.c_cflag &= ~CSTOPB;
	serialio.c_iflag = 0;         // no parity bit
	serialio.c_oflag = 0;
	serialio.c_lflag = 0;
	serialio.c_cc[VTIME] = 0; 
	serialio.c_cc[VMIN] = 1; 

	cfmakeraw(&serialio);

	tcflush (fd, TCIFLUSH ); 			// flush mode mline
	tcsetattr(fd, TCSANOW, &serialio );   // port attr setting

	return fd;
}

