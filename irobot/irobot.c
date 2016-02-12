#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
//#include <time.h>

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

const int          C_PACKET_SIZE = 2;			// Censor packet size
 
int speed_left =  200;
int speed_right = 200;

void showInstruction();
int rcv_command();
char rcv_direction();

void start(int fd);
void quit(int fd);		// stop OI
void clean(int fd);
void drive(int fd);
void forward(int fd);
void forward_d(int fd, int distance);
void reverse(int fd);
void reverse_d(int fd, int distance);
void left(int fd);
void left_a(int fd, int angle);
void right(int fd);
void right_a(int fd, int angle);
void stop(int fd);		// stop driving
void zigzag(int fd, int length, int width, int req_num_length);

void *receive_censor(void *fd);

void main()
{
	int fd;		// Serial handler of irobot
	struct termios serialio;
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
	
	if(fd < 0)
	{
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

	showInstruction();

	while(1)
	{
		//listen command
		int cmd_rcvd;
		cmd_rcvd = rcv_command();

		switch(cmd_rcvd)
		{
			case 1:
				start(fd);
				break;
			case 2:
				clean(fd);
				break;
			case 3:
				drive(fd);
				showInstruction();
				break;
			case 4:
				zigzag(fd, 1000, 500, 3);
				stop(fd);
				break;
			case 5:
				quit(fd);
				break;
			default:
				break;
		}
	}
	
	return;
}

void showInstruction()
{
	printf("===============================\n");
	printf("1. To start, Type 1\n");
	printf("2. To clean, Type 2\n");
	printf("3. To move around, Type 3\n");
	printf("4. To zigzag, Type 4\n");
	printf("5. To quit, Type 5\n");
	printf("===============================\n");
}

int rcv_command()
{
	int command;
	printf("Enter command : ");
	scanf("%d", &command);
	return command;
}

char rcv_direction()
{
	char dir;
	printf("Enter direction : ");
	scanf("%c", &dir);
	return dir;
}

// Start  - send irobot start and safe mode
// It tries to listen left / right encoder
void start(int fd)
{
	char buf[10];

	pthread_t p_thread[1];
	int thr_id;
	int status;

	sprintf(buf, "%c", Start);
	printf("[+] Send msg : %s\n", buf);
	write(fd, buf, 1);

	sprintf(buf, "%c", SafeMode);
	printf("[+] Send msg : %s\n", buf);
	write(fd, buf, 1);

	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)((speed_right>>8)&0xFF), (char)(speed_right&0xFF), 
			(char)(((-speed_left)>>8)&0xFF), (char)((-speed_left)&0xFF));

	//printf("[+] Send msg : %s (Left)\n", buf);
	write(fd, buf, 5);

	// Listen to encoders by creating a thread
	thr_id = pthread_create(&p_thread[0], NULL, receive_censor, (void *)&fd);
	if(thr_id < 0)
	{
		perror("Thread create error : ");
		exit(0);
	}

	//pthread_join(p_thread[0], (void **)&status);
}

void quit(int fd)
{
	char buf[10];

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
		(char)((speed_right>>8)&0xFF), (char)(speed_right&0xFF),
		(char)((speed_left>>8)&0xFF), (char)(speed_left&0xFF));

	printf("[+] Send msg : %s (Forward straight)\n", buf);
	write(fd, buf, 5);
}

void forward_d(int fd, int distance) // forward for distnace
{
	char buf[5];
	int waittime = 0;

	sprintf(buf, "%c%c%c%c%c",
		DriveDirect,
		(char)((speed_right>>8)&0xFF), (char)(speed_right&0xFF),
		(char)((speed_left>>8)&0xFF), (char)(speed_left&0xFF));

	printf("[+] Send msg : %s (Forward for %d mm)\n", buf, distance);
	write(fd, buf, 5);

	// Time = Distance (mm) / Velocity (mm)
	waittime = (int)(distance / speed_right);
	sleep(waittime);
}

void reverse(int fd) // backward straight
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-speed_right)>>8)&0xFF), (char)((-speed_right)&0xFF), 
			(char)(((-speed_left)>>8)&0xFF), (char)((-speed_left)&0xFF));

	printf("[+] Send msg : %s (Backward straight)\n", buf);
	write(fd, buf, 5);
}

void reverse_d(int fd, int distance) // backward for distance 
{
	char buf[5];
	int waittime = 0;

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect,
			(char)(((-speed_right)>>8)&0xFF), (char)((-speed_right)&0xFF), 
			(char)(((-speed_left)>>8)&0xFF), (char)((-speed_left)&0xFF));

	printf("[+] Send msg : %s (Backward for %d mm)\n", buf, distance);
	write(fd, buf, 5);

	// Time = Distance (mm) / Velocity (mm)
	waittime = (int)(distance / speed_right);
	sleep(waittime);
}

void left(int fd)
{
	char buf[5];

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)((speed_right>>8)&0xFF), (char)(speed_right&0xFF), 
			(char)(((-speed_left)>>8)&0xFF), (char)((-speed_left)&0xFF));

	printf("[+] Send msg : %s (Left)\n", buf);
	write(fd, buf, 5);
}

void left_a(int fd, int angle)
{
	char buf[5];
	int waittime = 0;

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)((speed_right>>8)&0xFF), (char)(speed_right&0xFF), 
			(char)(((-speed_left)>>8)&0xFF), (char)((-speed_left)&0xFF));

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
			(char)(((-speed_right)>>8)&0xFF), (char)((-speed_right)&0xFF), 
			(char)((speed_left>>8)&0xFF), (char)(speed_left&0xFF));

	printf("[+] Send msg : %s (Right)\n", buf);
	write(fd, buf, 5);
}

void right_a(int fd, int angle)
{
	char buf[5];
	int waittime = 0;

		sprintf(buf, "%c%c%c%c%c", 
			DriveDirect, 
			(char)(((-speed_right)>>8)&0xFF), (char)((-speed_right)&0xFF), 
			(char)((speed_left>>8)&0xFF), (char)(speed_left&0xFF));

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

	printf("[+] Stop Driving...\n");

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
		forward_d(fd, length);
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
			left_a(fd, 90);
			forward_d(fd, width);
			left_a(fd, 90);
		}
		// if num_length is even, turn right -> go 'width' -> turn right
		else
		{
			right_a(fd, 90);
			forward_d(fd, width);
			right_a(fd, 90);
		}
	}
}

// Interrupt Routine to read in serial sensor data packets - Encoder sensor only
void *receive_censor(void *v_fd)
{
	int fd = *(int *)v_fd;
	char buf[5];
	char start_character;
	unsigned char data_packet[C_PACKET_SIZE];

	while(1)
	{
		// request censor left and right
		sprintf(buf, "%c%c", Sensors, LeftEncoderCounts);
		write(fd, buf, 2);

		usleep( 15 * 1000 );

		//sprintf(buf, "%c%c", Sensors, RightEncoderCounts);
		//write(fd, buf, 2);

		//usleep( 15 * 1000 );

		if(C_PACKET_SIZE != read(fd, data_packet, C_PACKET_SIZE))
		{
			//printf("Not Valid Packet size\n");
			//printf("%s\n", data_packet);
			continue;
		}

		printf("2 bytes Received! %d %d\n", data_packet[0], data_packet[1]);
		usleep(10000);
	}

}