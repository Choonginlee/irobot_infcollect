#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>

const char		Start = 128;
const char		SafeMode = 131;
const char		Stop = 173;
const char		Sensors = 142;					// 1:    Sensor Packet ID
const char		SensorStream = 148;         // x+1: [# of packets requested] IDs of requested packets to stream
const char		StreamPause = 150;

//				iRobot Create 2 Packet IDs		//
const char		LeftEncoderCounts = 43;
const char		RightEncoderCounts = 44;

const int		IROBOT_PACKET_SIZE_STREAM = 9;	// irobot packet size
const int		IROBOT_PACKET_SIZE_SENSORS = 2;	// irobot packet size


void start(int fd);
int setIRobot();
void streamData(int fd);
void singleData(int fd);
void quit(int fd);

void main()
{
	int n;
	int fd;

	fd = setIRobot();

	while(1)
	{
		printf("1. Start 2. Stream 3. Single 4. Exit n pause\n");
		printf("Cmd : ");
		scanf("%d", &n);

		switch(n)
		{
			case 1:
				start(fd);
				break;
			case 2:
				streamData(fd);
				break;
			case 3:
				singleData(fd);
				break;
			case 4:
				quit(fd);
				break;
			default:
				break;
		}
	}
}

void start(int fd)
{
	char buf[1];
	
	buf[0] = Start;
	printf("[+] Start char. %x\n", buf[0]);
	write(fd, buf, 1);

	buf[0] = SafeMode;
	printf("[+] SafeMode char. %x\n", buf[0]);
	write(fd, buf, 1);
	
	usleep( 1000 * 1000 );
	
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

void streamData(int fd)
{
	char buf[4];
	char cnt=0;
	unsigned char data_packet[IROBOT_PACKET_SIZE_STREAM];
	
	buf[0] = (char)(SensorStream);
	buf[1] = (char)(2);
	buf[2] = (char)(LeftEncoderCounts);
	buf[3] = (char)(RightEncoderCounts);
	write(fd, buf, 4);

	while(1)
	{
		// The data received should be 9 bytes
		// [1 hdr][1 nbytes][1 pktID1][2 rcvdata][1 pktID2][2 rcvdata][1 chksum]
		// [19][6][43][xxxx][44][xxxx][xxx]
		if(IROBOT_PACKET_SIZE_STREAM != read(fd, data_packet, IROBOT_PACKET_SIZE_STREAM))
		{
			//printf("Not valid packet size\n");
			continue;
		}

		printf("[V] data : [%x] [%x%x] [%x%x%x] [%x%x%x]\n", 
			data_packet[0], data_packet[1], data_packet[2],
			data_packet[3], data_packet[4], data_packet[5],
			data_packet[6], data_packet[7], data_packet[8]);

		// 9 bytes detected. check header and bytes
		if(data_packet[0] == 19 && data_packet[1] == 6)
		{
			// check packet ID 1
			if(data_packet[2] != 43 || data_packet[5] != 44)
			{
				continue;
			}
			//leften = (data_packet[3] << 8) | data_packet[4];
			//righten = (data_packet[6] << 8) | data_packet[7];

			printf("[%d] Data received\n", ++cnt);
		}
	}
}

void singleData(int fd)
{
	char buf[2];
	char cnt=0;
	unsigned char data_packet[IROBOT_PACKET_SIZE_SENSORS];

	while(1)
	{
		cnt++;

		tcflush(fd, TCIFLUSH);

		buf[0] = Sensors;
		buf[1] = LeftEncoderCounts;
		write(fd, buf, 2);
		printf("Sent %d request of left encoder\n", cnt);

		while(1)
		{
			if(IROBOT_PACKET_SIZE_SENSORS != read(fd, data_packet, IROBOT_PACKET_SIZE_SENSORS))
			{
				//printf("Not valid packet size\n");
				sleep( 15 * 1000 );
				write(fd, buf, 2);
				continue;
			}
			//leften = (data_packet[0] << 8) | data_packet[1];
			break;
		}

		printf("[V] data : [%x%x] \n", data_packet[0], data_packet[1]);

		tcflush(fd, TCIFLUSH);
		
		buf[1] = RightEncoderCounts;
		write(fd, buf, 2);
		printf("Sent %d request of right encoder\n", cnt);

		while(1)
		{
			if(IROBOT_PACKET_SIZE_SENSORS != read(fd, data_packet, IROBOT_PACKET_SIZE_SENSORS))
			{
				//printf("Not valid packet size\n");
				sleep( 15 * 1000 );
				write(fd, buf, 2);
				continue;
			}
			//righten = (data_packet[0] << 8) | data_packet[1];
			break;
		}

		printf("[%x%x]\n", data_packet[0], data_packet[1]);
		printf("[%d] Data received\n", cnt);
	}
}

void quit(int fd)
{
	char buf1[2];
	char buf2[1];

 	// pause stream
	buf1[0] = StreamPause;
	buf1[1] = 0;
	printf("[+] Send msg : %d%d (Pause Stream)\n", buf1[0], buf1[1]);
	write(fd, buf1, 2);

	// stop OI
	buf2[0] = Stop;
	printf("[+] Send msg : %d (Stop OI) \n", buf2[0]);
	write(fd, buf2, 1);

	close(fd);

	exit(0);
}