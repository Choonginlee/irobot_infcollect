#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <stdlib.h>

//Define constants
const char COMM_PORT[] = "/dev/ttyUSB1";
const int PACKET_SIZE = 8;
const int SAMPLES = 1000;

//File descriptor for serial connection
int fd;

void ccr1050_getvalue();

int main()
{
	int i;

	struct termios serialio_xg;
	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);
	
	if(fd < 0)
	{
		perror("/dev/ttyUSB1");
		exit(-1);
	}

	memset( &serialio_xg, 0, sizeof(serialio_xg) );
	serialio_xg.c_cflag = B115200;   // baud - 115200 
	serialio_xg.c_cflag |= CS8;      // data bit - 8bit 
	serialio_xg.c_cflag |= CLOCAL;   // use local comm port 
	serialio_xg.c_cflag |= CREAD;    // read & write
	serialio_xg.c_iflag = 0;         // no parity bit
	serialio_xg.c_oflag = 0;
	serialio_xg.c_lflag = 0;
	serialio_xg.c_cc[VTIME] = 0; 
	serialio_xg.c_cc[VMIN] = 1; 
	tcflush (fd, TCIFLUSH ); 			// flush mode mline
	tcsetattr(fd, TCSANOW, &serialio_xg );   // port attr setting

	// Get, parse and display data
	for(i=0; i<SAMPLES ;i++)
		ccr1050_getvalue();

	close(fd);

}

// Get a data packet and parse it
void ccr1050_getvalue()
{
	short header;
	short rate_int;
	short angle_int;
	float rate_float;
	float angle_float;
	short check_sum;
	unsigned char data_packet[PACKET_SIZE];

	if(PACKET_SIZE != read(fd, data_packet, PACKET_SIZE))
		return;

	// Verify data packet header 
	memcpy(&header, data_packet, sizeof(short));
	if(header != (short)0xFFFF)
	{
		printf("Header error !!!\n");
		return;
	}

	// Copy values from data string 
	memcpy(&rate_int, data_packet+2, sizeof(short));
	memcpy(&angle_int, data_packet+4, sizeof(short));
	memcpy(&check_sum, data_packet+6, sizeof(short));

	// Verify checksum
	if(check_sum != (short)(0xFFFF + rate_int + angle_int))
	{
		printf("Checksum error!!\n");
		return;
	}

	// Apply scale factors
	rate_float = rate_int/100.0;
 	angle_float = angle_int/100.0;
	
	printf("rate_float: %f [deg/sec]\t angle_float : %f [deg]\n", rate_float, angle_float);

	return;
}