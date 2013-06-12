#include <termios.h>
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringSerial.h>
#include <semsSensorCom.h>

#define PIN_DIR   RPI_GPIO_P1_11  //rs-485 Transmit/Recive Pin (High transmit/Low Recive)
#define PIN_EN_A  RPI_GPIO_P1_16  //Multiplexer Pin Enable - Bottom Row
#define PIN_EN_B  RPI_GPIO_P1_18  //Multiplexer Pin Enable - Top Row
#define PIN_S0    RPI_GPIO_P1_13  //Multiplexer Channel/Port Selection - Low Bit
#define PIN_S1    RPI_GPIO_P1_15  //Multiplexer Channel/Port Selection - High Bit

#define SENSOR_TYPE_TEMP 1
#define SENSOR_TYPE_TH   2
#define SENSOR_TYPE_MAX  SENSOR_TYPE_TH

#define RD_BUFF_SIZE 25

#define DELAY_READ 25 //milliseconds
#define DELAY_MULTI 10

#define RETRYS 5  //Max number of retries when reciving corrupt data


int rd,i;
char rd_buff[RD_BUFF_SIZE];
char location[PORTS_TOTAL][LOCATION_BUFF_SIZE];

struct termios settings;


void semsComDisableMultiplex() {
        bcm2835_gpio_write(PIN_EN_B,LOW);
        bcm2835_gpio_write(PIN_EN_A,LOW);
}

void semsSelectMultiplex(unsigned int port) {
        char bitmask = 0x00; //High nibble is which Multiplexer, Low nibble is which Port
        if (port <= PORTMAX) {
	semsComDisableMultiplex();
            if (port < 4) {
                bitmask = 0x10; //Select the top row multiplexer
                bitmask |= port;
            } else {
                bitmask = 0x20; //select the bottom row multiplexer
                bitmask |= (port-4);
            }
            bcm2835_gpio_write(PIN_S0,(bitmask & 0x01));
            bcm2835_gpio_write(PIN_S1,(bitmask & 0x02)>>1);
            bcm2835_gpio_write(PIN_EN_A,(bitmask & 0x10)>>4);
            bcm2835_gpio_write(PIN_EN_B,(bitmask & 0x20)>>5);
	   delay(3);
	}
}

void semsSendCMD(int fd, unsigned char cmd) {
         bcm2835_gpio_write(PIN_DIR, HIGH);  //Transmit mode
         serialPutchar (fd, cmd);
         //tcdrain(sfd); // takes to long to return
         delay(1);       //wait for writing to finish
         bcm2835_gpio_write(PIN_DIR, LOW);  //Recieve Mode
}


float semsGetTemp(int fd) {
    float temp=0;
	int count = 0;
	while (temp == 0 && count < 5){
		semsSendCMD(fd,'T');
		delay(DELAY_READ);
		for(i=0;serialDataAvail(fd) && i < RD_BUFF_SIZE-1; i++)
			rd_buff[i] = serialGetchar(fd);
		rd_buff[i] = '\0';
		if (i>0)
			temp = atof(rd_buff); //Return 0.0 if string has no valid number
		count++;
	}
	return temp;
}

float semsGetTempRaw(int fd) {
	float temp = 0;
	char *buff = (char *)&temp;
	unsigned char chksum, mysum;
	int i, trys=0;
	serialFlush (fd);
	semsSendCMD(fd,'T');
	delay(DELAY_READ);
	if (serialDataAvail(fd) >= 5) {
	printf("Data Availibe\n");
	   do {
	      mysum = 0;
	      for (i=0;i<4;i++){
                *(buff+i) = serialGetchar(fd);
                 mysum -= *(buff+i);
	      }
	      chksum = serialGetchar(fd);
	      printf("%d - %d",mysum,chksum);
	   } while(mysum != chksum && trys++ < RETRYS);
	   if (mysum != chksum) 
	      return -275.15;
	    return temp;
	}
	return -275.15;
}


short int semsGetRHRaw(int fd) {
        short int rh = 0;
        char *buff = (char *)&rh;
        unsigned char chksum, mysum;
        int i, trys=0;
        serialFlush (fd);
        semsSendCMD(fd,'H');
        delay(DELAY_READ);
        if (serialDataAvail(fd) >= 3) {
           do {
              mysum = 0;
              for (i=0;i<2;i++){
                *(buff+i) = serialGetchar(fd);
                 mysum -= *(buff+i);
              }
              chksum = serialGetchar(fd);
           } while(mysum != chksum && trys++ < RETRYS);
           if (mysum != chksum)
              return -100;
            return rh;
        }
        return -100;
}

short int semsGetTypeRaw(int fd) {
        short int type = 0;
        char *buff = (char *)&type;
        unsigned char chksum, mysum;
        int i,trys=0;
        serialFlush (fd);
        semsSendCMD(fd,'S');
        delay(DELAY_READ);
        if (serialDataAvail(fd) >= 3) {
           do {
              mysum = 0;
              for (i=0;i<2;i++){
                *(buff+i) = serialGetchar(fd);
                 mysum -= *(buff+i);
              }
              chksum = serialGetchar(fd);
           } while(mysum != chksum && trys < RETRYS);
           if (mysum != chksum)
              return -1;
            return type;
        }
        return -1;
}


float semsGetRH(int fd) {
	float rh = 0;
	int count = 0;
	while (rh == 0 && count < 5){
        semsSendCMD(fd,'H');
        delay(DELAY_READ);
        for(i=0;serialDataAvail(fd) && i < RD_BUFF_SIZE-1; i++)
                rd_buff[i] = serialGetchar(fd);
        rd_buff[i] = '\0';
        if (i>0)
			rh = atof(rd_buff);
		count++;
	}
	if (rh > 100) {
		return 100;
	} else if (rh < 0) {
		return 0;
	}
	return rh;
}

float semsGetType(int fd) {
	int count = 0;
	int type = -1;
	serialFlush(fd);
	while ((type <= 0 || type > SENSOR_TYPE_MAX) && count < 5) {
        semsSendCMD(fd,'S');
        delay(DELAY_READ);
        for(i=0;serialDataAvail(fd) && i < RD_BUFF_SIZE-1; i++){
                rd_buff[i] = serialGetchar(fd);
		printf("%c",rd_buff[i]);
	}
        rd_buff[i] = '\0';
        if (i>0)
	  //printf("%s\n",rd_buff);
          //type = atof(rd_buff);
		type = 2;
		count++;
	}
	if (type == 0) 
		return -1;
	return type;
}

int semsInitComs() {
	int sfd;
	sfd = serialOpen ("/dev/ttyAMA0",9600); //Open the serial port on the Raspberry Pi at 9600 baud
											//ttyAMA0 is the file handle for BCM2835's native UART 
        if (sfd < 0) {
			printf("Error Opening Serial");
            return -1;
        }
		/* Theses are the settings that should be set to
		   the temrinal (serial port), the serialOpen command
		   from the wiringSerial librarry takes care of this
		   but in case We wish to not use wiringSerial these 
		   are the settings that should be set at minimal */
        /* 
        tcgetattr(sfd, &settings);
        cfsetospeed(&settings, B9600); // baud rate
        settings.c_cflag &= ~PARENB; // no parity
        settings.c_cflag &= ~CSTOPB; // 1 stop bit
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= CS8 | CLOCAL; // 8 bits
        settings.c_lflag &=  ~(ICANON | ECHO | ECHOE | ISIG); //Raw Input
        settings.c_oflag &= ~OPOST; // raw output
        tcsetattr(sfd, TCSANOW, &settings); // apply the settings
        tcflush(sfd, TCOFLUSH);
        */

        if (!bcm2835_init()){
			printf("Failed to init the GPIO lines\n");
            return -1;
		}
        //Set all used pins to output mode
        bcm2835_gpio_fsel(PIN_DIR,  BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(PIN_EN_A, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(PIN_EN_B, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(PIN_S0,   BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(PIN_S1,   BCM2835_GPIO_FSEL_OUTP);
	return sfd;
}

void semsUpdateSensorInfo(int fd,Sensor* s) {	
	int result;
	/* While the result is out of range, which could mean bad data 
	   or an unknown sensor, then ask for sensor information again.
	   If we ask 3 times and still no good data, assume sensor
	   is not present. */
	result = semsGetType(fd);
	printf("Sensor Type: %d\n",result);
	if (result > 0 && result <= SENSOR_TYPE_MAX) {
		s->type = (int)result;
		s->status = 1;
		return;
	} else if (result == -1) {       //-1 means no data return from the command, assume no sensor
		s->type = SENSOR_TYPE_TEMP;  //Sensor type needs to be valid for SNMP, so temp for default
		s->status = 0;               // Disconected Sensor
		return;
	}
	s->type = (int)result;			//Unknown Sensor Type, set it to w/e it returned...
	s->status = 0;					//But dissable the sensor.
	return;
}

void semsUpdateSensor(int fd,Sensor* s) {
	printf("Sensor Port %d \n",s->port);
	semsSelectMultiplex(s->port-1);
	semsUpdateSensorInfo(fd,s);
	float temp;
	float rh;
	if (s->status == 1) {  //Is the sensor an active one?
		if (s->type == SENSOR_TYPE_TEMP || s->type == SENSOR_TYPE_TH) {
			temp = semsGetTempRaw(fd);
			temp += 0.005;
			temp *= 100;
			s->temp = (int)temp;

			if (s->type == SENSOR_TYPE_TH) {
				rh = semsGetRHRaw(fd);
				rh = rh>100 ? 100 : rh;
				rh = rh<0 ? 0 : rh;
				s->rh = (unsigned int)(rh);
			}
		}
	}
    return;
}

void Sensor_init(int fd, unsigned int port, Sensor * s) {
	if (port <= PORTMAX) {
		s->temp =0;
		s->rh = 0;
		s->location = location[port];
		s->port = port+1;
		semsUpdateSensor(fd,s);
		return;
	}
	s->status = 0;
	return;
}
