#ifndef _SEMSSENSORCOM_H_
#define _SEMSSENSORCOM_H_

#define PORT1 0x00
#define PORT2 0x01
#define PORT3 0x02
#define PORT4 0x03
#define PORT5 0x04
#define PORT6 0x05
#define PORT7 0x06
#define PORT8 0x07
#define PORTMAX PORT8
#define PORTS_TOTAL PORTMAX + 1
#define LOCATION_BUFF_SIZE 255

typedef struct SemsSensor {
    unsigned int port;
    int status;
    int type;
    char* location;
    int temp;
    unsigned int rh;
} Sensor;

void Sensor_init(int, unsigned int, Sensor*);
int semsInitComs(void);
void semsUpdateSensor(int,Sensor*);


#endif

