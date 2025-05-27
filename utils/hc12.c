#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>      // open()
#include <unistd.h>     // write(), close()
#include <termios.h>    // termios
#include <time.h>       // time(), strftime()

int main() {
    int serial_port = open("/dev/ttyS4", O_RDWR);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting termios attributes");
        close(serial_port);
        return 1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);  
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            
    tty.c_cflag &= ~PARENB;         
    tty.c_cflag &= ~CSTOPB;       
    tty.c_cflag &= ~CRTSCTS;         

    tty.c_lflag = 0;                  
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;             

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error setting termios attributes");
        close(serial_port);
        return 1;
    }


    char timestamp[64];
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);

 
    char message[128];
    snprintf(message, sizeof(message), "mission accomplished at %s", timestamp);

    write(serial_port, message, strlen(message));


    close(serial_port);

    printf("Sent: %s\n", message);

    return 0;
}