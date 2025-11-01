// fork-uart.c
// Test UART read/write using fork()
// Parent: reads from UART, exits on '!'
// Child : writes user input to UART, exits on 'q'

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/wait.h>

const char *portTTY = "/dev/ttyUSB0";  // modify if needed

int init_uart(void) {
    int fd = open(portTTY, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Erreur ouverture port serie");
        exit(1);
    }

    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);

    // Baudrate
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    // 8N1
    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    // Disable flow control
    SerialPortSettings.c_cflag &= ~CRTSCTS;

    // Raw input
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Non-canonical mode for UART
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Raw output
    SerialPortSettings.c_oflag &= ~OPOST;

    // VMIN=1, VTIME=0 => block until one char
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur configuration port serie");
        exit(1);
    }

    tcflush(fd, TCIFLUSH);
    return fd;
}

int main() {
    printf("./fork-uart\n");


    int fd = init_uart();

    pid_t pid = fork();

    if (pid < 0) {
        perror("fork failed");
        exit(1);
    }

    if (pid == 0) {
        // CHILD: write to UART
        char c;
        printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)...\n");
        while (1) {
            c = getchar();
            if (c == 'q' || c == '!') {
                write(fd, &c, 1);
                printf("\nFin du Fils\n");
                close(fd);
                exit(0);
            }
            write(fd, &c, 1);
        }
    }
    else {
        // PARENT: read UART
        char read_buffer[32];   // Buffer to store the data received 
        int  bytes_read = 0;    // Number of bytes read by the read() system call 
        int i = 0;
        printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série...\n");
        while (1) {
            bytes_read = read(fd, &read_buffer, 32); // Read the data 
            if (bytes_read > 0) {
                printf("processus Père: nombres d'octets recus : %d --> ", bytes_read);
                for(i=0; i<bytes_read; i++){	 // printing only the received characters
		            printf("%c", read_buffer[i]);
                    fflush(stdout);
                    if (read_buffer[i] == '!') {
                        printf("\nFin du Père\n");
                        close(fd);
                        kill(pid, SIGTERM);
                        wait(NULL);
                        exit(0);
                    }
                }
                printf("\n");
            }
        }
    }

    return 0;
}
