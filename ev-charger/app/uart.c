#include "uart.h"

int open_serial_port(const char *port) {
    // 打开串口设备（阻塞模式）
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Error opening serial port");
        return -1;
    }

    // 获取当前串口配置
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }

    // 设置波特率为115200
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // 配置数据位、停止位和校验位
    options.c_cflag &= ~CSIZE;  // 清除数据位掩码
    options.c_cflag |= CS8;     // 8位数据位
    options.c_cflag &= ~PARENB; // 无校验位
    options.c_cflag &= ~CSTOPB; // 1位停止位

    // 启用接收和本地模式
    options.c_cflag |= (CLOCAL | CREAD);

    // 设置原始输入模式（非规范模式）
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 禁用特殊输出处理
    options.c_oflag &= ~OPOST;

    // 设置超时：立即返回（VMIN=0, VTIME=0）
    //options.c_cc[VMIN] = 0;
    //options.c_cc[VTIME] = 0;

    // 修改 c_cc 配置，启用阻塞读
    options.c_cc[VMIN] = 1;   // 至少读取1个字符才返回
    options.c_cc[VTIME] = 10; // 等待超时：1秒（10 * 0.1秒）

    // 应用配置
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }

    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);
    return fd;
}

int write_to_serial(int fd, const char *data, size_t length) {
    ssize_t bytes_written = write(fd, data, length);
    if (bytes_written < 0) {
        perror("Error writing to serial port");
    }
    return bytes_written;
}

int read_from_serial(int fd, char *buffer, size_t buffer_size) {
    ssize_t bytes_read = read(fd, buffer, buffer_size - 1);
    if (bytes_read < 0) {
        perror("Error reading from serial port");
        return -1;
    }
    buffer[bytes_read] = '\0'; // 添加字符串结束符
    return bytes_read;
}

void close_serial_port(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}
