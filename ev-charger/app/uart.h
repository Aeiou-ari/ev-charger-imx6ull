#ifndef __UART_H__
#define __UART_H__

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>

// 打开串口设备并配置为115200波特率、8位数据位、无校验、1位停止位
int open_serial_port(const char *port);

// 写入数据到串口
int write_to_serial(int fd, const char *data, size_t length);

// 从串口读取数据
int read_from_serial(int fd, char *buffer, size_t buffer_size);

// 关闭串口设备
void close_serial_port(int fd);

#endif // TERMIOS_H
