/*
 * log.h
 *
 *  Created on: Oct 10, 2026
 *      Author: mhanzout
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_


#include "stdio.h"


#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"
#define COLOR_WHITE   "\x1b[37m"
#define COLOR_RESET   "\x1b[0m"

#define LOG_INFO(color, fmt, ...)  printf("%s" fmt COLOR_RESET "\r\n", color, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)        LOG_INFO(COLOR_RED, fmt, ##__VA_ARGS__)

extern SemaphoreHandle_t xUartMutex;


#endif /* INC_LOG_H_ */
