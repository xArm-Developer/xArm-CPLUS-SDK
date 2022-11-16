/**
 * Software License Agreement (MIT License)
 * 
 * @copyright Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Zhang <jimy92@163.com>
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef OS_NETWORK_H_
#define OS_NETWORK_H_

#include "xarm/core/common/data_type.h"

int socket_init(char *local_ip, int port, int is_server);
int socket_send_data(int client_fp, unsigned char *data, int len);
int socket_connect_server(int *socket, char server_ip[], int server_port);

#endif // OS_NETWORK_H_
