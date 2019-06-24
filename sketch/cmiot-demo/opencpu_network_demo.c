/*
   opencpu_network.c
   created by xgl,2019/1/22
*/

#include "m5311_opencpu.h"

char test_msg[] = "network test\n";
char test_rx_buf[100];

//dns功能回调函数
void test_cmdns_cb(unsigned char *ip)
{
	
    opencpu_printf ("opencpu dns:%s\n",ip);	
}

//ping功能的回调函数，用户在这个回调函数里处理结果
void test_ping_cb(ping_result_type_t type, void* p)
{
    switch(type)
	{
        case PING_TOTAL_RESULT: 
		{
		    if (p) {
				ping_result_t* ping_result = (ping_result_t*)p;
		       opencpu_printf ("\r\n--- %s ping statistics ---\r\n"
		            "%d packets transmitted, %d received, %d%% packet loss\r\n"
		            "rtt min/avg/max = %d/%d/%d", inet_ntoa(ping_result->ping_target),
		            (int)ping_result->total_num, (int)ping_result->recv_num,  (int)((ping_result->lost_num * 100)/ping_result->total_num),
		            ping_result->min_time,ping_result->avg_time,ping_result->max_time);

		    } else {
		        break;//response.pdata = "+Ping: finish, no result!";
		    }
		}
		break;
		case PING_PACKET_RESULT:
		{
			ping_packet_result_t* ping_result = (ping_packet_result_t*)p;
			if (p == NULL)
			{
                break;
            }
			else if (ping_result->is_timeout == true)
			{
                opencpu_printf ("+PINGERR: 1");
				break;
            }
			else
			{
                if (ping_result->is_ipv4 == true)
				{
					opencpu_printf ("+PING: %d.%d.%d.%d,%d,%d",ping_result->ip_address[0],
                           ping_result->ip_address[1], ping_result->ip_address[2], ping_result->ip_address[3],
						   ping_result->ttl,ping_result->rtt);
                }
				else
				{
                   opencpu_printf ("+PING: %x:%x:%x:%x:%x:%x:%x:%x,%d,%d",ping_result->ip_address[0],
                         ping_result->ip_address[1],ping_result->ip_address[2], ping_result->ip_address[3],
                         ping_result->ip_address[4],ping_result->ip_address[5],
                         ping_result->ip_address[6],ping_result->ip_address[7],
						 ping_result->ttl,ping_result->rtt);
                }
            }
		}
		break;
	}
}

//发起dns测试的函数
void test_dns()
{
	opencpu_printf ( "result:%d\n",opencpu_get_host_by_name("www.baidu.com",0,test_cmdns_cb));
}

//发起ping功能的测试函数
void test_ping()
{
	opencpu_ping("180.97.33.108",3,64,test_ping_cb);
	opencpu_printf ("ping waiting...\n");
}

struct in_addr test_remote_addr;
uint16_t test_remote_port;

void udp_test()
{
    struct sockaddr_in server_addr;  
    int sock_fd; 
	struct addrinfo dd;
    struct sockaddr_in from;
    int data_len;
	inet_aton("47.93.217.230", &test_remote_addr);  
    test_remote_port = 2019;	
	socklen_t fromlen = sizeof(struct sockaddr_in);
	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sock_fd == -1)
	{
		opencpu_printf ( "socket create error\n");
		return;
	}
	memset(&server_addr, 0, sizeof(server_addr)); 
	server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = test_remote_addr.s_addr;  
    server_addr.sin_port = htons(test_remote_port);  

	connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)); 
	opencpu_printf ("data sending\n");
	send(sock_fd, (char *)test_msg, sizeof(test_msg), 0);
	 memset(test_rx_buf,0,100);
	/*data_len = recvfrom(sock_fd, test_rx_buf,
                    100, MSG_TRUNC | MSG_DONTWAIT, (struct sockaddr*)&from, &fromlen);*/
    opencpu_printf ( "waiting...\n");
    data_len = recvfrom(sock_fd, test_rx_buf,
                    100, MSG_TRUNC, (struct sockaddr*)&from, &fromlen);
    opencpu_printf("waiting end\n");                   
	if(data_len >0)
		{
		    opencpu_printf ( "get:%s\n",test_rx_buf);
			opencpu_printf ( "len:%d\n",data_len);			 
		}
		
	close(sock_fd);	
}

void tcp_test()
{
	struct sockaddr_in server_addr;  
    int sock_fd;  
    int data_len;
	inet_aton("47.93.217.230", &test_remote_addr);  
    test_remote_port = 2019;
	sock_fd = socket(AF_INET, SOCK_STREAM, 0);  
	if (sock_fd == -1) {  
        opencpu_printf ( "socket create error\n");
		 return;
    }  
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = test_remote_addr.s_addr;
    server_addr.sin_port = htons(test_remote_port);
    if(connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)))
	{
		opencpu_printf ( "tcp connect error\n");
		return;
	}
	    opencpu_printf ( "tcp data sending\n");
	    send(sock_fd, (char *)test_msg, sizeof(test_msg), 0);
	    memset(test_rx_buf,0,100);
		/*data_len = recvfrom(sock_fd, test_rx_buf,
                    100, MSG_TRUNC | MSG_DONTWAIT, (struct sockaddr*)&from, &fromlen);*/
        opencpu_printf ( "tcp waiting...\n");
        data_len = recv(sock_fd, test_rx_buf,100, MSG_TRUNC);
        opencpu_printf ( "waiting end\n");
                    
	    if(data_len >0)
		{
		    opencpu_printf ("get:%s\n",test_rx_buf);
			opencpu_printf ( "len:%d\n",data_len);			 
		}
		opencpu_printf ( "tcp waiting..1.\n");
        data_len = recv(sock_fd, test_rx_buf,100, MSG_TRUNC);
        opencpu_printf ( "waiting end1\n");
	close(sock_fd);
	
}
