#define NANOSENSE_VERSION 1.1
#define LIBNANOSENSE_VERSION 0.5

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define GET_ALL_CONTEXTS 0xff

typedef struct context_def_t
{
	unsigned char context_type;
	char context_descr[16];
} context_def_t;


enum libNS_pkt_types { ADVERTISE = 0x00, PUBLISH, SUBSCRIBE };

int libNS_create_adv_msg(unsigned char *buf, unsigned short int ttl, unsigned char contexts, struct context_def_t *context_def);

int libNS_create_pub_msg(unsigned char *buf, unsigned short int sample_rate, unsigned char contexts, unsigned char samples, unsigned char *context_types, unsigned short int **data);

int libNS_create_sub_msg(unsigned char *buf, unsigned short int ttl, unsigned short int sample_interval, unsigned char contexts, unsigned char *context_types, unsigned char *utility_values);

int libNS_parse_adv_msg(unsigned char *buf, unsigned short int len, unsigned short int *ttl, unsigned char *contexts, struct context_def_t *context_def);

int libNS_parse_pub_msg(unsigned char *buf, unsigned short int *sample_int, unsigned char *contexts, unsigned char *samples, unsigned char *context_types, unsigned short int **data);

int libNS_parse_sub_msg(unsigned char *buf, unsigned short int *ttl, unsigned short int *sample_int, unsigned char *contexts, unsigned char *context_types, unsigned char *utility_values);
