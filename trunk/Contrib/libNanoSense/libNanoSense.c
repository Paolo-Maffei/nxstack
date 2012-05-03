#include "libNanoSense.h"

int libNS_create_adv_msg(unsigned char *buf, unsigned short int ttl, unsigned char contexts, struct context_def_t *context_def)
{
	int i;
	int index = 4;
	
	/*	Set the packet type. */
	buf[0] = ADVERTISE;


	/*	Set the time-to-live value. */
	buf[1] = (ttl >> 8);
	buf[2] = (ttl & 0xff);
	
	/*	Set the number of contexts. */
	buf[3] = contexts;
	
	for(i=1;i<=contexts;i++)
	{
		buf[index] = context_def[i].context_type;
		sprintf((char *)&(buf[index]), "%s\n", context_def[i].context_descr);
		index += (strlen(context_def[i].context_descr) + 1);
	}
	
	return(index);
}


int libNS_create_pub_msg(unsigned char *buf, unsigned short int sample_rate, unsigned char contexts, unsigned char samples, unsigned char *context_types, unsigned short int **data)
{
	int i, j;
	int index=4;
	
	buf[0] = PUBLISH;
	buf[1] = sample_rate;
	buf[2] = contexts;
	buf[3] = samples;
	
	for(i=1;i<=contexts;i++)
	{
		buf[index] = context_types[i];
		index++;
	}
	
	for(i=0;i<samples;i++)
	{
		for(j=0;j<contexts;j++)
		{
			buf[index] = (data[i][j] >> 8);
			buf[index+1] = (data[i][j] & 0xff);
			
			index += 2;
		}
	}
	
	return(index);
}


int libNS_create_sub_msg(unsigned char *buf, unsigned short int ttl, unsigned short int sample_interval, unsigned char contexts, unsigned char *context_types, unsigned char *utility_values)
{
	int i;
	int index = 6;
	
	buf[0] = SUBSCRIBE;
	buf[1] = (ttl >> 8);
	buf[2] = (ttl & 0xff);
	buf[3] = (sample_interval >> 8);
	buf[4] = (sample_interval & 0xff);
	buf[5] = contexts;
	
	for(i=0;i<contexts;i++)
	{
		buf[index] = context_types[i];
		buf[index+1] = utility_values[i];
	
		index += 2;
	}
	
	return(index);
}		

int libNS_parse_adv_msg(unsigned char *buf, unsigned short int len, unsigned short int *ttl, unsigned char *contexts, struct context_def_t *context_def)
{
	int i;
	int index=5;
	
	/*	Just in case, check the message type. */
	
	if(buf[0] != ADVERTISE)
	{
		printf("Trying to parse message type 0x%2.2x as Advertisement message (0x00).\n", buf[0]);
		return(-1);
	}
	
	*ttl = (unsigned short int)((buf[1] << 8) | (buf[2]));
	
	*contexts = buf[3];
	
	for(i=0; i<*contexts; i++)
	{
		context_def[i].context_type = buf[4+i];
		strcpy(context_def[i].context_descr, (char *)&(buf[index]));

		index += (strlen(context_def[i].context_descr) + 1); // Note that strlen() does not calculate the terminating NUL character into the length
	}
	
	return(1);
}


int libNS_parse_pub_msg(unsigned char *buf, unsigned short int *sample_int, unsigned char *contexts, unsigned char *samples, unsigned char *context_types, unsigned short int **data)
{
	int i, j;
	int index;
	
	/*	Just in case, check the message type. */
	
	if(buf[0] != PUBLISH)
	{
		printf("Trying to parse message type 0x%2.2x as Publish message (0x01).\n", buf[0]);
		return(-1);
	}
	
	*sample_int = (unsigned short int)((buf[1] << 8) | buf[2]);
	
	*contexts = buf[3];

	*samples = buf[4];

	index = 5 + *contexts;
	
	for(i=0; i<buf[3];i++)
	{
		context_types[i] = buf[5+i];
	}

	for(i=0; i<buf[4];i++)
	{
		for(j=0;j<buf[3];j++)
		{
			data[i][j] = ((buf[index]<<8) | buf[index+1]);
			index += 2;
		}
	}
	
	return(1);
}
		
int libNS_parse_sub_msg(unsigned char *buf, unsigned short int *ttl, unsigned short int *sample_int, unsigned char *contexts, unsigned char *context_types, unsigned char *utility_values)
{
	int i;

	/*	Just in case, check the message type. */
	
	if(buf[0] != SUBSCRIBE)
	{
		printf("Trying to parse message type 0x%2.2x as Subscribe message (0x02).\n", buf[0]);
		return(-1);
	}

	*ttl = (unsigned short int)((buf[1] << 8) | buf[2]);
	
	*sample_int = (unsigned short int)((buf[3] << 8) | buf[4]);

	*contexts = buf[5];
	
	for(i=0;i<*contexts;i++)
	{
		context_types[i] = buf[6+2*i];
		utility_values[i] = buf[7+2*i];
	}
	
	return(1);
}
