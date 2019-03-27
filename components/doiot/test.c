#include "doiot.h"
#include <stdio.h>
// #include <stdlib.h>
#include <string.h>
#include <stdbool.h>

extern doiot_obj_t obj[DOIOT_OBJ_NUM];
extern doiot_event_t doiot;

int main()
{
	// doiot_event_t* doiot = get_doiot();
	
	// doiot_obj_t* temp = doiot_obj("3303");
	// object_reset(temp, "3303");
	//strcpy(temp.id, "3303");
	printf("doiot test: %d\n", doiot.flag_ip);
	for(size_t i = 0; i < 4; i++)
	{
		printf("obj_id: %s\n", obj[i].id);
	}
	
	
	return 0;
}
