#include "main.h"
#include "string.h"
#include "protocol.h"
#include "referee.h"

//自定义控制器
frame_header_t referee_receive_header;
custom_controller_t custom_controller_data_t;
uint8_t button_left = 0;
uint8_t button_right = 0;
float ctr_loaction[6];

void referee_data_solve(uint8_t *frame)
{
  uint16_t cmd_id = 0;
  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_t));
  if(referee_receive_header.SOF == HEADER_SOF)
  {
    index += sizeof(frame_header_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
		if(cmd_id == CUSTOM_CONTROLLER_DATA_ID)
		{
			memcpy(&custom_controller_data_t, frame + index, sizeof(custom_controller_t));
			button_left = custom_controller_data_t.key_1;
			button_right = custom_controller_data_t.key_2;
			for(int i=0;i<6;i++){
			ctr_loaction[i]=custom_controller_data_t.location[i];
			}
			memset(&custom_controller_data_t, 0, sizeof(custom_controller_t));
		}
  }
}