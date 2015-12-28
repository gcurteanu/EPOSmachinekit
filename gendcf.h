#include <data.h>

#define DCF_MAX_SIZE 8192
#define DEVICE_DICT_NAME "dcfdata.txt"

void dcf_data_display(uint8_t dcfdata[][DCF_MAX_SIZE]);
int dcf_read_in_file(char *fileName, uint8_t dcfdata[][DCF_MAX_SIZE]);
