/*
dcf.h
*/

#include <data.h>

int clear_dcf (UNS32 size, UNS8 dcfstream[]);
UNS32 get_dcf_count (UNS32 size, UNS8   dcfstream[]);
int add_dcf_entry (UNS32 size, UNS8 dcfstream[], UNS16 object, UNS8 subindex, UNS32 count, void * data);
void    display_dcf (UNS32 size, UNS8 dcfstream[]);

