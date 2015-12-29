#include <dcf.h>


int clear_dcf (UNS32 size, UNS8 dcfstream[]) {
    
    if (size >= 4) {
        // clean the DCF data
        dcfstream[0] = 0x00;
        dcfstream[1] = 0x00;
        dcfstream[2] = 0x00;
        dcfstream[3] = 0x00;

        return 1;
    }
    
    return 0;
}

UNS32 get_dcf_count (UNS32 size, UNS8   dcfstream[]) {

    if (size < 4)
        return 0;
    
    UNS32 count = dcfstream[0] | dcfstream[1]<<8 | dcfstream[2]<<16 | dcfstream[3]<<24;
    return count;
}

int add_dcf_entry (UNS32 size, UNS8 dcfstream[], UNS16 object, UNS8 subindex, UNS32 count, void * data)
{
    int cursor = 4;
    UNS32 count = 0;
    UNS32 total_items = get_dcf_count (size, dcfstream);
    
    // go to the DCF end
    while (count < total_items) {
        // consume 1 item
        
        // consume index and subindex (16+8)
        cursor+=3;
        
        // are we still in the array?
        if ((cursor+4) >= size)
            return 0;
        
        UNS32   itemsize = dcfstream[cursor+0] | dcfstream[cursor+1]<<8 | 
            dcfstream[cursor+2]<<16 | dcfstream[cursor+3]<<24;
            
        cursor += 4;
        
        // are we still in the array?
        if ((cursor+itemsize) >= size)
            return 0;
        
        cursor += itemsize;
        
        // consumed the item
        count++;
    }
    
    // we have the cursor at the end of the stream
    // verify that we have space
    if ((cursor + 7 + count) >= size)
        return 0;
    
    dcfstream[cursor++] = object;
    dcfstream[cursor++] = object >> 8;
    dcfstream[cursor++] = subindex;
    dcfstream[cursor++] = count;
    dcfstream[cursor++] = count >> 8;
    dcfstream[cursor++] = count >> 16;
    dcfstream[cursor++] = count >> 24;
    
    int     idx;
    for (idx = 0; idx < count; idx++)
        dcfstream[cursor++] = ((UNS8 *)data)[idx];
    
    // increment count
    total_items++;
    dcfstream[0] = total_items;
    dcfstream[0] = total_items >> 8;
    dcfstream[0] = total_items >> 16;
    dcfstream[0] = total_items >> 24;
    
    return 1;
}

void    display_dcf (UNS32 size, UNS8 dcfstream[]) {
    
    UNS32   total_items = get_dcf_count (size, dcfstream);
    UNS32   count;
    int     cursor = 4;
    
    while (count < total_items) {
        
        UNS16   idx;
        UNS8    subidx;
        UNS32   size;
        UNS32   data;
        int     i;
        
        idx = dcfstream[cursor++] | dcfstream[cursor++] << 8;
        subidx = dcfstream[cursor++];
        size = dcfstream[cursor++] | dcfstream[cursor++] << 8 | 
            dcfstream[cursor++] << 16 | dcfstream[cursor++] << 24;
            
        for (i = 0; i < size ; i++) {
            
            data = data | dcfstream[cursor++] << (8*i);
        }
        
        // we have the item, display it
        
        eprintf ("%04x %02x (%d) %08x", idx, subidx, size, data);
        count++;
    }
}