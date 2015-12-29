#include "dcf.h"

int clear_dcf (dcfstream_t *dcf) {
    
    if (!dcf)
        return 0;
    
    if (dcf->size >= 4) {
        // clean the DCF data
        dcf->dcf[0] = 0x00;
        dcf->dcf[1] = 0x00;
        dcf->dcf[2] = 0x00;
        dcf->dcf[3] = 0x00;
        dcf->nodeid = 0x00;
        return 1;
    }
    
    return 0;
}

UNS32 get_dcf_count (dcfstream_t *dcf) {

    if (!dcf)
       return 0;
   
    if (dcf->size < 4)
        return 0;
    
    UNS32 count = dcf->dcf[0] | dcf->dcf[1]<<8 | dcf->dcf[2]<<16 | dcf->dcf[3]<<24;
    return count;
}

int add_dcf_entry (dcfstream_t *dcf, UNS16 object, UNS8 subindex, UNS32 datasize, void * data)
{
    int cursor = 4;
    UNS32 count = 0;
    UNS32 total_items = get_dcf_count (dcf);
    
    if (!dcf)
        return 0;
    
    // go to the DCF end
    while (count < total_items) {
        // consume 1 item
        
        // consume index and subindex (16+8)
        cursor+=3;
        
        // are we still in the array?
        if ((cursor+4) >= dcf->size)
            return 0;
        
        UNS32   itemsize = dcf->dcf[cursor+0] | dcf->dcf[cursor+1]<<8 | 
            dcf->dcf[cursor+2]<<16 | dcf->dcf[cursor+3]<<24;
            
        cursor += 4;
        
        // are we still in the array?
        if ((cursor+itemsize) >= dcf->size)
            return 0;
        
        cursor += itemsize;
        
        // consumed the item
        count++;
    }
    
    // we have the cursor at the end of the stream
    // verify that we have space
    if ((cursor + 7 + count) >= dcf->size)
        return 0;
    
    dcf->dcf[cursor++] = object;
    dcf->dcf[cursor++] = object >> 8;
    dcf->dcf[cursor++] = subindex;
    dcf->dcf[cursor++] = datasize;
    dcf->dcf[cursor++] = datasize >> 8;
    dcf->dcf[cursor++] = datasize >> 16;
    dcf->dcf[cursor++] = datasize >> 24;
    
    int     idx;
    for (idx = 0; idx < datasize; idx++)
        dcf->dcf[cursor++] = ((UNS8 *)data)[idx];
    
    // increment count
    total_items++;
    dcf->dcf[0] = total_items;
    dcf->dcf[0] = total_items >> 8;
    dcf->dcf[0] = total_items >> 16;
    dcf->dcf[0] = total_items >> 24;
    
    return 1;
}

void    display_dcf (dcfstream_t *dcf) {
    
    UNS32   total_items = get_dcf_count (dcf);
    UNS32   count;
    int     cursor = 4;
    
    if (!dcf)
        return;
    
    while (count < total_items) {
        
        UNS16   idx;
        UNS8    subidx;
        UNS32   datasize;
        UNS32   data;
        int     i;
        
        idx = dcf->dcf[cursor++] | dcf->dcf[cursor++] << 8;
        subidx = dcf->dcf[cursor++];
        datasize = dcf->dcf[cursor++] | dcf->dcf[cursor++] << 8 | 
            dcf->dcf[cursor++] << 16 | dcf->dcf[cursor++] << 24;
            
        for (i = 0; i < datasize ; i++) {
            
            data = data | dcf->dcf[cursor++] << (8*i);
        }
        
        // we have the item, display it
        
        switch (datasize) {
            case 1:
                eprintf ("%04x %02x (%d) %02x", idx, subidx, datasize, data);
                break;
            case 2:
                eprintf ("%04x %02x (%d) %04x", idx, subidx, datasize, data);
                break;
            case 4:
            default:
                eprintf ("%04x %02x (%d) %08x", idx, subidx, datasize, data);
        }
        
        count++;
    }
}

int     get_dcf_node (dcfset_t * set, UNS8 nodeid, dcfstream_t** dcf) {
    
    if (!set || !dcf || nodeid < 1)
        return 0;
    
    int idx;
    
    *dcf = NULL;
    
    for (idx = 0; idx < set->count; idx++)
        if (set->nodes[idx].nodeid = nodeid) {
            *dcf = &set->nodes[idx];
            return 1;
        }
        
    return 0;
}
