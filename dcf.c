#include <stdlib.h>
#include "dcf.h"

#define eprintf(...) printf(__VA_ARGS__)

int clear_dcf (dcfstream_t *dcf) {
    
    if (!dcf)
        return 0;
    
    dcf->size = EPOS_DCF_MAX_SIZE;
    
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

int     add_dcf_node (dcfset_t * set, UNS8 nodeid, dcfstream_t** dcf) {
    
    if (!set || !dcf || nodeid < 1)
        return 0;
    
    *dcf = NULL;
    
    if (set->count < set->size) {
        clear_dcf (&set->nodes[set->count]);
        set->nodes[set->count].nodeid = nodeid;
        *dcf = &set->nodes[set->count];
        set->count++;
        
        return 1;
    }

    return 0;
}

int clear_dcf_set (dcfset_t *set) {
    
    if (!set)
        return 0;
    
    set->size = EPOS_DCF_MAX_NODES;
    set->count = 0;
    
    int idx;
    
    for (idx = 0; idx < set->size; idx++) {
        
        set->nodes[idx].nodeid = 0x00;
        clear_dcf (&set->nodes[idx]);
    }
    
    return 1;
}

void    display_dcf_set (dcfset_t *set) {
    
    if (!set)
        return;
    
    int idx;
    
    for (idx = 0; idx < set->count; idx++) {
        
        printf ("Concise DCF for node %d", set->nodes[idx].nodeid);
        printf ("Has %d entries\n", get_dcf_count (&set->nodes[idx]));
        display_dcf (&set->nodes[idx]);
    }
}

int     load_dcf_set (dcfset_t *set, const char *filename) {
    
    if (!set)
        return 0;
    
    clear_dcf_set (set);
    
    FILE    *f;
    
    f = fopen (filename, "r");
    if (!f)
        return 0;
    
    char        line[2048];
    dcfstream_t *dcfstream = NULL;
    
    UNS8        nodeid;
    UNS16       idx;
    UNS8        subidx;
    UNS32       len;
    UNS32       data;
    
    char        *token, *errcheck;
    
    while (!feof(f)) {
        char *ptr = fgets (line, 2048, f);
        if (!ptr)
            break;
        
        // skin comments
        if (line[0] == '#' || line[0] == '/')
            continue;
        
        if (line[0] == '[') {
            // new section
            token = strtok (line, " \t\r\n[]");
            if (!token) {
                printf ("Invalid section start %s", line);
                continue;
            }
            
            nodeid = strtol (token, &errcheck, 0);
            if (token == errcheck || errcheck[0] != 0x00) {
                printf("Can not convert <%s> to index number, skipping line\n", token);
                continue;
            }
            
            // we have a [nodeid]
            // test to see if duplicate
            if (get_dcf_node (set, nodeid, &dcfstream)) {
                printf("Duplicate nodeid %d found in file\n", nodeid);
                fclose (f);
                return 0;
            }
            
            if (!add_dcf_node(set, nodeid, &dcfstream)) {
                printf("Can not add nodeid %d\n", nodeid);
                fclose (f);
                return 0;                
            }
            // hopefully we have the node ID and allocated the entry for it
            continue;
        }
        
        // split the line in tokens
        token = strtok (line, " \t\r\n");
        // empty line?
        if (!token)
            continue;
        
        idx = strtol (token, &errcheck, 0);
        if (token == errcheck || errcheck[0] != 0x00) {
            printf("Can not convert <%s> to index number, skipping line\n", token);
            continue;
        }
        
        token = strtok (NULL, " \t\r\n");
        if (!token)
            continue;
        subidx = strtol (token, &errcheck, 0);
        if (token == errcheck || errcheck[0] != 0x00) {
            printf("Can not convert <%s> to subindex number, skipping line\n", token);
            continue;
        }

        token = strtok (NULL, " \t\r\n");
        if (!token)
            continue;
        len = strtol (token, &errcheck, 0);
        if (token == errcheck || errcheck[0] != 0x00) {
            printf("Can not convert <%s> to length, skipping line\n", token);
            continue;
        }
        
        token = strtok (NULL, " \t\r\n");
        if (!token)
            continue;
        data = strtol (token, &errcheck, 0);
        if (token == errcheck || errcheck[0] != 0x00) {
            printf("Can not convert <%s> to data, skipping line\n", token);
            continue;
        }
        
        // we have a entry. Add it to the DCF
        if (dcfstream != NULL) {
            if (!add_dcf_entry (dcfstream, idx, subidx, len, &data)) {
                printf ("Can't add DCF entry %04x/%02x = %08x (%d)\n", idx, subidx, data, len);
                continue;
            }
        } else {
            printf ("Found data outside of [nodeid] section\n");
        }
    }
    
    close (f);
}
