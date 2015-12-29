/*
dcf.h
*/

#include <data.h>

#define EPOS_DCF_MAX_SIZE   16384
#define EPOS_DCF_MAX_NODES  16

typedef struct {
    UNS8    dcf[EPOS_DCF_MAX_SIZE];
    int     size;
    UNS8    nodeid;
} dcfstream_t;

typedef struct {
    dcfstream_t nodes[EPOS_DCF_MAX_NODES];
    int         size;
    int         count;
} dcfset_t;

int     clear_dcf (dcfstream_t *);
UNS32   get_dcf_count (dcfstream_t *);
int     add_dcf_entry (dcfstream_t *, UNS16 object, UNS8 subindex, UNS32 count, void * data);
void    display_dcf (dcfstream_t *);

int     clear_dcf_set (dcfset_t *);
int     add_dcf_node (dcfset_t *, UNS8);
int     get_dcf_node (dcfset_t *, UNS8, dcfstream_t**);

int     load_dcf_set (dcfset_t *, const char *);
