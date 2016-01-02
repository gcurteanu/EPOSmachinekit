/*
dcf.h
*/
#ifndef __EPOS_DCF_H__
#define __EPOS_DCF_H__

#include <data.h>
#include "config.h"

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
int     add_dcf_node (dcfset_t *, UNS8, dcfstream_t**);
int     get_dcf_node (dcfset_t *, UNS8, dcfstream_t**);
int     load_dcf_set (dcfset_t *, const char *);

void    display_dcf_set (dcfset_t *);

#endif