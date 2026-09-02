#ifndef OSHW_H
#define OSHW_H

#include "osal.h"
#include "nicdrv.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EC_LITTLE_ENDIAN
#define EC_LITTLE_ENDIAN
#endif

#ifndef htoes
#define htoes(A) (A)
#endif

#ifndef htoel
#define htoel(A) (A)
#endif

#ifndef etohs
#define etohs(A) (A)
#endif

#ifndef etohl
#define etohl(A) (A)
#endif

typedef struct ec_adapter ec_adaptert;

extern const uint16 priMAC[3];
extern const uint16 secMAC[3];

uint16 oshw_htons(uint16 host);
uint16 oshw_ntohs(uint16 network);
uint32 oshw_htonl(uint32 host);
uint32 oshw_ntohl(uint32 network);

ec_adaptert *oshw_find_adapters(void);
void oshw_free_adapters(ec_adaptert *adapter);

#ifdef __cplusplus
}
#endif

#endif
