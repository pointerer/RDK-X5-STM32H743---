#ifndef _ec_options_
#define _ec_options_

#ifdef __cplusplus
extern "C" {
#endif

/* Max sizes */
#define EC_BUFSIZE              (1518)
#define EC_MAXBUF               (16)
#define EC_MAXEEPBITMAP         (128)
#define EC_MAXEEPBUF            (EC_MAXEEPBITMAP << 5)
#define EC_LOGGROUPOFFSET       (16)
#define EC_MAXELIST             (64)
#define EC_MAXNAME              (40)
#define EC_MAXSLAVE             (200)
#define EC_MAXGROUP             (2)
#define EC_MAXIOSEGMENTS        (64)
#define EC_MAXMBX               (1486)
#define EC_MBXPOOLSIZE          (32)
#define EC_MAXEEPDO             (0x200)
#define EC_MAXSM                (8)
#define EC_MAXFMMU              (4)
#define EC_MAXLEN_ADAPTERNAME   (128)
#define EC_MAX_MAPT             (1)
#define EC_MAXODLIST            (1024)
#define EC_MAXOELIST            (256)
#define EC_SOE_MAXNAME          (60)
#define EC_SOE_MAXMAPPING       (64)

/* Timeouts and retries, unit: us */
#define EC_TIMEOUTRET           (2000)
#define EC_TIMEOUTRET3          (EC_TIMEOUTRET * 3)
#define EC_TIMEOUTSAFE          (20000)
#define EC_TIMEOUTEEP           (20000)
#define EC_TIMEOUTTXM           (20000)
#define EC_TIMEOUTRXM           (700000)
#define EC_TIMEOUTSTATE         (2000000)
#define EC_DEFAULTRETRIES       (3)

/* MAC addresses used internally by SOEM */
#define EC_PRIMARY_MAC_ARRAY    {0x0101, 0x0101, 0x0101}
#define EC_SECONDARY_MAC_ARRAY  {0x0404, 0x0404, 0x0404}

#ifdef __cplusplus
}
#endif

#endif
