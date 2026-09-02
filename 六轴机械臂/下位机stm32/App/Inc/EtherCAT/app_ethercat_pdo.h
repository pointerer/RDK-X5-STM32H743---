#ifndef APP_ETHERCAT_PDO_H
#define APP_ETHERCAT_PDO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_ETHERCAT_SERVO_RXPDO_SIZE 22U
#define APP_ETHERCAT_SERVO_TXPDO_SIZE 42U

#if defined(__CC_ARM)
#define APP_ETHERCAT_PDO_PACKED_BEGIN _Pragma("pack(push, 1)")
#define APP_ETHERCAT_PDO_PACKED
#define APP_ETHERCAT_PDO_PACKED_END _Pragma("pack(pop)")
#elif defined(__GNUC__)
#define APP_ETHERCAT_PDO_PACKED_BEGIN
#define APP_ETHERCAT_PDO_PACKED __attribute__((__packed__))
#define APP_ETHERCAT_PDO_PACKED_END
#else
#define APP_ETHERCAT_PDO_PACKED_BEGIN
#define APP_ETHERCAT_PDO_PACKED
#define APP_ETHERCAT_PDO_PACKED_END
#endif

APP_ETHERCAT_PDO_PACKED_BEGIN
typedef struct APP_ETHERCAT_PDO_PACKED
{
  /* Axis 1, RxPDO 0x1600, byte 0..10. */
  uint16_t controlword;                  /* 0x6040:00, byte 0, 16 bit */
  int8_t modes_of_operation;             /* 0x6060:00, byte 2, 8 bit */
  int32_t target_position;               /* 0x607A:00, byte 3, 32 bit */
  int16_t target_torque;                 /* 0x6071:00, byte 7, 16 bit */
  int16_t axis1_velocity_feed_forward;   /* 0x3097:00, byte 9, 16 bit */

  /* Axis 2, RxPDO 0x1610, byte 11..21. */
  uint16_t axis2_controlword;             /* 0x6840:00, byte 11, 16 bit */
  int8_t axis2_modes_of_operation;        /* 0x6860:00, byte 13, 8 bit */
  int32_t axis2_target_position;          /* 0x687A:00, byte 14, 32 bit */
  int16_t axis2_target_torque;            /* 0x6871:00, byte 18, 16 bit */
  int16_t axis2_velocity_feed_forward;    /* 0x3897:00, byte 20, 16 bit */
} AppEtherCAT_ServoRxPdo;
APP_ETHERCAT_PDO_PACKED_END

APP_ETHERCAT_PDO_PACKED_BEGIN
typedef struct APP_ETHERCAT_PDO_PACKED
{
  /* Axis 1, TxPDO 0x1A00, byte 0..20. */
  uint16_t statusword;                       /* 0x6041:00, byte 0, 16 bit */
  int16_t torque_actual_value;               /* 0x6077:00, byte 2, 16 bit */
  int32_t position_actual_value;             /* 0x6064:00, byte 4, 32 bit */
  int32_t velocity_actual_value;             /* 0x606C:00, byte 8, 32 bit */
  uint16_t error_code;                       /* 0x603F:00, byte 12, 16 bit */
  int8_t modes_of_operation_display;         /* 0x6061:00, byte 14, 8 bit */
  int16_t axis1_accelerometer;               /* 0x3154:00, byte 15, 16 bit */
  int32_t axis1_multi_position_actual;       /* 0x6164:00, byte 17, 32 bit */

  /* Axis 2, TxPDO 0x1A10, byte 21..41. */
  uint16_t axis2_statusword;                 /* 0x6841:00, byte 21, 16 bit */
  int16_t axis2_torque_actual_value;         /* 0x6877:00, byte 23, 16 bit */
  int32_t axis2_position_actual_value;       /* 0x6864:00, byte 25, 32 bit */
  int32_t axis2_velocity_actual_value;       /* 0x686C:00, byte 29, 32 bit */
  uint16_t axis2_error_code;                 /* 0x683F:00, byte 33, 16 bit */
  int8_t axis2_modes_of_operation_display;   /* 0x6861:00, byte 35, 8 bit */
  int16_t axis2_accelerometer;               /* 0x3954:00, byte 36, 16 bit */
  int32_t axis2_multi_position_actual;       /* 0x6964:00, byte 38, 32 bit */
} AppEtherCAT_ServoTxPdo;
APP_ETHERCAT_PDO_PACKED_END

typedef char AppEtherCAT_ServoRxPdo_SizeCheck[
  (sizeof(AppEtherCAT_ServoRxPdo) == APP_ETHERCAT_SERVO_RXPDO_SIZE) ? 1 : -1
];

typedef char AppEtherCAT_ServoTxPdo_SizeCheck[
  (sizeof(AppEtherCAT_ServoTxPdo) == APP_ETHERCAT_SERVO_TXPDO_SIZE) ? 1 : -1
];

typedef char AppEtherCAT_ServoRxPdo_LayoutCheck[
  ((offsetof(AppEtherCAT_ServoRxPdo, target_position) == 3U) &&
   (offsetof(AppEtherCAT_ServoRxPdo, axis2_controlword) == 11U) &&
   (offsetof(AppEtherCAT_ServoRxPdo, axis2_target_position) == 14U)) ? 1 : -1
];

typedef char AppEtherCAT_ServoTxPdo_LayoutCheck[
  ((offsetof(AppEtherCAT_ServoTxPdo, axis1_accelerometer) == 15U) &&
   (offsetof(AppEtherCAT_ServoTxPdo, axis1_multi_position_actual) == 17U) &&
   (offsetof(AppEtherCAT_ServoTxPdo, axis2_statusword) == 21U) &&
   (offsetof(AppEtherCAT_ServoTxPdo, axis2_position_actual_value) == 25U) &&
   (offsetof(AppEtherCAT_ServoTxPdo, axis2_multi_position_actual) == 38U)) ? 1 : -1
];

#ifdef __cplusplus
}
#endif

#endif /* APP_ETHERCAT_PDO_H */
