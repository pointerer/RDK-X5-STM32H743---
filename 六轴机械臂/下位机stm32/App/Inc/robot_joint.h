#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_JOINT_COUNT            6U
#define ROBOT_JOINT_AXIS_1           1U
#define ROBOT_JOINT_AXIS_2           2U
#define ROBOT_JOINT_COUNTS_PER_REV   13238272L
#define ROBOT_JOINT_MDEG_PER_REV     360000L

typedef struct
{
  uint16_t slave;
  uint8_t axis;
  int8_t direction;
  int32_t zero_position;
} ROBOT_JOINT_Config;

/* joint_index uses the logical Joint1..Joint6 order, represented by 0..5. */
const ROBOT_JOINT_Config *ROBOT_JOINT_GetConfig(uint8_t joint_index);

/* Convert an absolute encoder count to a zero-relative joint angle in 0.001 deg. */
uint8_t ROBOT_JOINT_RawToMilliDegree(uint8_t joint_index,
                                     int32_t raw_position,
                                     int32_t *angle_mdeg);

/* Convert a zero-relative joint angle in 0.001 deg to an absolute encoder count. */
uint8_t ROBOT_JOINT_MilliDegreeToRaw(uint8_t joint_index,
                                     int32_t angle_mdeg,
                                     int32_t *raw_position);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_JOINT_H */
