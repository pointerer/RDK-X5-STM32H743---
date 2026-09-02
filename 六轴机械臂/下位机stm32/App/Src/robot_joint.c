#include "robot_joint.h"

#include <limits.h>

static const ROBOT_JOINT_Config robot_joint_config[ROBOT_JOINT_COUNT] =
{
  {1U, ROBOT_JOINT_AXIS_2, 1,  226000L},
  {1U, ROBOT_JOINT_AXIS_1, 1,  260000L},
  {2U, ROBOT_JOINT_AXIS_1, 1, 1666230L},
  {2U, ROBOT_JOINT_AXIS_2, 1, 1960000L},
  {3U, ROBOT_JOINT_AXIS_1, 1, 1713600L},
  {3U, ROBOT_JOINT_AXIS_2, 1, 2890000L}
};

static int64_t ROBOT_JOINT_DivideRounded(int64_t numerator,
                                         int64_t denominator)
{
  if (numerator >= 0)
  {
    return (numerator + (denominator / 2)) / denominator;
  }

  return -(((-numerator) + (denominator / 2)) / denominator);
}

const ROBOT_JOINT_Config *ROBOT_JOINT_GetConfig(uint8_t joint_index)
{
  if (joint_index >= ROBOT_JOINT_COUNT)
  {
    return 0;
  }

  return &robot_joint_config[joint_index];
}

uint8_t ROBOT_JOINT_RawToMilliDegree(uint8_t joint_index,
                                     int32_t raw_position,
                                     int32_t *angle_mdeg)
{
  const ROBOT_JOINT_Config *config;
  int64_t relative_count;
  int64_t result;

  config = ROBOT_JOINT_GetConfig(joint_index);
  if ((config == 0) || (angle_mdeg == 0))
  {
    return 0U;
  }

  relative_count = (int64_t)raw_position - (int64_t)config->zero_position;
  relative_count *= (int64_t)config->direction;
  result = ROBOT_JOINT_DivideRounded(
    relative_count * (int64_t)ROBOT_JOINT_MDEG_PER_REV,
    (int64_t)ROBOT_JOINT_COUNTS_PER_REV);
  if ((result < (int64_t)INT32_MIN) || (result > (int64_t)INT32_MAX))
  {
    return 0U;
  }

  *angle_mdeg = (int32_t)result;
  return 1U;
}

uint8_t ROBOT_JOINT_MilliDegreeToRaw(uint8_t joint_index,
                                     int32_t angle_mdeg,
                                     int32_t *raw_position)
{
  const ROBOT_JOINT_Config *config;
  int64_t relative_count;
  int64_t result;

  config = ROBOT_JOINT_GetConfig(joint_index);
  if ((config == 0) || (raw_position == 0))
  {
    return 0U;
  }

  relative_count = ROBOT_JOINT_DivideRounded(
    (int64_t)angle_mdeg * (int64_t)ROBOT_JOINT_COUNTS_PER_REV,
    (int64_t)ROBOT_JOINT_MDEG_PER_REV);
  relative_count *= (int64_t)config->direction;
  result = (int64_t)config->zero_position + relative_count;
  if ((result < (int64_t)INT32_MIN) || (result > (int64_t)INT32_MAX))
  {
    return 0U;
  }

  *raw_position = (int32_t)result;
  return 1U;
}
