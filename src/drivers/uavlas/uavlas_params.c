
#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * UAVLAS landing system driver enable.
 *
 * This parameter allows to enable/disable uavlas driver
 *
 * @boolean
 * @value 0 Disabled
 * @value 1 Enabled
 * @reboot_required true
 * @group UAVLAS
 */
PARAM_DEFINE_INT32(ULS_ENABLED, 1);

/**
 * UAVLAS landing system platform mode
 *
 * This parameter allows to set landig on moved platform or stationary.
 *
 * @value 0 Moved platform
 * @value 1 Fixed platform
 * @group UAVLAS
 */

PARAM_DEFINE_INT32(ULS_PFM_MODE, 0);

/**
 * UAVLAS landing orientation mode
 *
 * This parameter allows to set mode of operation.
 * Use "GU IMU only" if you have only 1 receiver on board - no MRX
 * Use "MRX Body YAW" only if you have 2 or more receivers and ground unit compass unstable.
 *
 * @value 0 Any sensor
 * @value 1 GU IMU only
 * @value 2 MRX Body YAW only
 * @group UAVLAS
 */

PARAM_DEFINE_INT32(ULS_ORIENT_MODE, 0);

/**
 * UAVLAS landing system altitude data.
 *
 * This parameter force set altitude from ULS sensor
 *
 * @boolean
 * @value 0 Disabled
 * @value 1 Enabled
 * @group UAVLAS
 */
PARAM_DEFINE_INT32(ULS_PROVIDE_AGL, 0);

/**
 * UAVLAS  Force landing detection to drop vehicle/
 *
 * This parameter allows to force land dedection evet at pecific altitude (0-disable).
 * It is useful in conditions when landing detect may fail (landing for muving objects).
 *
 * @unit m
 * @min 0.0
 * @max 5.0
 * @group UAVLAS
 */
PARAM_DEFINE_FLOAT(ULS_DROP_ALT, 0.0f);


