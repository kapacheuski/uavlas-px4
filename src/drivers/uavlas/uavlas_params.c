
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
 * UAVLAS landing system mode
 *
 * This parameter allows to set mode of operation.
 *
 * @value 0 Moved platform
 * @value 1 Fixed platform
 * @group UAVLAS
 */

PARAM_DEFINE_INT32(ULS_MODE, 0);


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
 * UAVLAS  drop altitude.
 *
 * This parameter allows to decrease and cut throttle on landing stage ( 0-disabled )
 *
 * @unit m
 * @min 0.0
 * @max 5.0
 * @group UAVLAS
 */
PARAM_DEFINE_FLOAT(ULS_DROP_ALT, 0.0f);


