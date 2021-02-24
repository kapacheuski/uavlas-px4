# UAVLAS_REPORT message data

uint64 timestamp		# time since system start (microseconds)

uint16 id			# Beacon ID
uint16 status			# Sensor status

# Sensor return relative data of beacon offset

float32 pos_x # offset from beacon in X axis (in m)
float32 pos_y # offset from beacon in Y axis (in m)
float32 pos_z # offset from beacon in Z axis (altitude in m)

float32 vel_x # velocity relative to groud unit. X axis (m/s)
float32 vel_y # velocity relative to groud unit. Y axis (m/s)
float32 vel_z # velocity relative to groud unit. Z axis (m/s)

float32 gimu_roll  # orientation of ground unit - roll (deg)
float32 gimu_pitch # orientation of ground unit - pitch (deg)
float32 gimu_yaw   # orientation of ground unit - yaw (compass) (deg)

float32 bro_yaw	#boady relative orientation in MultiRX mode (deg)

float32 sq    # signal quality
float32 ss    # common lighting level (db)
float32 sl    # signal level (%)
