# UAVLAS_REPORT message data

uint64 timestamp		# time since system start (microseconds)

uint16 id			# Beacon ID
uint16 status			# Sensor status

# Sensor return NED data of beacon offset
float32 pos_x # offset from beacon in N axis (in m)
float32 pos_y # offset from beacon in E axis (in m)
float32 pos_z # offset from beacon in D axis (altitude in m)
