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

float32 mrxyaw	#boady relative orientation in MultiRX mode (deg)

float32	  rx_x	#  Receiver location on vehicle frame X position cm
float32	  rx_y	#  Receiver location on vehicle frame Y position cm

float32 sq    # signal quality
float32 ss    # common lighting level (db)
float32 sl    # signal level (%)

 # Estimated Position Data
float32  pos_ned_n # Relative Position of transmitter in NED (North)[m]
float32  pos_ned_e # Relative Position of transmitter in NED (East)[m]
float32  pos_ned_d # Relative Position of transmitter in NED (Down)[m]

float32  vel_ned_n # Ralative velocity of tranmitter in NED (North) [m/s]
float32  vel_ned_e # Ralative velocity of tranmitter in NED (East) [m/s]
float32  vel_ned_d # Ralative velocity of tranmitter in NED (Down) [m/s]

float32  pos_frd_f #Relative Position of transmitter in FRD (Forward) [m]
float32  pos_frd_r #Relative Position of transmitter in FRD (Right) [m]
float32  pos_frd_d #Relative Position of transmitter in FRD (Down) [m]

float32  vel_frd_f # Ralative velocity of tranmitter in FRD (Forward)[m/s]
float32  vel_frd_r # Ralative velocity of tranmitter in FRD (Right)[m/s]
float32  vel_frd_d # Ralative velocity of tranmitter in FRD (Down)[m/s]

float32  pos_wld_lat # World Position of transmitter in NED (Lat)[d]
float32  pos_wld_lon # World Position of transmitter in NED (Lon)[d]
float32  pos_wld_msl # World Position of transmitter in NED (Msl)[m]

float32  vel_wld_n   # World velocity of tranmitter in NED (North) [m/s]
float32  vel_wld_e   # World velocity of tranmitter in NED (East) [m/s]
float32  vel_wld_d   # World velocity of tranmitter in NED (Down) [m/s]


