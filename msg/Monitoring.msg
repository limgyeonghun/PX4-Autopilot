uint64 timestamp    # time since system start (microseconds)
uint32 tow          # GPS Time of Week
float32 pos_x       # Current position X (m)
float32 pos_y       # Current position Y (m)
float32 pos_z       # Current position Z (m)
float32 head        # Heading (degree)
float32 vx          # North velocity in NED earth-fixed frame, (metres/sec)
float32 vy          # East velocity in NED earth-fixed frame, (metres/sec)
float32 vz          # Down velocity in NED earth-fixed frame, (metres/sec)
float32 roll        # Roll
float32 pitch       # Pitch
float32 yaw         # Yaw
uint32 status1      # status #1
uint32 status2      # status #2 (RESERVED)
uint8 rtk_nbase     # The number of GPS satellite base obeservation
uint8 rtk_nrover    # The number of GPS satellite base obeservation
uint8 battery       # Battery
float32 rtk_n       # Rtk Baseline North coordinate (m)
float32 rtk_e       # Rtk Baseline East coordinate (m)
float32 rtk_d       # Rtk Baseline Down coordinate (m)