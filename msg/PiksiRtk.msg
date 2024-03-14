# piksi_rtk
uint64 timestamp    # time since system start (microseconds)
uint64 utc_usec	    # time of this estimate, in microseconds since system start
uint32 tow			# GPS Time of Week
uint32 iar			# IAR(Integer Ambiguity Resolution)
uint16 age_corr     # Age correction
uint8 flag			# flg (0: float RTK, 1: fixed RTK)
uint8 nsats			# Number of satellites used in solution
float32 n			# Baseline North coordinate (m)
float32 e			# Baseline East coordinate (m)
float32 d			# Baseline Down coordinate (m)
float32 vn			# Velocity of Baseline North coordinate (m)
float32 ve			# Velocity of Baseline East coordinate (m)
float32 vd			# Velocity of Baseline Down coordinate (m)
uint8 nobs_base			# The number of GPS satellite base obeservation
uint32 period_base		# The recv period for base observation (ms)
uint8 nobs_rover		# The number of GPS satellite rover obeservation
uint32 period_rover		# The recv period for rover observation (ms)
uint32 max_time_base		# The maximum time of loss from base during some period (ms)
uint32 max_time_rover		# The maximum time of loss from rover during some period (ms)
