# This is piksi message data topic from mavlink
uint64 timestamp        # time since system start (microseconds)
uint16 sender_id		# sender ID
uint16 msg_type         # message type
uint8 len			    # observation data length
uint8[249] data			# piksi observation data
