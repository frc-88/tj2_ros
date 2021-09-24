int8 NOMODE=0
int8 DISABLED=1
int8 ESTOP=2
int8 TELEOP=3
int8 AUTONOMOUS=4
int8 TEST=5

bool is_connected
bool has_code
int8 mode
float32 voltage
int32 can_util
int32 cpu_usage
int32 ram_usage
int32 disk_usage
string status
