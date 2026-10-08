std_msgs/Header header
string clock_id
string session_id
float64 control_session_start
uint64 command_sequence
uint64 attempt_id
uint16 code
# failed, deferred, complete, complete_late, transport_latched
string outcome
uint64 frame_bytes
uint64 bytes_written
int32 system_error
float64 started_steady_time
float64 finished_steady_time
float64 deadline_steady_time
float64 elapsed_seconds
float64 deadline_overrun_seconds
# Wall time within write()/poll(); can also include scheduling during the call.
float64 write_syscall_seconds
float64 poll_seconds
float64 thread_cpu_seconds
uint32 write_calls
uint32 eagain_count
# Identity of the first fatal write, distinct from this attempted operation.
uint64 failure_attempt_id
uint16 failure_code
