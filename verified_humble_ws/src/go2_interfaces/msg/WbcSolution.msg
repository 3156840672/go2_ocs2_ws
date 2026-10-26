# Read-only result, not an actuator command or authorization to execute.
std_msgs/Header header
float64 reference_time
uint64 reset_epoch
uint64 policy_sequence
string command_path_manifest_sha256
float64 valid_from
float64 valid_until
uint8 planned_mode
string[12] joint_names
float64[12] desired_joint_positions
float64[12] desired_joint_velocities
float64[18] generalized_acceleration
# World-frame forces, FL FR RL RR. Torques follow joint_names (URDF order).
float64[12] contact_forces_world
float64[12] joint_torques
float64 dynamics_residual_inf
float64 contact_constraint_residual_inf
float64 inequality_violation
float64 solve_time_ms
