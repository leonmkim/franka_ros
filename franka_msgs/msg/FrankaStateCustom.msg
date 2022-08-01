std_msgs/Header header
float64[6] cartesian_collision
float64[6] cartesian_contact
float64[7] q
float64[7] q_d
float64[7] dq
float64[7] dq_d
float64[7] ddq_d
float64[7] theta
float64[7] dtheta
float64[7] tau_J
float64[7] dtau_J
float64[7] tau_J_d
float64[6] K_F_ext_hat_K
float64[2] elbow
float64[2] elbow_d
float64[2] elbow_c
float64[2] delbow_c
float64[2] ddelbow_c
float64[7] joint_collision
float64[7] joint_contact
float64[6] O_F_ext_hat_K
float64[6] O_dP_EE_d
float64[6] O_dP_EE_c
float64[6] O_ddP_EE_c
float64[7] tau_ext_hat_filtered
float64 m_ee
float64[3] F_x_Cee
float64[9] I_ee
float64 m_load
float64[3] F_x_Cload
float64[9] I_load
float64 m_total
float64[3] F_x_Ctotal
float64[9] I_total
float64[16] O_T_EE
float64[16] O_T_EE_d
float64[16] O_T_EE_c
float64[16] F_T_EE
float64[16] F_T_NE
float64[16] NE_T_EE
float64[16] EE_T_K
float64 time
float64 control_command_success_rate
uint8 ROBOT_MODE_OTHER=0
uint8 ROBOT_MODE_IDLE=1
uint8 ROBOT_MODE_MOVE=2
uint8 ROBOT_MODE_GUIDING=3
uint8 ROBOT_MODE_REFLEX=4
uint8 ROBOT_MODE_USER_STOPPED=5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
uint8 robot_mode
franka_msgs/Errors current_errors
franka_msgs/Errors last_motion_errors

# Custom
float64[7] gravity
float64[7] coriolis
float64[42] O_Jac_EE # zero jacobian of end-effector frame. Vectorized 6x7 Jacobian, column-major
float64[49] mass_matrix # mass matrix of end-effector wrt base frame # Vectorized 7x7, column-major
float64[6] O_dP_EE # EE vel computed as J*dq
float64[6] O_F_EE_d # Tau_J_d transformed to EE space via the zero jacobian


