#if !defined(__PID_CONTROLLER_H__)
#define __PID_CONTROLLER_H__


template <class var_type, class coefs_type>
class PID_controller {
protected:
	var_type prev_err{0};			// previous error value
	var_type integral{0};			// sum of errors
	var_type min_int{0};			// minimal integral component value
	var_type max_int{0};			// maximum integral component value
	var_type control_left_lim{0};	// left limit for computing control signal
	var_type control_right_lim{0};	// right limit for computing control signal
	float time_step{0.001};			// discretization step
	coefs_type k_p, k_i, k_d;		// controller coefficients
public:
	/**
	 * @brief Default empty constructor
	 */	
	PID_controller() {}

	/**
	 * @brief Constructor with discretization time argument
	 * @param dt Time step (default value is 0.001)
	 */
	PID_controller(float dt);

	/**
	 * @brief Default empty destructor
	 */
	~PID_controller() {}

	/**
	 * @brief Controller coefficients setter
	 * @param new_k_p New proportional coefficient value
	 * @param new_k_i New integral coefficient value
	 * @param new_k_d New differential coefficient value
	 */
	void set_coefficients(coefs_type new_k_p, coefs_type new_k_i, coefs_type new_k_d);

	/**
	 * @brief Individual proportional coefficient setter
	 * @param new_k_p New proportional coefficient value
	 */
	void set_kp(coefs_type new_k_p);

	/**
	 * @brief Individual integral coefficient setter
	 * @param new_k_i New integral coefficient value
	 */
	void set_ki(coefs_type new_k_i);

	/**
	 * @brief Individual differential coefficient setter
	 * @param new_k_d New differential coefficient value
	 */
	void set_kd(coefs_type new_k_d);

	/**
	 * @brief Get proportional coefficient
	 * @return Proportional coefficient value
	 */
	coefs_type get_kp(void);

	/**
	 * @brief Get integral coefficient
	 * @return Integral coefficient value
	 */
	coefs_type get_ki(void);

	/**
	 * @brief Get differential coefficient
	 * @return Differential coefficient value
	 */
	coefs_type get_kd(void);

	/**
	 * @brief Set discretization step
	 * @param dt New time step value
	 */
	void set_time_step(float dt);

	/**
	 * @brief Set min and max integral sum value
	 * @param min_signal Min integral sum value
	 * @param max_signal Max integral sum value
	 */
	void set_integral_limits(var_type min_signal, var_type max_signal);

	/**
	 * @brief Set control limits
	 * @param left_ctrl Left limit value
	 * @param right_ctrl Right limit value
	 */
	void set_control_limits(var_type left_ctrl, var_type right_ctrl);

	/**
	 * @brief Computing the control signal value
	 * @param current_val Current controlling value
	 * @param target_val Target controlling value
	 * @return Computed control signal value
	 * */
	virtual var_type step(var_type current_val, var_type target_val);

	/**
	 * @brief Clear current process values
	 * Can be used in case of new target value 
	 * */
	void clear(void);
};

//////////////////////////////
template <class var_type, class coefs_type>
PID_controller<var_type, coefs_type>::PID_controller(float dt) : time_step(dt) {}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::set_coefficients(coefs_type new_k_p, coefs_type new_k_i, coefs_type new_k_d) {
    set_kp(new_k_p);
    set_ki(new_k_i);
    set_kd(new_k_d);
}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::set_kp(coefs_type new_k_p) {k_p = new_k_p;}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::set_ki(coefs_type new_k_i) {k_i = new_k_i * time_step;}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::set_kd(coefs_type new_k_d) {k_d = new_k_d / time_step;}

template <class var_type, class coefs_type>
coefs_type PID_controller<var_type, coefs_type>::get_kp(void) {return k_p;}

template <class var_type, class coefs_type>
coefs_type PID_controller<var_type, coefs_type>::get_ki(void) {return k_i / time_step;}

template <class var_type, class coefs_type>
coefs_type PID_controller<var_type, coefs_type>::get_kd(void) {return k_d * time_step;}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::set_time_step(float dt) {
	coefs_type tmp_k_i = k_i / time_step; // backup
	coefs_type tmp_k_d = k_d * time_step; // backup
	// set new values
	time_step = dt;
	set_ki(tmp_k_i);
	set_kd(tmp_k_d);
}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::set_integral_limits(var_type min_signal, var_type max_signal) {
	if (max_signal > min_signal) {
		min_int = min_signal;
		max_int = max_signal;
	}
}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::set_control_limits(var_type left_ctrl, var_type right_ctrl) {
	if (right_ctrl >= left_ctrl) {
		control_left_lim = left_ctrl;
		control_right_lim = right_ctrl;
	}
}

template <class var_type, class coefs_type>
var_type PID_controller<var_type, coefs_type>::step(var_type current_val, var_type target_val) {
	var_type output_signal; // value to return
	var_type err = target_val - current_val; // control error computing
	if ((err >= control_left_lim) && (err <= control_right_lim)) {
		output_signal = 0;
	} else {
		integral += err;
		// constraining	
		if (integral > max_int) integral = max_int;
		else if (integral < min_int) integral = min_int;
		// compute resulting control signal
		output_signal = err * k_p + integral * k_i + (err - prev_err) * k_d;
	}
	return output_signal;
}

template <class var_type, class coefs_type>
void PID_controller<var_type, coefs_type>::clear(void) {
	integral = 0;
	prev_err = 0;
}
#endif // __PID_CONTROLLER_H__
