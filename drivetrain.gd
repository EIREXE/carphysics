class_name CarDrivetrain

const GEARBOX_INERTIA = 0.02

var negative_gearbox_ratios: PackedFloat32Array = [
	-2.840
]

var positive_gearbox_ratios: PackedFloat32Array = [
	3.166,
	1.904,
	1.310,
	0.969,
	0.815
]

const GEAR_NEUTRAL = 0

var current_gear := 0

var final_drive := 4.312

var clutch_max_torque := 500.0
var clutch_stiffness := 1.8
var clutch_damping := 0.7
var clutch_torque := 0.0

func get_gear_ratio(gear_idx: int) -> float:
	if gear_idx == 0:
		return 0.0
	if gear_idx > 0:
		return positive_gearbox_ratios[gear_idx-1]
	
	return negative_gearbox_ratios[-gear_idx-1]
	
func gearbox_angular_vel_to_upstream(angular_velocity: float) -> float:
	return angular_velocity * get_gear_ratio(current_gear)

func gearbox_get_downstream_torque(clutch_torque: float) -> float:
	return clutch_torque * get_gear_ratio(current_gear)

func differential_angular_vel_to_upstream(angular_velocities: PackedFloat64Array) -> float:
	assert(angular_velocities.size() == 2)
	return (angular_velocities[0] + angular_velocities[1]) * final_drive * 0.5

func differential_get_downstream_torque(torque_input: float) -> PackedFloat32Array:
	return [torque_input * final_drive * 0.5, torque_input * final_drive * 0.5]

func update_clutch(clutch_input: float, in_gear: bool, velocity_in: float, velocity_out: float):
	var clutch_engagement := 1.0 - clutch_input
	var slip := 0.0
	if in_gear:
		slip = velocity_in - velocity_out
	else:
		slip = 0.0
	
	var torque := clutch_engagement * slip * clutch_stiffness
	clutch_torque += (torque - clutch_torque) * clutch_damping
	clutch_torque = clamp(clutch_torque, -clutch_max_torque, clutch_max_torque)

func gear_down():
	current_gear = max(current_gear-1, -negative_gearbox_ratios.size())

func gear_up():
	current_gear = min(current_gear+1, positive_gearbox_ratios.size())
