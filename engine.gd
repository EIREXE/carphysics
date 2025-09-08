class_name CarEngine

var angular_velocity := 0.0
var inertia := 0.12

const AV_2_RPM := 60.0 / (2.0 * PI)

var torque_curve: PackedVector2Array

var coast_ref_rpm := 7000.0
var coast_ref_torque := 70.0
var coast_ref_non_linearity := 0.0
var rpm_limit := 7250.0
var power_cut_frequency_hz := 20.0

var last_limiter_hit := 0.0
var t := 0.0

var current_applied_torque := 0.0

var effective_throttle := 0.0

func _init():
	var torque_curve_data := [
		0.0, 100.0,
		500.0,100.0,
		1000.0,96.0,
		1500.0,104.0,
		2000.0,112.0,
		2500.0,115.0,
		3000.0,117.0,
		3500.0,113.0,
		4000.0,122.0,
		4500.0,125.0,
		5000.0,124.0,
		5200.0,126.0,
		5500.0,125.0,
		6000.0,124.0,
		6500.0,122.0,
		6600.0,122.0,
		7000.0,114.0,
		7500.0,106.0,
		8000.0,96.0,
		8500.0,87.0,
		9000.0,0.0
	]
	
	for i in range(torque_curve_data.size() / 2):
		torque_curve.push_back(Vector2(torque_curve_data[i*2], torque_curve_data[i*2+1]))

func sample_torque(rpm: float) -> float:
	for i in range(torque_curve.size()):
		if i == 0:
			continue
		if torque_curve[i].x > rpm and torque_curve[i-1].x < rpm:
			var lerp_val := inverse_lerp(torque_curve[i-1].x, torque_curve[i].x, rpm)
			return lerp(torque_curve[i-1].y, torque_curve[i].y, lerp_val)
	return torque_curve[0].y

func sample_coast(rpm: float) -> float:
	return pow(rpm / coast_ref_rpm, 1.0 + coast_ref_non_linearity) * coast_ref_torque

func update(throttle: float, delta: float, clutch_torque: float):
	t += delta
	
	if t - last_limiter_hit < (1.0 / power_cut_frequency_hz):
		throttle = 0.0
	
	var current_rpm := angular_velocity * AV_2_RPM
	var torque_at_rpm := sample_torque(current_rpm)
	var coast_torque_at_rpm := sample_coast(current_rpm)
	
	# Idle regulation
	
	var activeIdleRange := 0.25;
	var idleThrottle := coast_torque_at_rpm / sample_torque(current_rpm);
	var idle_rpm := 1000.0
	var idleFadeRPM := inverse_lerp(idle_rpm + (rpm_limit - idle_rpm) * activeIdleRange, idle_rpm, current_rpm) as float
	idleThrottle *= idleFadeRPM;
	var _throttle = lerp(idleThrottle,1.0,throttle)
	_throttle = clamp(_throttle, 0.0, 1.0)
	throttle = _throttle
	effective_throttle = throttle
	
	var net_torque := lerp(-coast_torque_at_rpm, torque_at_rpm, throttle) as float
	net_torque -= clutch_torque
	angular_velocity += (net_torque / inertia) * delta
	angular_velocity = max(angular_velocity, 0)
	if angular_velocity * AV_2_RPM >= rpm_limit:
		last_limiter_hit = t
	current_applied_torque = net_torque
