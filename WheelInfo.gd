class_name WheelInfo

var n: String
var attachment_point: Vector3
var resting_legth := 0.0
var max_length := 0.5
var min_length := 0.0
var extension := 0.0
var radius := 0.3
var angular_vel := 0.0
var direction := Vector3.DOWN
var mi: MeshInstance3D
var mass := 20.0
var differential_slip_ratio := 0.0
var differential_tan_slip_angle := 0.0
var steer := 0.0
var steerable := false
var last_torque := 0.0
var inertia := 0.0

enum DifferentialSide {
	NOT_DRIVEN,
	DIFF_L,
	DIFF_R
}

var differential_side := DifferentialSide.NOT_DRIVEN

func get_local_forward() -> Vector3:
	return Vector3.FORWARD.rotated(Vector3.UP, steer)
func get_local_right() -> Vector3:
	return Vector3.RIGHT.rotated(Vector3.UP, steer)
