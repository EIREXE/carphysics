extends RigidBody3D

class_name Car

enum WheelPos {
	FL,
	FR,
	RL,
	RR
}

var wheels: Array[WheelInfo]

var im := ImmediateMesh.new()
var mi := MeshInstance3D.new()

var relLenLongitudinal := 0.08
var relLenLateral := 0.16

@export var min_velocity: float = 0.25 # Minimum velocity threshold (m/s) to avoid singularity

var drivetrain := CarDrivetrain.new()
var engine := CarEngine.new()
@onready var engine_rpm_draw := EngineRPMGraph.new()

func _ready() -> void:
	var w := Window.new()
	w.hide()
	w.force_native = true
	w.size = Vector2(512, 512)
	w.show()
	add_child(w)
	engine_rpm_draw.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	w.add_child(engine_rpm_draw)
	# FL
	var fl := WheelInfo.new()
	fl.attachment_point = Vector3(-0.65, 0.0, -1.0)
	fl.steerable = true
	
	var fr := WheelInfo.new()
	fr.attachment_point = Vector3(0.65, 0.0, -1.0)
	fr.steerable = true
	
	var rl := WheelInfo.new()
	rl.differential_side = WheelInfo.DifferentialSide.DIFF_L
	rl.attachment_point = Vector3(-0.65, 0.0, 1.0)
	
	var rr := WheelInfo.new()
	rr.differential_side = WheelInfo.DifferentialSide.DIFF_R
	rr.attachment_point = Vector3(0.65, 0.0, 1.0)
	fl.n = "fl"
	fr.n = "fr"
	rr.n = "rr"
	rl.n = "rl"
	wheels.push_back(fl)
	wheels.push_back(fr)
	wheels.push_back(rr)
	wheels.push_back(rl)
	
	add_child(mi)
	mi.top_level = true
	mi.global_transform = Transform3D()
	mi.mesh = im
	var sm := StandardMaterial3D.new()
	sm.vertex_color_use_as_albedo = true
	sm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	sm.no_depth_test = true
	mi.material_overlay = sm
	
	for wheel in wheels:
		wheel.mi = $WheelMesh.duplicate()
		add_child(wheel.mi)
		wheel.mi.show()

func velocity_at_pos(world_pos: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(world_pos - global_position)

@export_range(0, 2, 0.05) var peak: float = 1
@export_range(1, 2.5, 0.05) var x_shape: float = 1.35
@export_range(1, 2.5, 0.05) var z_shape: float = 1.65
@export_range(0, 20, 0.1) var stiff: float = 10
@export_range(-10, 0, 0.05) var curve: float = 0

func pacejka(slip, t_shape, spring_force: float) -> float:
	return spring_force * peak * sin(t_shape * atan(stiff * slip - curve * (stiff * slip - atan(stiff * slip))))
	
func calc_slip_ratio(wheel: WheelInfo, wheel_velocity_forward: float, delta: float) -> float:
	var slip_vel_forward := wheel.angular_vel * wheel.radius - wheel_velocity_forward
	var velocity_forward_abs := max(abs(wheel_velocity_forward), 0.5) as float
	var steady_state_slip_ratio := slip_vel_forward / velocity_forward_abs
	var slip_ratio_delta_clamp := abs(steady_state_slip_ratio - wheel.differential_slip_ratio) / relLenLongitudinal / delta as float
	var slip_ratio_delta := (slip_vel_forward - abs(wheel_velocity_forward) * wheel.differential_slip_ratio) / relLenLongitudinal as float
	slip_ratio_delta = clamp(slip_ratio_delta, -slip_ratio_delta_clamp, slip_ratio_delta_clamp)
	wheel.differential_slip_ratio += slip_ratio_delta * delta
	wheel.differential_slip_ratio = clamp(wheel.differential_slip_ratio, -abs(steady_state_slip_ratio), abs(steady_state_slip_ratio))
	return wheel.differential_slip_ratio
	
func calc_slip_angle(wheel: WheelInfo, wheel_velocity_forward: float, wheel_velocity_right: float, delta: float):
	var velocity_forward_abs := max(abs(wheel_velocity_forward), 0.5) as float
	var steady_state_slip_angle := atan2(wheel_velocity_right, velocity_forward_abs)
	var tan_slip_angle_delta_clamp := abs(tan(steady_state_slip_angle) - wheel.differential_tan_slip_angle) / relLenLateral / delta as float
	var tan_slip_angle_delta := (wheel_velocity_right - abs(wheel_velocity_forward) * wheel.differential_tan_slip_angle) / relLenLateral as float
	tan_slip_angle_delta = clamp(tan_slip_angle_delta, -tan_slip_angle_delta_clamp, tan_slip_angle_delta_clamp)
	wheel.differential_tan_slip_angle += tan_slip_angle_delta * delta
	wheel.differential_tan_slip_angle = clamp(wheel.differential_tan_slip_angle, -abs(tan(steady_state_slip_angle)), abs(tan(steady_state_slip_angle)))
	return -atan(wheel.differential_tan_slip_angle)

func get_longitudinal_stiffness_from_vertical_load(vertical_load: float) -> float:
	return vertical_load * peak * stiff * z_shape

func aslr(delta: float, wheel_radius: float, wheel_angular_velocity: float, wheel_longitudinal_velocity: float, wheel_inertia: float, chassis_mass: float, vertical_load: float) -> float:
	var tau_m := delta * 0.5
	var stiffness := get_longitudinal_stiffness_from_vertical_load(abs(vertical_load)) # replaced with longitudinal slip stiffness from tyre model based on vertical load
	const SAFETY_FACTOR := 1.1
	var marginal_speed := tau_m * stiffness * (((wheel_radius * wheel_radius) / wheel_inertia) + (1.0 / chassis_mass))
	var kp: float = (wheel_radius * wheel_angular_velocity - wheel_longitudinal_velocity) / max(abs(wheel_longitudinal_velocity), SAFETY_FACTOR * marginal_speed)
	return kp

func _physics_process(delta: float) -> void:
	im.clear_surfaces()
	im.surface_begin(Mesh.PRIMITIVE_LINES)
	
	#if Input.is_action_pressed("ui_up"):
	#	apply_central_force(Vector3.FORWARD * 10000)
	if Input.is_action_just_pressed("ui_cancel"):
		linear_velocity = Vector3(-0.75, 0.0, 0.0)
		
	engine.update(Input.get_action_strength("throttle"), delta, drivetrain.clutch_torque)
		
	if Input.is_action_just_pressed("gear_up"):
		drivetrain.gear_up()
	if Input.is_action_just_pressed("gear_down"):
		drivetrain.gear_down()
	$"../Label".text = ""
	$"../Label".text += "GEAR: %d" % [drivetrain.current_gear]
	$"../Label".text += "\nENGINE RPM: %d" % [CarEngine.AV_2_RPM * engine.angular_velocity]
	$"../Label".text += "\nSPEED: %.2f" % [linear_velocity.length() * 3.6]
	im.surface_set_color(Color.WHITE)
	
	engine_rpm_draw.graph_update(engine.angular_velocity * CarEngine.AV_2_RPM)
	
	drivetrain.update_clutch(0.0, drivetrain.current_gear != CarDrivetrain.GEAR_NEUTRAL, engine.angular_velocity, drivetrain.gearbox_angular_vel_to_upstream(drivetrain.differential_angular_vel_to_upstream([wheels[WheelPos.RL].angular_vel, wheels[WheelPos.RR].angular_vel])))
	for wheel in wheels:
		var wp_wheel_attachment_point = to_global(wheel.attachment_point)
		var world_wheel_direction = global_transform.basis * wheel.direction
		
		var raycast := PhysicsRayQueryParameters3D.new()
		raycast.from = wp_wheel_attachment_point
		raycast.to = wp_wheel_attachment_point + world_wheel_direction * (wheel.max_length + wheel.radius)
		raycast.exclude = [get_rid()]
		var dss := get_world_3d().direct_space_state
		var result := dss.intersect_ray(raycast)
		if not result.is_empty():
			var wheel_moment := 0.5 * wheel.mass * wheel.radius * wheel.radius
			
			var new_extension := wp_wheel_attachment_point.distance_to(result.position) - wheel.radius
			var compression_depth := wheel.max_length - new_extension
			var spring_force := 14000.0 * compression_depth
			
			var contact_velocity := velocity_at_pos(result.position)
			var contact_velocity_along_spring := world_wheel_direction.dot(contact_velocity)
			
			var world_wheel_forward := global_transform.basis * wheel.get_local_forward()
			var world_wheel_right := global_transform.basis * wheel.get_local_right()
			var longitudinal_wheel_speed := contact_velocity.dot(world_wheel_forward)
			var lateral_wheel_speed := contact_velocity.dot(world_wheel_right)
			var damping_force := contact_velocity_along_spring * 900.38
			var out_spring_force := max(spring_force + damping_force, 0.0) as float
			apply_force(-world_wheel_direction * (out_spring_force), to_global(wheel.attachment_point) - global_position)
			
			var x_slip = calc_slip_angle(wheel, longitudinal_wheel_speed, lateral_wheel_speed, delta)
			#var z_slip = aslr(delta, wheel.radius, wheel.angular_vel, longitudinal_wheel_speed, wheel_moment, mass, spring_force + damping_force)
			var z_slip = calc_slip_ratio(wheel, longitudinal_wheel_speed, delta)
			var x_force := pacejka(x_slip, x_shape, out_spring_force)
			var z_force := pacejka(z_slip, z_shape, out_spring_force)
			im.surface_set_color(Color.BLUE)
			apply_force(world_wheel_forward * z_force, result.position - global_position)
			im.surface_add_vertex(result.position)
			im.surface_add_vertex(result.position + world_wheel_forward * z_force)
			im.surface_set_color(Color.WHITE)
			apply_force(world_wheel_right * x_force, result.position - global_position)
			$"../Label".text += "\n%s %s" % [wheel.n, str(wheel.steerable)]
			$"../Label".text += "\nAV: %.2f" % [wheel.angular_vel]

			wheel.mi.position = wheel.attachment_point + wheel.direction * (wheel.max_length - compression_depth)
			wheel.mi.rotation.y = wheel.steer
			wheel.mi.rotation.x += -wheel.angular_vel * delta
			
			im.surface_set_color(Color.RED)
			im.surface_add_vertex(result.position)
			im.surface_add_vertex(result.position + world_wheel_right * x_force)
			im.surface_set_color(Color.WHITE)
			
			im.surface_add_vertex(wp_wheel_attachment_point)
			im.surface_add_vertex(wp_wheel_attachment_point + world_wheel_direction * (wheel.max_length - compression_depth))
			
			var net_torque := (-z_force) * wheel.radius
			if wheel == wheels[WheelPos.RL]:
				net_torque += drivetrain.differential_get_downstream_torque(drivetrain.gearbox_get_downstream_torque(drivetrain.clutch_torque))[0]
			if wheel == wheels[WheelPos.RR]:
				net_torque += drivetrain.differential_get_downstream_torque(drivetrain.gearbox_get_downstream_torque(drivetrain.clutch_torque))[1]
			if not wheel.steerable:
				pass
				#net_torque += 400.0 if Input.is_action_pressed("ui_up") else 0.0
			if Input.is_action_pressed("hb") and not wheel.steerable:
				var brake_torque := min(net_torque + ((wheel.angular_vel * wheel_moment) / delta), 1000.0) as float
				net_torque -= brake_torque
			if Input.is_action_pressed("ui_down"):
				var brake_torque := min(net_torque + ((wheel.angular_vel * wheel_moment) / delta), 500.0) as float
				net_torque -= brake_torque
			wheel.angular_vel += delta * (net_torque / wheel_moment)
			
			if wheel.steerable:
				wheel.steer = 0.0
				if Input.is_action_pressed("ui_left"):
					wheel.steer = deg_to_rad(25.0)
				if Input.is_action_pressed("ui_right"):
					wheel.steer = -deg_to_rad(25.0)
			
	im.surface_end()
