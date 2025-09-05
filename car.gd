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

var relLenLongitudinal := 1.08
var relLenLateral := 0.76

@export var min_velocity: float = 0.25 # Minimum velocity threshold (m/s) to avoid singularity

var drivetrain := CarDrivetrain.new()
var engine := CarEngine.new()
@onready var engine_rpm_draw := TelemetryGraph.new()
@onready var slip_angle_graph := TelemetryGraph.new()
@onready var slip_ratio_graph := TelemetryGraph.new()
@onready var drivetrain_torques_graph := TelemetryGraph.new()
@onready var tire_forces_longitudinal_graph := TelemetryGraph.new()
@onready var wheel_torque_graph := TelemetryGraph.new()

enum DrivetrainTorquesGraphChannels {
	CLUTCH_TORQUE,
	ENGINE_TORQUE
}

func _ready() -> void:
	var w := Window.new()
	w.hide()
	w.force_native = true
	w.size = Vector2(512, 512)
	w.show()
	add_child(w)
	engine_rpm_draw.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	w.add_child(engine_rpm_draw)
	engine_rpm_draw.set_graph_channels(1)
	engine_rpm_draw.channel_set_name(0, "Engine RPM")
	var vbox := VBoxContainer.new()
	vbox.set_anchors_and_offsets_preset(Control.PRESET_TOP_WIDE)
	add_child(vbox)
	var hbox := GridContainer.new()
	hbox.columns = 2
	vbox.add_child(hbox)
	
	hbox.add_child(slip_angle_graph)
	slip_angle_graph.set_graph_channels(WheelPos.size())
	slip_angle_graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slip_angle_graph.custom_minimum_size = Vector2(0, 256)
	
	hbox.add_child(slip_ratio_graph)
	slip_ratio_graph.set_graph_channels(WheelPos.size())
	slip_ratio_graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slip_ratio_graph.custom_minimum_size = Vector2(0, 256)
	
	hbox.add_child(tire_forces_longitudinal_graph)
	tire_forces_longitudinal_graph.set_graph_channels(WheelPos.size())
	tire_forces_longitudinal_graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	tire_forces_longitudinal_graph.custom_minimum_size = Vector2(0, 256)
	
	hbox.add_child(drivetrain_torques_graph)
	drivetrain_torques_graph.set_graph_channels(DrivetrainTorquesGraphChannels.size())
	drivetrain_torques_graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	drivetrain_torques_graph.custom_minimum_size = Vector2(0, 256)
		
	hbox.add_child(wheel_torque_graph)
	wheel_torque_graph.set_graph_channels(WheelPos.size())
	wheel_torque_graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	wheel_torque_graph.custom_minimum_size = Vector2(0, 256)
		
	for i in range(WheelPos.size()):
		slip_angle_graph.channel_set_name(i, "%s slip angle" % [WheelPos.keys()[i]])
		slip_ratio_graph.channel_set_name(i, "%s slip ratio" % [WheelPos.keys()[i]])
		tire_forces_longitudinal_graph.channel_set_name(i, "%s longitudinal force" % [WheelPos.keys()[i]])
		wheel_torque_graph.channel_set_name(i, "%s wheel net torque" % [WheelPos.keys()[i]])
	slip_angle_graph.set_limits(-180.0, 180.0)
	slip_ratio_graph.set_limits(-10.0, 10.0)
	tire_forces_longitudinal_graph.set_limits(-4000.0, 4000.0)
	wheel_torque_graph.set_limits(-250.0, 250.0)
	drivetrain_torques_graph.channel_set_name(DrivetrainTorquesGraphChannels.CLUTCH_TORQUE, "Clutch torque")
	drivetrain_torques_graph.channel_set_name(DrivetrainTorquesGraphChannels.ENGINE_TORQUE, "Engine net torque")
	drivetrain_torques_graph.set_limits(-300.0, 300.0)
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

var con_patch := 0.15
var mu := 0.85
var tire_stiffness := 5.5

# Tire parameters
@export var cornering_stiffness: float = 10000.0  # N/rad, lateral stiffness
@export var longitudinal_stiffness: float = 15000.0  # N/unit slip, longitudinal stiffness
@export var friction_coefficient: float = 1.0  # Peak friction coefficient
@export var normal_load: float = 4000.0  # N, vertical load on the tire
@export var contact_patch_length: float = 0.15  # m, length of the contact patch

# Calculates tire forces based on slip angle (radians) and slip ratio
func calculate_tire_forces(slip_angle: float, slip_ratio: float, wheel_velocity: float) -> Vector2:
	# Initialize forces (longitudinal, lateral)
	var tire_forces := Vector2.ZERO

# Effective stiffness considering load
	var effective_cornering_stiffness = cornering_stiffness * normal_load / 4000.0
	var effective_longitudinal_stiffness = longitudinal_stiffness * normal_load / 4000.0

# Critical slip values (where sliding begins)
	var critical_lateral_slip = (friction_coefficient * normal_load) / (2.0 * effective_cornering_stiffness * contact_patch_length)
	var critical_longitudinal_slip = (friction_coefficient * normal_load) / (2.0 * effective_longitudinal_stiffness * contact_patch_length)

# Lateral force (based on slip angle)
	var lateral_force: float
	if abs(slip_angle) < critical_lateral_slip:
	# Elastic region: linear relationship
		lateral_force = -effective_cornering_stiffness * slip_angle
	else:
	# Sliding region: force limited by friction
		lateral_force = -friction_coefficient * normal_load * sign(slip_angle)

	# Longitudinal force (based on slip ratio)
	var longitudinal_force: float
	if abs(slip_ratio) < critical_longitudinal_slip:
	# Elastic region: linear relationship
		longitudinal_force = effective_longitudinal_stiffness * slip_ratio
	else:
		# Sliding region: force limited by friction
		longitudinal_force = friction_coefficient * normal_load * sign(slip_ratio)

	# Combine forces into a Vector2 (longitudinal, lateral)
	tire_forces.x = longitudinal_force
	tire_forces.y = lateral_force

	# Apply friction ellipse to limit combined forces
	var combined_force_magnitude = sqrt(tire_forces.x * tire_forces.x + tire_forces.y * tire_forces.y)
	var max_force = friction_coefficient * normal_load
	if combined_force_magnitude > max_force:
		# Scale forces to stay within friction circle
		tire_forces = tire_forces.normalized() * max_force

	return tire_forces

func brush(slip: Vector2, y_force: float) -> Vector2:
	# 500000 because this line is multiplied by 0.5, while stiffness values are actually in the millions
	# tire_stiffness is a small value just for convenience
	var stiffness = 500000 * tire_stiffness * pow(con_patch, 2)
	var friction = mu * y_force
	# "Brush" tire formula
	var deflect = sqrt(pow(stiffness * slip.y, 2) + pow(stiffness * tan(slip.x), 2))
	if deflect == 0:  return Vector2.ZERO
	else:
		var vector = Vector2.ZERO
		var crit_length = friction * (1 - slip.y) * con_patch / (2 * deflect)
		if crit_length >= con_patch:
			vector.y = stiffness * -slip.y / (1 - slip.y)
			vector.x = stiffness * tan(slip.x) / (1 - slip.y)
		else:
			var brushy = (1 - friction * (1 - slip.y) / (4 * deflect)) / deflect
			vector.y = friction * stiffness * -slip.y * brushy
			vector.x = friction * stiffness * tan(slip.x) * brushy
		return vector

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
	$"../Label".text += "\nGEAR RATIO: %.2f" % [drivetrain.get_gear_ratio(drivetrain.current_gear)]
	$"../Label".text += "\nSPEED: %.2f" % [linear_velocity.length() * 3.6]
	im.surface_set_color(Color.WHITE)
	
	engine_rpm_draw.graph_update(0, engine.angular_velocity * CarEngine.AV_2_RPM)
	
	drivetrain.update_clutch(0.0, drivetrain.current_gear != CarDrivetrain.GEAR_NEUTRAL, engine.angular_velocity, drivetrain.gearbox_angular_vel_to_upstream(drivetrain.differential_angular_vel_to_upstream([wheels[WheelPos.RL].angular_vel, wheels[WheelPos.RR].angular_vel])))
	drivetrain_torques_graph.graph_update(DrivetrainTorquesGraphChannels.CLUTCH_TORQUE, drivetrain.clutch_torque)
	drivetrain_torques_graph.graph_update(DrivetrainTorquesGraphChannels.ENGINE_TORQUE, engine.current_applied_torque)
	var engine_rpm := engine.angular_velocity * CarEngine.AV_2_RPM
	if engine_rpm <= 0.0:
		$AudioStreamPlayer3D.volume_db = linear_to_db(0.0)
	else:
		$AudioStreamPlayer3D.pitch_scale = engine_rpm / 7000.0
		$AudioStreamPlayer3D.volume_db = 0.0
		prints($AudioStreamPlayer3D.get_playback_position(), $AudioStreamPlayer3D.playing, $AudioStreamPlayer3D.pitch_scale)
		if not $AudioStreamPlayer3D.playing:
			$AudioStreamPlayer3D.playing = true
			$AudioStreamPlayer3D.play()
	for wheel_i in range(wheels.size()):
		var wheel := wheels[wheel_i]
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
			var combined_forces := brush(Vector2(x_slip, z_slip), out_spring_force)
			
			var z_force := -combined_forces.y
			var x_force := combined_forces.x
			im.surface_set_color(Color.BLUE)
			apply_force(world_wheel_forward * z_force, result.position - global_position)
			im.surface_add_vertex(result.position)
			im.surface_add_vertex(result.position + world_wheel_forward * z_force)
			im.surface_set_color(Color.WHITE)
			apply_force(world_wheel_right * x_force, result.position - global_position)

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
			var wt := 0.0
			if wheel == wheels[WheelPos.RL]:
				wt = drivetrain.differential_get_downstream_torque(drivetrain.gearbox_get_downstream_torque(drivetrain.clutch_torque))[0]
				net_torque += wt
			if wheel == wheels[WheelPos.RR]:
				wt = drivetrain.differential_get_downstream_torque(drivetrain.gearbox_get_downstream_torque(drivetrain.clutch_torque))[1]
				net_torque += wt
			if Input.is_action_pressed("hb") and not wheel.steerable:
				var brake_torque := min(net_torque + ((wheel.angular_vel * wheel_moment) / delta), 1000.0) as float
				net_torque -= brake_torque
			if Input.is_action_pressed("ui_down"):
				var brake_torque := min(net_torque + ((wheel.angular_vel * wheel_moment) / delta), 500.0) as float
				net_torque -= brake_torque
			wheel.angular_vel += delta * (net_torque / wheel_moment)
			slip_angle_graph.graph_update(wheel_i, rad_to_deg(x_slip))
			slip_ratio_graph.graph_update(wheel_i, rad_to_deg(z_slip))
			tire_forces_longitudinal_graph.graph_update(wheel_i, z_force)
			wheel_torque_graph.graph_update(wheel_i, net_torque)
			
			if wheel.steerable:
				wheel.steer = 0.0
				if Input.is_action_pressed("ui_left"):
					wheel.steer = deg_to_rad(25.0)
				if Input.is_action_pressed("ui_right"):
					wheel.steer = -deg_to_rad(25.0)
	im.surface_end()
