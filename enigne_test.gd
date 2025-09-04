extends Control

var ce := CarEngine.new()

func _physics_process(delta: float) -> void:
	var throttle := 1.0 if Input.is_action_pressed("ui_up") else 0.0
	ce.update(throttle, delta)
	$Label.text = ""
	$Label.text += "%.2f" % [ce.angular_velocity * ce.AV_2_RPM]
	$Label.text += "\n%.2f" % [ce.current_applied_torque]
