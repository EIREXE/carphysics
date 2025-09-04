extends Node3D

var mouse_motion: Vector2

func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		mouse_motion += event.relative

func _process(delta: float) -> void:
	rotate_y(-mouse_motion.x * 0.01)
	rotate_object_local(Vector3.RIGHT, -mouse_motion.y * 0.01)
	
	mouse_motion = Vector2.ZERO
