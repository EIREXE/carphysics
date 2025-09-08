extends Control

@onready var throttle_pb: ProgressBar = $HBoxContainer/ProgressBar
@onready var brake_pb: ProgressBar = $HBoxContainer/ProgressBar2
@onready var clutch_pb: ProgressBar = $HBoxContainer/ProgressBar3

var throttle:
	set(val):
		throttle_pb.value = val

var brake:
	set(val):
		brake_pb.value = val

var clutch:
	set(val):
		clutch_pb.value = val

func _ready() -> void:
	for i in [throttle_pb, brake_pb, clutch_pb]:
		i.max_value = 1.0
		i.step = 0.0
