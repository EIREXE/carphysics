extends Control

class_name TelemetryGraph

var graph_points: Array[PackedFloat32Array]

var channel_names: Array[String]
var channel_colors: Array[Color]

var limit_min: float = 0.0
var limit_max: float = 7000.0

const MARGIN := 50.0

var legend_items_container: GridContainer

func set_limits(_limit_min: float, _limit_max: float):
	limit_min = _limit_min
	limit_max = _limit_max

func set_graph_channels(channel_count: int, sample_count := 128):
	graph_points.resize(channel_count)
	for i in range(graph_points.size()):
		var gp := graph_points[i]
		gp.resize(sample_count)
		graph_points[i] = gp
	channel_names.resize(channel_count)
	channel_colors.resize(channel_count)
	
	for i in range(channel_colors.size()):
		channel_colors[i] = Color.from_hsv((i / float(channel_colors.size() -1)) * 0.5, 1.0, 1.0)
	update_legend()

func channel_set_name(channel: int, new_name: String):
	channel_names[channel] = new_name
	update_legend()

func update_legend():
	for child in legend_items_container.get_children():
		child.queue_free()
	
	for channel in range(channel_names.size()):
		var hbox := HBoxContainer.new()
		var color_rect := ColorRect.new()
		color_rect.color = channel_colors[channel]
		color_rect.custom_minimum_size = Vector2(24, 24)
		var label := Label.new()
		label.text = channel_names[channel]
		hbox.add_child(label)
		hbox.add_child(color_rect)
		hbox.alignment = BoxContainer.ALIGNMENT_END
		
		legend_items_container.add_child(hbox)

func _ready() -> void:
	var legend_container := MarginContainer.new()
	legend_container.add_theme_constant_override(&"margin_left", MARGIN)
	legend_container.add_theme_constant_override(&"margin_right", MARGIN)
	legend_container.add_theme_constant_override(&"margin_top", MARGIN)
	legend_container.add_theme_constant_override(&"margin_bottom", MARGIN + 2)
	add_child(legend_container)
	legend_container.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var hbox := HBoxContainer.new()
	hbox.alignment = BoxContainer.ALIGNMENT_END
	
	legend_container.add_child(hbox)
	var vbox := VBoxContainer.new()
	hbox.add_child(vbox)
	vbox.alignment = BoxContainer.ALIGNMENT_END
	legend_items_container = GridContainer.new()
	legend_items_container.columns = 2
	vbox.add_child(legend_items_container)

func graph_update(channel: int, new_value: float):
	for i in range(graph_points[channel].size()-1):
		graph_points[channel][i] = graph_points[channel][i+1]
	graph_points[channel][-1] = new_value
	queue_redraw()

func _draw() -> void:
	var max_rpm := 7000.0
	draw_rect(Rect2(Vector2.ZERO, size), Color.BLACK * Color(1.0, 1.0, 1.0, 0.5))

	
	var usable_rect := Rect2(Vector2(MARGIN, MARGIN), size - Vector2(MARGIN, MARGIN) * 2.0)
	
	if limit_min != 0.0:
		var center_y := inverse_lerp(limit_min, limit_max, 0.0) * usable_rect.size.y + usable_rect.position.y
		draw_line(Vector2(usable_rect.position.x, center_y), Vector2(usable_rect.position.x + usable_rect.size.x, center_y), Color.GRAY)
	
	for channel in graph_points.size():
		var polyline := PackedVector2Array()
		polyline.resize(graph_points[channel].size())
		
		for i in range(polyline.size()):
			var x := (i / float(graph_points[channel].size()-1))
			var y := 1.0 - inverse_lerp(limit_min, limit_max, graph_points[channel][i])
			polyline[i] = usable_rect.position + usable_rect.size * Vector2(x, y)
		draw_string(get_theme_default_font(), polyline[-1], "%.2f" % [graph_points[channel][-1]])
		draw_polyline(polyline, channel_colors[channel], 1.0, true)
	draw_line(usable_rect.position, usable_rect.position + Vector2(0.0, usable_rect.size.y + MARGIN), Color.WHITE, 2.0)
	draw_line(usable_rect.position + Vector2(-MARGIN, usable_rect.size.y), usable_rect.position + Vector2(usable_rect.size.x + MARGIN, usable_rect.size.y), Color.WHITE, 2.0)

	var bottom_left_corner := Vector2(7.0, usable_rect.position.y + usable_rect.size.y - 15.0)
	draw_string(get_theme_default_font(), Vector2(0, usable_rect.position.y), "%.0f" % limit_max, HORIZONTAL_ALIGNMENT_RIGHT)
	draw_string(get_theme_default_font(), bottom_left_corner, "%.0f" % limit_min, HORIZONTAL_ALIGNMENT_RIGHT)
