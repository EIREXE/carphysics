extends Control

class_name EngineRPMGraph

var graph_points := PackedFloat32Array()

func _init() -> void:
	graph_points.resize(128)

func graph_update(new_rpm: float):
	for i in range(graph_points.size()-1):
		graph_points[i] = graph_points[i+1]
	graph_points[-1] = new_rpm
	queue_redraw()

func _draw() -> void:
	var max_rpm := 7000.0
	draw_rect(Rect2(Vector2.ZERO, size), Color.BLACK)
	
	var polyline := PackedVector2Array()
	polyline.resize(graph_points.size())
	
	for i in range(polyline.size()):
		var x := (i / float(graph_points.size()-1)) * size.x
		var y := size.y - (graph_points[i] / max_rpm) * size.y
		polyline[i] = Vector2(x, y)
	draw_polyline(polyline, Color.WHITE)
