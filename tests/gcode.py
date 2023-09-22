import bpy
import gcodeparser as gp
import numpy as np



def move_quick(x,y,z,speed_in_mm_per_sec):
    global cur_pos,cur_speed,cur_time,axis_x,axis_y,axis_z
    if x: x /= 1000
    if y: y /= 1000
    if z: z /= 1000

    if not x: x = cur_pos[0]
    if not y: y = cur_pos[1]
    if not z: z = cur_pos[2]
    if not speed_in_mm_per_sec: speed_in_mm_per_sec = cur_speed * 1000
    cur_speed = speed_in_mm_per_sec / 1000
    distance = np.linalg.norm(np.array(cur_pos) - [x,y,z])
    seconds = distance / cur_speed
    frames = round(seconds * 240)
    cur_frame = round(cur_time * 240)

    axis_x.location.x += x - cur_pos[0]
    axis_y.location.y += y - cur_pos[1]
    axis_z.location.z += z - cur_pos[2]
    bpy.context.scene.frame_set(cur_frame + frames)
    axis_x.keyframe_insert(data_path='location',frame=cur_frame + frames,index=0)
    axis_y.keyframe_insert(data_path='location',frame=cur_frame + frames,index=1)
    axis_z.keyframe_insert(data_path='location',frame=cur_frame + frames,index=2)
    
    cur_time += seconds
    cur_pos = [x,y,z]

def move_linear(x,y,z,speed_in_mm_per_sec,P,V):
    global cur_pos,cur_speed,cur_time,axis_x,axis_y,axis_z
    if x: x /= 1000
    if y: y /= 1000
    if z: z /= 1000
    if not x: x = cur_pos[0]
    if not y: y = cur_pos[1]
    if not z: z = cur_pos[2]
    if not speed_in_mm_per_sec: speed_in_mm_per_sec = cur_speed * 1000
    cur_speed = speed_in_mm_per_sec / 1000
    distance = np.linalg.norm(np.array(cur_pos) - [x,y,z])
    seconds = distance / cur_speed
    frames = round(seconds * 240)
    cur_frame = round(cur_time * 240)
    axis_x.location.x += x - cur_pos[0]
    axis_y.location.y += y - cur_pos[1]
    axis_z.location.z += z - cur_pos[2]
    bpy.context.scene.frame_set(cur_frame + frames)
    axis_x.keyframe_insert(data_path='location',frame=cur_frame + frames,index=0)
    axis_y.keyframe_insert(data_path='location',frame=cur_frame + frames,index=1)
    axis_z.keyframe_insert(data_path='location',frame=cur_frame + frames,index=2)

    cur_time += seconds
    cur_pos = [x,y,z]
    pass

def set_pinch_roller_pose(W):
    pass

if __name__ == "__main__":
    global cur_pos,cur_speed,cur_time,axis_x,axis_y,axis_z
    cur_extrusion_length = 0
    cur_pos = [0,0,0]
    cur_speed = 1500
    cur_time = 0

    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = 10000

    axis_x=bpy.data.objects["axis_x"]
    axis_y=bpy.data.objects["axis_y"]
    axis_z=bpy.data.objects["axis_z"]
    cur_frame = round(cur_time * 240)
    axis_x.keyframe_insert(data_path='location',frame=cur_frame,index=0)
    axis_y.keyframe_insert(data_path='location',frame=cur_frame,index=1)
    axis_z.keyframe_insert(data_path='location',frame=cur_frame,index=2)

    start_x = axis_x.location.x
    start_y = axis_y.location.y
    start_z = axis_z.location.z
    
    with open('C:/Users/SLTru/Desktop/圆锥体_G50_6_Ender-PLA_31m.gcode', 'r') as f:
        gcode = f.read()

    command = gp.GcodeParser(gcode,True)
    for line in command.lines: 
        if line.type == gp.Commands.COMMENT:
            continue
        print(line.type,line.command,line.params)
        if line.type == gp.Commands.OTHER:
            if line.command_str == 'G90' and 'W' in line.params:
                set_pinch_roller_pose(line.get_param('W'))
            if line.command_str == 'G28':
                move_quick(line.get_param('X'),
                        line.get_param('Y'),
                        line.get_param('Z'),
                        line.get_param('F'))
        elif line.type == gp.Commands.MOVE:
            if line.command_str == 'G0':
                move_quick(line.get_param('X'),
                        line.get_param('Y'),
                        line.get_param('Z'),
                        line.get_param('F'))
            if line.command_str == 'G1':
                move_linear(line.get_param('X'),
                            line.get_param('Y'),
                            line.get_param('Z'),
                            line.get_param('F'),
                            line.get_param('P'),
                            line.get_param('V'))
        import time
        
        cur_frame = round(cur_time * 240)
        print('time',cur_time,cur_frame)

# i = 0
# while True:
#     i+=1
#     bpy.context.scene.frame_set(i)
#     bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

axis_x.location.x = start_x
axis_y.location.y = start_y
axis_z.location.z = start_z