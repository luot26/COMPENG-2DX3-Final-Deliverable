import serial
import math
import open3d as o3d

def wait_for_scan():
    while True:
        raw = s.readline()
        message = raw.decode().strip()
        if message == "start scanning": 
            return True
        elif message == "stop": 
            return False
        else:
            try:
                distance = float(message)
                return distance
            except ValueError:
                continue

def create_line_set(points): 
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    lines = []

    num_planes = len(points) // 32 
    for p in range(num_planes):
        for i in range(32):
            if i < points_per_plane - 1:
                lines.append([p * 32 + i, p * 32 + i + 1])
            else:
                lines.append([p * 32 + i, p * 32]) 
            if p < num_planes - 1:  
                lines.append([p * 32 + i, (p + 1) * 32 + i])

        lines.append([p * 32, p * 32 + 32 - 1]) 
    line_set.lines = o3d.utility.Vector2iVector(lines)
    
    return line_set

def create_point_cloud(points): 
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def plot_point_cloud(point_cloud, line_set): 
    o3d.visualization.draw_geometries([point_cloud, line_set])

def cartesian_convert(distance, angle):
    x = depth                                    
    y = distance * math.cos(math.radians(angle)) 
    z = distance * math.sin(math.radians(angle)) 
    return x, y, z


s = serial.Serial('COM4', 115200, timeout = 10)
print("Opening: " + s.name)

s.reset_output_buffer()
s.reset_input_buffer()

f = open("2dx3points.xyz", "w") 

angle_increment = 360 / 32

input("Press Enter to start communication...")
s.write('s'.encode())
points = []

i = 0
depth = 0
scans = 0
points_per_plane = 32 

while scans < 3:

    result = wait_for_scan()
    if result == True: 
        continue
    elif result == False: 
        break
    else:
        distance = result
        angle = i * angle_increment
        
        
        if distance!= 9999:

            x, y, z = cartesian_convert(distance, angle) 
            prev_x, prev_y, prev_z = x, y, z
            print(f"x: {x}, y: {y}, z: {z}")

            points.append((x, y, z))
            f.write(f"{x} {y} {z}\n")

            i += 1

            if i % 32 == 0 and i != 0: 
                depth += 500
                scan += 1
                print("Scan Finished")
        else:
            print(f"x: {x}, y: {y}, z: {z}")

            points.append((x, y, z))
            f.write(f"{x} {y} {z}\n")

            i += 1

            if i % 32 == 0 and i != 0: 
                depth += 500
                scan += 1
                print("Scan Finished")

print("Closing: " + s.name) 
s.close()
f.close()

point_cloud = create_point_cloud(points) 
line_set = create_line_set(points)
lines = line_set.lines
plot_point_cloud(point_cloud, line_set)
