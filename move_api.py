# from dronekit import connect, VehicleMode
# from pymavlink import mavutil # 导入 MAVLink 工具
# import time
# import serial
# import serial.tools.list_ports
# import threading

# # --- 导入 Flask ---
# # (需要先安装: pip install Flask)
# from flask import Flask, request, jsonify

# # --- 全局变量 ---
# app = Flask(__name__)
# vehicle = None # 全局无人机对象

# # --- 保留的无人机控制函数 ---

# def send_body_frame_velocity(vehicle_instance, velocity_x, velocity_y, velocity_z, duration_sec):
#     """
#     (此函数将在一个单独的线程中运行)
#     以机体坐标系 (BODY_FRAME) 发送速度指令。
    
#     velocity_x: (m/s) + 前, - 后
#     velocity_y: (m/s) + 右, - 左
#     velocity_z: (m/s) + 下, - 上 (注意：我们通常希望保持高度，所以设为 0)
#     duration_sec: (s) 持续发送指令的时间
#     """
    
#     # 确保我们操作的是有效的 vehicle 实例
#     if not vehicle_instance or not vehicle_instance.armed:
#         print("[API-MOVE] 指令被跳过：无人机未连接或未解锁。")
#         return

#     print(f"[API-MOVE] 开始机动: Fwd={velocity_x}, Right={velocity_y}, Down={velocity_z} for {duration_sec}s")
    
#     msg = vehicle_instance.message_factory.set_position_target_local_ned_encode(
#         0,      # time_boot_ms (not used)
#         0, 0,   # target_system, target_component
#         mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
#         0b0000011111000111, # type_mask (X,Y,Z 速度使能, 忽略其他)
#         0, 0, 0, # x, y, z positions (not used)
#         velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
#         0, 0, 0, # x, y, z acceleration (not used)
#         0, 0)    # yaw, yaw_rate (not used)

#     end_time = time.time() + duration_sec
#     send_interval = 0.1 # 10Hz
    
#     while time.time() < end_time:
#         # 循环检查无人机是否仍然连接和解锁
#         if not vehicle_instance.armed or vehicle_instance.mode.name != "GUIDED":
#             print("[API-MOVE] 机动中断：无人机已上锁或退出 GUIDED 模式。")
#             break
        
#         vehicle_instance.send_mavlink(msg)
#         time.sleep(send_interval)
        
#     # 发送 "停止" 指令 (所有速度为 0)
#     print(f"[API-MOVE] 机动时间到。发送停止指令 (悬停)。")
#     stop_msg = vehicle_instance.message_factory.set_position_target_local_ned_encode(
#         0,      
#         0, 0,   
#         mavutil.mavlink.MAV_FRAME_BODY_NED, 
#         0b0000011111000111, 
#         0, 0, 0, 
#         0, 0, 0, # 速度 X,Y,Z = 0
#         0, 0, 0, 
#         0, 0)
    
#     try:
#         vehicle_instance.send_mavlink(stop_msg)
#     except Exception as e:
#         print(f"[API-MOVE] 发送停止指令时出错: {e}")
    
#     print(f"[API-MOVE] 机动线程结束。")


# def debug_connect_drone():
#     """带详细调试信息的连接函数 (与您原版相同)"""
#     global vehicle
    
#     # 1. 检查串口状态
#     print("=== 串口状态检查 ===")
#     ports = list(serial.tools.list_ports.comports())
    
#     com5_available = any('COM5' in p.device for p in ports)
#     print(f"COM5 可用: {com5_available}")
    
#     if not com5_available:
#         print("错误: 未检测到 COM5。")
#         print("可用串口:")
#         if not ports:
#             print("  (无)")
#         for p in ports:
#             print(f"  {p.device} - {p.description}")
#         return False, "未检测到 COM5"
    
#     # 2. 尝试连接
#     connection_method = {'baud': 57600, 'wait_ready': False, 'desc': '快速连接 (57600)'}
    
#     try:
#         print(f"\n尝试: {connection_method['desc']}, 波特率: {connection_method['baud']}")
#         vehicle_instance = connect(
#             'COM5',
#             baud=connection_method['baud'],
#             wait_ready=False, 
#             timeout=30,
#             heartbeat_timeout=60
#         )
        
#         print("✓ 连接已建立 (MAVLink)")
        
#         print("等待无人机参数初始化 (wait_ready)...")
#         vehicle_instance.wait_ready(True, timeout=30) 
#         print("   ✅ 参数初始化完成")
        
#         vehicle = vehicle_instance # 将实例存储到全局变量
#         return True, "连接成功"
        
#     except Exception as e:
#         print(f"✗ 连接失败: {e}")
#         return False, str(e)

# # --- API 终结点 (Endpoints) ---

# @app.route('/connect', methods=['POST'])
# def api_connect():
#     global vehicle
#     if vehicle:
#         return jsonify({"status": "error", "message": "已经连接"}), 400
    
#     # input("请确保 Mission Planner 已完全关闭 (否则会占用COM5)，然后按Enter键继续...")
    
#     success, message = debug_connect_drone()
    
#     if success:
#         return jsonify({"status": "ok", "message": message})
#     else:
#         return jsonify({"status": "error", "message": message}), 500

# @app.route('/disconnect', methods=['POST'])
# def api_disconnect():
#     global vehicle
#     if not vehicle:
#         return jsonify({"status": "error", "message": "尚未连接"}), 400
        
#     try:
#         vehicle.close()
#         vehicle = None
#         print("[API] 连接已关闭。")
#         return jsonify({"status": "ok", "message": "连接已关闭"})
#     except Exception as e:
#         return jsonify({"status": "error", "message": f"断开连接时出错: {e}"}), 500

# @app.route('/status', methods=['GET'])
# def api_status():
#     if not vehicle:
#         return jsonify({"status": "error", "message": "尚未连接"}), 400
        
#     try:
#         status_data = {
#             "connected": True,
#             "armed": vehicle.armed,
#             "mode": vehicle.mode.name,
#             "altitude_rel": vehicle.location.global_relative_frame.alt,
#             "gps_fix": vehicle.gps_0.fix_type,
#             "ekf_ok": vehicle.ekf_ok,
#             "battery_voltage": vehicle.battery.voltage,
#             "battery_level": vehicle.battery.level,
#             "is_armable": vehicle.is_armable,
#             "heartbeat": vehicle.last_heartbeat
#         }
#         return jsonify({"status": "ok", "data": status_data})
#     except Exception as e:
#         return jsonify({"status": "error", "message": f"获取状态时出错: {e}"}), 500

# @app.route('/arm', methods=['POST'])
# def api_arm():
#     if not vehicle:
#         return jsonify({"status": "error", "message": "尚未连接"}), 400
#     if vehicle.armed:
#         return jsonify({"status": "error", "message": "已经解锁"}), 400

#     try:
#         print("[API] 检查飞前条件...")
#         while not vehicle.is_armable:
#             print("  等待 'is_armable'...")
#             time.sleep(1)
        
#         print("[API] 设置模式为 GUIDED...")
#         vehicle.mode = VehicleMode("GUIDED")
#         while vehicle.mode.name != "GUIDED":
#             print(f"  等待切换到 GUIDED... (当前: {vehicle.mode.name})")
#             time.sleep(0.5)
        
#         print("[API] 解锁 (Arming)...")
#         vehicle.armed = True
#         start_time = time.time()
#         while not vehicle.armed:
#             print("  等待解锁...")
#             if time.time() - start_time > 10:
#                 raise Exception("解锁超时")
#             time.sleep(0.5)
            
#         print("[API]  ✅ 无人机已解锁并进入 GUIDED 模式")
#         return jsonify({"status": "ok", "message": "无人机已解锁 (GUIDED)"})

#     except Exception as e:
#         return jsonify({"status": "error", "message": f"解锁失败: {e}"}), 500

# @app.route('/takeoff', methods=['POST'])
# def api_takeoff():
#     if not vehicle:
#         return jsonify({"status": "error", "message": "尚未连接"}), 400
#     if not vehicle.armed:
#         return jsonify({"status": "error", "message": "无人机未解锁"}), 400
#     if vehicle.mode.name != "GUIDED":
#         return jsonify({"status": "error", "message": "无人机不处于 GUIDED 模式"}), 400

#     data = request.json
#     if not data or 'altitude' not in data:
#         return jsonify({"status": "error", "message": "请求体中必须包含 'altitude' (米)"}), 400
        
#     try:
#         target_altitude = float(data['altitude'])
#         if target_altitude < 1 or target_altitude > 20:
#             return jsonify({"status": "error", "message": "高度必须在 1 到 20 米之间"}), 400

#         print(f"[API] 发送起飞指令至 {target_altitude} 米...")
#         vehicle.simple_takeoff(target_altitude)
        
#         # (注意: simple_takeoff 是非阻塞的。调用者应轮询 /status 来检查高度)
#         return jsonify({"status": "ok", "message": f"已发送起飞指令至 {target_altitude} 米"})

#     except Exception as e:
#         return jsonify({"status": "error", "message": f"起飞指令失败: {e}"}), 500

# @app.route('/move', methods=['POST'])
# def api_move():
#     if not vehicle:
#         return jsonify({"status": "error", "message": "尚未连接"}), 400
#     if not vehicle.armed:
#         return jsonify({"status": "error", "message": "无人机未解锁"}), 400
#     if vehicle.mode.name != "GUIDED":
#         return jsonify({"status": "error", "message": "无人机不处于 GUIDED 模式"}), 400

#     data = request.json
#     if not data:
#         return jsonify({"status": "error", "message": "请求体不能为空"}), 400
    
#     # 从 JSON 获取参数，提供默认值
#     velocity_x = float(data.get('velocity_x', 0.0))
#     velocity_y = float(data.get('velocity_y', 0.0))
#     velocity_z = float(data.get('velocity_z', 0.0)) # +Z = 向下
#     duration = float(data.get('duration', 0.0))

#     if duration <= 0:
#         return jsonify({"status": "error", "message": "'duration' (秒) 必须大于 0"}), 400

#     print(f"[API] 收到 /move 请求。Fwd={velocity_x}, Right={velocity_y}, Down={velocity_z}, Duration={duration}")
    
#     # --- 在新线程中运行阻塞的移动函数 ---
#     move_thread = threading.Thread(
#         target=send_body_frame_velocity, 
#         args=(vehicle, velocity_x, velocity_y, velocity_z, duration)
#     )
#     move_thread.start()
    
#     return jsonify({"status": "ok", "message": "机动指令已在后台开始执行"})

# @app.route('/land', methods=['POST'])
# def api_land():
#     if not vehicle:
#         return jsonify({"status": "error", "message": "尚未连接"}), 400

#     try:
#         print("[API] 设置模式为 LAND...")
#         vehicle.mode = VehicleMode("LAND")
#         return jsonify({"status": "ok", "message": "已发送 LAND 模式指令"})
#     except Exception as e:
#         return jsonify({"status": "error", "message": f"发送 LAND 指令失败: {e}"}), 500


# # --- 主程序 ---
# if __name__ == "__main__":
#     print("="*60)
#     print(" 警告：即将启动一个 Web API 服务器来控制无人机！")
#     print(" 确保您已阅读所有安全警告，并且飞手已准备好随时接管。")
#     print(" (需要先安装: pip install Flask)")
#     print("="*60)
#     print("\n API 服务器将在 http://0.0.0.0:5001 上运行")
#     print(" 您可以使用 API 调试器 (如 Postman) 向以下地址发送请求:")
#     print("   - [POST] http://localhost:5001/connect")
#     print("   - [GET]  http://localhost:5001/status")
#     print("   - [POST] http://localhost:5001/arm")
#     print("   - [POST] http://localhost:5001/takeoff  (Body: {\"altitude\": 3})")
#     print("   - [POST] http://localhost:5001/move     (Body: {\"velocity_x\": 0.5, \"duration\": 4})")
#     print("   - [POST] http://localhost:5001/land")
#     print("   - [POST] http://localhost:5001/disconnect")
    
#     # 运行 Flask 服务器
#     # host='0.0.0.0' 允许从网络上的其他计算机访问
#     app.run(host='0.0.0.0', port=5001, debug=False)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobalRelative # 增加了 LocationGlobalRelative
from pymavlink import mavutil # 导入 MAVLink 工具
import time
import math # 增加 math
import serial
import serial.tools.list_ports
import threading # 保留 import，因为 dronekit 内部可能使用

# --- 导入 Flask ---
from flask import Flask, request, jsonify

# --- 全局变量 ---
app = Flask(__name__)
vehicle = None # 全局无人机对象
# 移除了 mission_lock

# === 从 point_control.py 引入的辅助函数 ===

def get_location_offset_meters(original, dNorth, dEast, alt=None):
    """返回 original 点偏移 dNorth、dEast(米) 后的新 LocationGlobalRelative"""
    earth = 6378137.0
    dLat = dNorth / earth * (180 / math.pi)
    dLon = dEast / (earth * math.cos(math.pi * original.lat / 180)) * (180 / math.pi)
    newLat = original.lat + dLat
    newLon = original.lon + dLon
    return LocationGlobalRelative(newLat, newLon, alt if alt else original.alt)

def get_distance_meters(loc1, loc2):
    """两 GPS 点水平距离（米）"""
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 111139) ** 2 +
                     (dlon * 111139 * math.cos(math.pi * loc1.lat / 180)) ** 2)

def move_relative_meters(vehicle_instance, dForward, dRight, alt=None, wait_for_arrival=True):
    """
    (此函数是阻塞的)
    在机体坐标系下移动（向前、向右），并可选等待到达
    """
    
    # 确保我们操作的是有效的 vehicle 实例
    if not vehicle_instance or not vehicle_instance.armed:
        print("[API-GOTO] 指令被跳过：无人机未连接或未解锁。")
        return
        
    print(f"[API-GOTO]   > 正在移动：向前 {dForward} m, 向右 {dRight} m")
    
    current_loc = vehicle_instance.location.global_relative_frame
    current_alt = alt if alt is not None else current_loc.alt
    
    # 获取当前朝向 (弧度)
    yaw = vehicle_instance.attitude.yaw 
    
    # 将机体坐标系 (forward, right) 转换为全局坐标系 (North, East)
    dNorth = dForward * math.cos(yaw) - dRight * math.sin(yaw)
    dEast  = dForward * math.sin(yaw) + dRight * math.cos(yaw)

    # 计算新的 GPS 目标点
    target_loc = get_location_offset_meters(current_loc, dNorth, dEast, current_alt)
    
    # 发送 GOTO 指令
    vehicle_instance.simple_goto(target_loc)

    # 如果需要，等待到达
    if wait_for_arrival:
        while True:
            # 检查是否仍在 GUIDED 模式，防止模式切换导致死循环
            if not vehicle_instance or vehicle_instance.mode.name != "GUIDED" or not vehicle_instance.armed:
                print("[API-GOTO] ! 模式切换或已上锁，中断 GOTO 等待")
                break
                
            dist = get_distance_meters(vehicle_instance.location.global_relative_frame, target_loc)
            print(f"[API-GOTO]     ... 距目标点 {dist:.2f} m")
            
            # 到达阈值（1米）
            if dist < 1.0: 
                print("[API-GOTO]   > 到达目标点")
                break
            time.sleep(1)
        
        time.sleep(2) # 到达后悬停 2 秒

# --- 原有的无人机控制函数 (与您原始文件一致) ---

def send_body_frame_velocity(vehicle_instance, velocity_x, velocity_y, velocity_z, duration_sec):
    """
    (此函数是阻塞的)
    以机体坐标系 (BODY_FRAME) 发送速度指令。
    """
    
    # 确保我们操作的是有效的 vehicle 实例
    if not vehicle_instance or not vehicle_instance.armed:
        print("[API-MOVE] 指令被跳过：无人机未连接或未解锁。")
        return

    print(f"[API-MOVE] 开始机动: Fwd={velocity_x}, Right={velocity_y}, Down={velocity_z} for {duration_sec}s")
    
    msg = vehicle_instance.message_factory.set_position_target_local_ned_encode(
        0,      # time_boot_ms (not used)
        0, 0,   # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000011111000111, # type_mask (X,Y,Z 速度使能, 忽略其他)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)

    end_time = time.time() + duration_sec
    send_interval = 0.1 # 10Hz
    
    while time.time() < end_time:
        # 循环检查无人机是否仍然连接和解锁
        if not vehicle_instance or not vehicle_instance.armed or vehicle_instance.mode.name != "GUIDED":
            print("[API-MOVE] 机动中断：无人机已上锁或退出 GUIDED 模式。")
            break
        
        vehicle_instance.send_mavlink(msg)
        time.sleep(send_interval)
        
    # 发送 "停止" 指令 (所有速度为 0)
    print(f"[API-MOVE] 机动时间到。发送停止指令 (悬停)。")
    stop_msg = vehicle_instance.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,   
        mavutil.mavlink.MAV_FRAME_BODY_NED, 
        0b0000011111000111, 
        0, 0, 0, 
        0, 0, 0, # 速度 X,Y,Z = 0
        0, 0, 0, 
        0, 0)
    
    try:
        vehicle_instance.send_mavlink(stop_msg)
    except Exception as e:
        print(f"[API-MOVE] 发送停止指令时出错: {e}")
    
    print(f"[API-MOVE] 机动函数结束。")


# --- 新增：正方形任务 (阻塞) ---

def execute_square_mission(vehicle_instance, side_length, alt):
    """
    (此函数是阻塞的)
    执行正方形航点任务。
    """
    
    try:
        print(f"[API-SQUARE] 开始执行正方形任务... 边长: {side_length}m, 高度: {alt}m")
        
        # 0. 设置空速 (可选)
        try:
            vehicle_instance.airspeed = 3 # m/s
        except Exception:
            print("[API-SQUARE] (警告) 设置空速失败，继续执行。")

        # 1. 向前
        print("[API-SQUARE] --- 1. 向前 {} m ---".format(side_length))
        move_relative_meters(vehicle_instance, side_length, 0, alt)

        # 2. 向右
        print("[API-SQUARE] --- 2. 向右 {} m ---".format(side_length))
        move_relative_meters(vehicle_instance, 0, side_length, alt)

        # 3.向右
        print("[API-SQUARE] --- 3. 向右 {} m ---".format(side_length))
        move_relative_meters(vehicle_instance, 0, side_length, alt)

        # # 3. 向右
        print("[API-SQUARE] --- 4. 向右 {} m ---".format(side_length))
        move_relative_meters(vehicle_instance, 0, side_length, alt)
        # print("[API-SQUARE] --- 3. 向后 {} m ---".format(side_length))
        # move_relative_meters(vehicle_instance, -side_length, 0, alt)

        # # 4. 向左 (返回起点)
        # print("[API-SQUARE] --- 4. 向左 {} m ---".format(side_length))
        # move_relative_meters(vehicle_instance, 0, -side_length, alt)

        print("[API-SQUARE] === 正方形任务完成 ===")
        
    except Exception as e:
        print(f"[API-SQUARE] 任务执行出错: {e}")
    
    print("[API-SQUARE] 正方形任务函数结束。")


# --- 连接函数 (与原版相同) ---
def debug_connect_drone():
    """带详细调试信息的连接函数"""
    global vehicle
    
    # 1. 检查串口状态
    print("=== 串口状态检查 ===")
    ports = list(serial.tools.list_ports.comports())
    
    com5_available = any('COM5' in p.device for p in ports)
    print(f"COM5 可用: {com5_available}")
    
    if not com5_available:
        print("错误: 未检测到 COM5。")
        print("可用串口:")
        if not ports:
            print("  (无)")
        for p in ports:
            print(f"  {p.device} - {p.description}")
        return False, "未检测到 COM5"
    
    # 2. 尝试连接
    connection_method = {'baud': 57600, 'wait_ready': False, 'desc': '快速连接 (57600)'}
    
    try:
        print(f"\n尝试: {connection_method['desc']}, 波特率: {connection_method['baud']}")
        vehicle_instance = connect(
           'COM5',
            baud=connection_method['baud'],
            wait_ready=False, 
            timeout=30,
            heartbeat_timeout=60
        )
        
        print("✓ 连接已建立 (MAVLink)")
        
        print("等待无人机参数初始化 (wait_ready)...")
        vehicle_instance.wait_ready(True, timeout=30) 
        print("   ✅ 参数初始化完成")
        
        vehicle = vehicle_instance # 将实例存储到全局变量
        return True, "连接成功"
        
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        return False, str(e)

# --- API 终结点 (Endpoints) ---

@app.route('/connect', methods=['POST'])
def api_connect():
    global vehicle
    if vehicle:
        return jsonify({"status": "error", "message": "已经连接"}), 400
    
    success, message = debug_connect_drone()
    
    if success:
        return jsonify({"status": "ok", "message": message})
    else:
        return jsonify({"status": "error", "message": message}), 500

@app.route('/disconnect', methods=['POST'])
def api_disconnect():
    global vehicle
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
        
    try:
        vehicle.close()
        vehicle = None
        print("[API] 连接已关闭。")
        return jsonify({"status": "ok", "message": "连接已关闭"})
    except Exception as e:
        return jsonify({"status": "error", "message": f"断开连接时出错: {e}"}), 500

@app.route('/status', methods=['GET'])
def api_status():
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
        
    try:
        status_data = {
            "connected": True,
            "armed": vehicle.armed,
            "mode": vehicle.mode.name,
            "altitude_rel": vehicle.location.global_relative_frame.alt,
            "gps_fix": vehicle.gps_0.fix_type,
            "ekf_ok": vehicle.ekf_ok,
            "battery_voltage": vehicle.battery.voltage,
            "battery_level": vehicle.battery.level,
            "is_armable": vehicle.is_armable,
            "heartbeat": vehicle.last_heartbeat
            # 移除了 mission_busy
        }
        return jsonify({"status": "ok", "data": status_data})
    except Exception as e:
        return jsonify({"status": "error", "message": f"获取状态时出错: {e}"}), 500

@app.route('/arm', methods=['POST'])
def api_arm():
    # ... (此函数不变) ...
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
    if vehicle.armed:
        return jsonify({"status": "error", "message": "已经解锁"}), 400
    try:
        print("[API] 检查飞前条件...")
        while not vehicle.is_armable:
            print("  等待 'is_armable'...")
            time.sleep(1)
        print("[API] 设置模式为 GUIDED...")
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            print(f"  等待切换到 GUIDED... (当前: {vehicle.mode.name})")
            time.sleep(0.5)
        print("[API] 解锁 (Arming)...")
        vehicle.armed = True
        start_time = time.time()
        while not vehicle.armed:
            print("  等待解锁...")
            if time.time() - start_time > 10:
                raise Exception("解锁超时")
            time.sleep(0.5)
        print("[API]  ✅ 无人机已解锁并进入 GUIDED 模式")
        return jsonify({"status": "ok", "message": "无人机已解锁 (GUIDED)"})
    except Exception as e:
        return jsonify({"status": "error", "message": f"解锁失败: {e}"}), 500

@app.route('/takeoff', methods=['POST'])
def api_takeoff():
    # ... (此函数不变，它本身就是非阻塞的) ...
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
    if not vehicle.armed:
        return jsonify({"status": "error", "message": "无人机未解锁"}), 400
    if vehicle.mode.name != "GUIDED":
        return jsonify({"status": "error", "message": "无人机不处于 GUIDED 模式"}), 400
    data = request.json
    if not data or 'altitude' not in data:
        return jsonify({"status": "error", "message": "请求体中必须包含 'altitude' (米)"}), 400
    try:
        target_altitude = float(data['altitude'])
        if target_altitude < 1 or target_altitude > 20:
            return jsonify({"status": "error", "message": "高度必须在 1 到 20 米之间"}), 400
        print(f"[API] 发送起飞指令至 {target_altitude} 米...")
        vehicle.simple_takeoff(target_altitude)
        return jsonify({"status": "ok", "message": f"已发送起飞指令至 {target_altitude} 米"})
    except Exception as e:
        return jsonify({"status": "error", "message": f"起飞指令失败: {e}"}), 500

@app.route('/move', methods=['POST'])
def api_move():
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
    if not vehicle.armed:
        return jsonify({"status": "error", "message": "无人机未解锁"}), 400
    if vehicle.mode.name != "GUIDED":
        return jsonify({"status": "error", "message": "无人机不处于 GUIDED 模式"}), 400

    data = request.json
    if not data:
        return jsonify({"status": "error", "message": "请求体不能为空"}), 400
    
    velocity_x = float(data.get('velocity_x', 0.0))
    velocity_y = float(data.get('velocity_y', 0.0))
    velocity_z = float(data.get('velocity_z', 0.0)) # +Z = 向下
    duration = float(data.get('duration', 0.0))

    if duration <= 0:
        return jsonify({"status": "error", "message": "'duration' (秒) 必须大于 0"}), 400

    print(f"[API] 收到 /move 请求。Fwd={velocity_x}, Right={velocity_y}, Down={velocity_z}, Duration={duration}")
    
    # --- 直接调用阻塞函数 ---
    send_body_frame_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration)
    
    return jsonify({"status": "ok", "message": "机动指令已执行完毕"})

@app.route('/land', methods=['POST'])
def api_land():
    # ... (此函数不变，它本身就是非阻塞的) ...
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
    try:
        print("[API] 设置模式为 LAND...")
        vehicle.mode = VehicleMode("LAND")
        return jsonify({"status": "ok", "message": "已发送 LAND 模式指令"})
    except Exception as e:
        return jsonify({"status": "error", "message": f"发送 LAND 指令失败: {e}"}), 500


# === 新增 API (同步阻塞版) ===

@app.route('/fly_square', methods=['POST'])
def api_fly_square():
    """
    【新增】API: 执行正方形航线任务 (同步阻塞)
    """
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
    if not vehicle.armed:
        return jsonify({"status": "error", "message": "无人机未解锁"}), 400
    if vehicle.mode.name != "GUIDED":
        return jsonify({"status": "error", "message": "无人机不处于 GUIDED 模式"}), 400

    data = request.json
    if not data:
        return jsonify({"status": "error", "message": "请求体不能为空"}), 400

    side_length = float(data.get('side_length', 5.0)) # 默认为 5 米
    altitude = float(data.get('altitude', 0.0)) # 0.0 表示保持当前高度

    if side_length <= 0:
        return jsonify({"status": "error", "message": "'side_length' (米) 必须大于 0"}), 400

    current_alt = vehicle.location.global_relative_frame.alt
    if altitude == 0.0:
        altitude = current_alt # 保持当前高度
    elif altitude < 1.0:
        return jsonify({"status": "error", "message": "目标 'altitude' (米) 必须大于 1"}), 400
        
    print(f"[API] 收到 /fly_square 请求。边长={side_length}, 目标高度={altitude}")
    
    # --- 直接调用阻塞函数 ---
    execute_square_mission(vehicle, side_length, altitude)
    
    return jsonify({"status": "ok", "message": f"正方形任务 (边长 {side_length}m) 已执行完毕"})


@app.route('/rotate_yaw', methods=['POST'])
def api_rotate_yaw():
    """
    【新增】API: 相对旋转偏航角 (左/右)
    (此函数是非阻塞的，MAVLink 指令会立即返回)
    """
    if not vehicle:
        return jsonify({"status": "error", "message": "尚未连接"}), 400
    if not vehicle.armed:
        return jsonify({"status": "error", "message": "无人机未解锁"}), 400
    if vehicle.mode.name != "GUIDED":
        return jsonify({"status": "error", "message": "无人机不处于 GUIDED 模式"}), 400

    data = request.json
    if not data:
        return jsonify({"status": "error", "message": "请求体不能为空"}), 400

    angle_deg = float(data.get('angle', 0.0))
    direction = data.get('direction', '') # 'left' 或 'right'
    speed_deg_s = float(data.get('speed', 30.0)) # 默认 30 度/秒

    if angle_deg <= 0:
        return jsonify({"status": "error", "message": "'angle' (度) 必须大于 0"}), 400
    if direction not in ['left', 'right']:
        return jsonify({"status": "error", "message": "'direction' 必须是 'left' 或 'right'"}), 400

    direction_val = -1.0 if direction == 'left' else 1.0
    
    print(f"[API-YAW] 发送旋转指令: {direction} {angle_deg} 度, 速度 {speed_deg_s} deg/s")

    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,       # confirmation
        angle_deg,   # param1: 旋转量 (度)
        speed_deg_s, # param2: 旋转速度 (度/秒)
        direction_val, # param3: 方向
        1,       # param4: 1 = 相对模式
        0, 0, 0) # param 5, 6, 7 (not used)

    try:
        vehicle.send_mavlink(msg)
        
        # (可选) 增加一个短暂的等待，以确保指令被执行
        # MAV_CMD_CONDITION_YAW 是非阻塞的，但我们可以估算一个完成时间
        wait_time = abs(angle_deg / speed_deg_s) + 1.0 # 估算时间 + 1秒缓冲
        print(f"[API-YAW] 等待旋转完成 (约 {wait_time:.1f} 秒)...")
        time.sleep(wait_time) 
        print(f"[API-YAW] 旋转等待结束。")
        
        return jsonify({"status": "ok", "message": f"已发送 {direction} 旋转 {angle_deg} 度指令并等待 {wait_time:.1f} 秒"})
    except Exception as e:
        return jsonify({"status": "error", "message": f"发送 YAW 指令失败: {e}"}), 500


# --- 主程序 (已更新 API 列表) ---
if __name__ == "__main__":
    print("="*60)
    print(" 警告：即将启动一个 Web API 服务器来控制无人机！")
    print(" (版本：同步/阻塞模式，适用于单步调试)")
    print(" (警告：长时间任务 (如 fly_square) 可能会导致客户端超时!)")
    print("="*60)
    print("\n API 服务器将在 http://0.0.0.0:5001 上运行")
    print("   - [POST] http://localhost:5001/connect")
    print("   - [GET]  http://localhost:5001/status")
    print("   - [POST] http://localhost:5001/arm")
    print("   - [POST] http://localhost:5001/takeoff    (Body: {\"altitude\": 3})")
    print("   - [POST] http://localhost:5001/move       (Body: {\"velocity_x\": 0.5, \"duration\": 4})")
    print("   --- 新增 API ---")
    print("   - [POST] http://localhost:5001/fly_square (Body: {\"side_length\": 5, \"altitude\": 5})")
    print("   - [POST] http://localhost:5001/rotate_yaw (Body: {\"angle\": 90, \"direction\": \"right\"})")
    print("   --- 结束任务 ---")
    print("   - [POST] http://localhost:5001/land")
    print("   - [POST] http://localhost:5001/disconnect")
    
    # 运行 Flask 服务器
    # 注意：debug=False 在生产/实际飞行中很重要
    # app.run(host='0.0.0.0', port=5001, debug=False)
    
    # 调试时：
    # 如果您希望在服务器阻塞时也能处理其他请求（如 /status），
    # 您需要启用线程，但这会使调试变得复杂。
    # 保持单线程模式 (默认) 最符合您的“一步一步”请求。
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=False)
