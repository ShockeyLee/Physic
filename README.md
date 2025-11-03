# Physic
Physic experiment  Code 
物理实验测试代码
第一阶段 指令控制无人机元动作执行
原动作包括：{连接无人机、解锁、起飞、降落、上升、下降、[向前、向后、向左、向右]移动、左右旋转、航点飞行等}
保持航向的一些参数设置
```python
msg = vehicle_instance.message_factory.set_position_target_local_ned_encode(
        0,      # time_boot_ms (not used)
        0, 0,   # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000011111000111, # type_mask (X,Y,Z 速度使能, 忽略其他)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)
```
mavutil.mavlink.MAV_FRAME_BODY_NED
0b0000011111000111
