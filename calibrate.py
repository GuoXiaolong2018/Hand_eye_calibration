#-*- coding:utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import absolutu_Orientation_Quaternion as aoq
import DobotDllType as dType

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

#dobot init
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()

#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])
if state != dType.DobotConnect.DobotConnect_NoError:
    print("Error Connecting Dobot!")

camera = []
world = []

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        if not depth_frame or not color_frame:
            continue

        print("请移动二维码标志")
        if input()=="q":
            break
        # Detect aruco in image, then return its
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()

        #lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if len(corners)!=0:
            point = np.average(corners[0][0], axis=0)
            depth = depth_frame.get_distance(point[0], point[1])
            xc,yc,zc=rs.rs2_deproject_pixel_to_point(color_intrinsics, [point[0], point[1]], depth)
            print("相机坐标系中位置:{},{},{}".format(1000*xc,1000*yc,1000*zc))
            camera.append([1000*xc,1000*yc,1000*zc])
        else:
            print("未检测到二维码标志!")
            continue

        print("请将机械手移动到二维码标志上方")
        input()
        pos = dType.GetPose(api)
        xw = pos[0]
        yw = pos[1]
        zw = pos[2]
        world.append([xw,yw,zw])
        print("世界坐标系中位置:{},{},{}".format(xw,yw,zw))
finally:
    camera = np.array(camera).T
    world = np.array(world).T
    doScale = 1
    s,R,T = aoq.absoluti_Orirentation_Quaternion(camera,world,doScale)
    print("s=",s)
    print("R=",R)
    print("T=",T)
    # Stop streaming
    pipeline.stop()
    dType.DisconnectDobot(api)
