import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
center = np.array([0.,0.,0.])
center_pre = np.array([0.,0.,0.])
camera_cord = np.zeros((4,1))
camera_cord[3] = 1

camera_to_arm = np.array([[ 4.73727030e-02,  6.78473559e-01,  6.60859162e-02,  2.23936619e+02],
                        [ 5.71950157e-01, -6.67840988e-02, -4.11233731e-01,  1.98663986e+02],
                        [ 1.33533277e-02, -6.37026535e-01, -4.70856506e+00,  3.13087141e+03]])

x_offset = 209.0232 - 202.0152
y_offset = -18.6872 + 14.8884
z_offset = 131.7800 - 57.6675

offset = np.array([[x_offset],[y_offset],[z_offset]])

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # get intrinsic of color image
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if len(corners)!=0:
            point = np.average(corners[0][0], axis=0)
            depth = depth_frame.get_distance(point[0], point[1])
            point = np.append(point,depth)

            if depth!=0:
                x=point[0]
                y=point[1]
                z=point[2]
                ## see rs2 document: 
                ## https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
                ## and example: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
                x,y,z=rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], z)
                center[0] = x
                center[1] = y
                center[2] = z
                #print('center:', center)
                #print('center_pre', center_pre)
                distance = np.sum(np.square(1000*center-1000*center_pre))
                if(distance>=10):
                    #print('Coordinate in camera: ({},{},{})'.format(1000*x,1000*y,1000*z))
                    center_pre[0] = center[0]
                    center_pre[1] = center[1]
                    center_pre[2] = center[2]
                    camera_cord[0] = 1000*x
                    camera_cord[1] = 1000*y
                    camera_cord[2] = 1000*z
                    arm_cord = np.dot(camera_to_arm, camera_cord)
                    print(arm_cord)
                color_image = aruco.drawDetectedMarkers(color_image, corners)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:

    # Stop streaming
    pipeline.stop()

