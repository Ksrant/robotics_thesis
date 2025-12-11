import cv2
import numpy as np
import pyrealsense2 as rs
import torch

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Set up the RealSense D455 camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
profile=pipeline.get_active_profile()
depth_profile=profile.get_stream(rs.stream.depth)
depth_intrinsics=depth_profile.as_video_stream_profile().get_intrinsics()
#write your yolov5 depth scale here

depth_scale = 0.0010000000474974513

spatial=rs.spatial_filter()
temporal=rs.temporal_filter()
# Main loop
while True:
    
    # Get the latest frame from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Convert the frames to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    depth_frame=spatial.process(depth_frame)
    depth_frame=temporal.process(depth_frame)

    # Convert the color image to grayscale
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Convert the depth image to meters
    depth_image = depth_image * depth_scale

    # Detect objects using YOLOv5
    results = model(color_image)

    # Process the results
    for result in results.xyxy[0]:
        x1, y1, x2, y2, confidence, class_id = result

        # Calculate the distance to the object
        object_depth = np.median(depth_image[int(y1):int(y2), int(x1):int(x2)])
        label = f"{object_depth:.2f}m"

        cx=int((x1+x2)/2)
        cy=int((y1+y2)/2)

        Z=depth_image[cy,cx]

        X,Y,Z=rs.rs2_deproject_pixel_to_point(depth_intrinsics,[cx,cy],Z)



        # Draw a rectangle around the object
        cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (252, 119, 30), 2)

        # Draw the bounding box
        cv2.putText(color_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)
        object_positions={}
        object_positions[int(class_id)]=(X,Y,Z)
        # Print the object's class and distance
        print(f"{model.names[int(class_id)]}: {object_depth:.2f}m")
        print(f"{model.names[int(class_id)]}:X={X},Y={Y},Z={Z}")
        
    # Show the image
    cv2.imshow("Color Image", color_image)
    cv2.waitKey(1)

# Release the VideoWriter object
out.release()
