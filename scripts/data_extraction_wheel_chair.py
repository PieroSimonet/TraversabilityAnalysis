#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
from nav_msgs.msg import Odometry


class MultiCameraImageSubscriber:
    def __init__(self, camera_count):
        rospy.init_node('multi_camera_image_subscriber', anonymous=True)
        self.output_template = rospy.get_param("output_format", "{}/{:04d}")

        self.cv_image = []
        for i in range(camera_count):
            self.cv_image.append(Image())

        # Initialize variables
        self.bridge = CvBridge()
        self.camera_count = camera_count
        self.counter = [0]*self.camera_count
        self.folders = [f"data/camera{i}" for i in range(1, camera_count + 1)]
        self.odom = None
        self.new_data = False

        # Set up folders for images
        for folder in self.folders:
            if not os.path.exists(folder + "/image"):
                os.makedirs(folder + "/image")
            if not os.path.exists(folder + "/pose"):
                os.makedirs(folder + "/pose")

        # Save the intrinsics
        for i in range(camera_count):
            self.camera_info = rospy.wait_for_message("/opt_0"+str(i+1)+"/camera/camera/color/camera_info", CameraInfo)
            self.callback_info(self.camera_info, i)

        #for i in range(camera_count):
        #    self.image_sub = rospy.Subscriber("/opt_0"+str(i+1)+"/camera/camera/color/image_raw", Image, self.callback_image, callback_args=i)

        self.image_sub = rospy.Subscriber("/opt_01"+"/camera/camera/color/image_raw", Image, self.callback_image1, callback_args=1)
        self.image_sub = rospy.Subscriber("/opt_02"+"/camera/camera/color/image_raw", Image, self.callback_image2, callback_args=2)
        #self.image_sub = rospy.Subscriber("/opt_03"+"/camera/camera/color/image_raw", Image, self.callback_image2, callback_args=3)

        # Subscribe to odometry topic
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #self.odom_sub = rospy.Subscriber("/opt_01/kinect2/rgb/image_raw", Image, self.odom_callback)
    
    def callback_image1(self, data, args):
        self.cv_image[args - 1] = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.new_data = True

    def run(self):
        while not rospy.is_shutdown():
            if self.new_data:
                cv2.imshow("Image window", self.cv_image[0])

                pressed_key = cv2.waitKey(0)

                args = []
                for i in range(self.camera_count):
                    args.append(i)

                if pressed_key == ord('s'):  
                    for arg in args:
                        output_name = self.folders[arg] + f"/image/{self.counter[arg]:04d}.png"
                        cv2.imwrite(output_name, self.cv_image[arg])
                        print("Saved image to %s" % output_name)
                        if self.odom is not None:
                            pose = self.odom.pose.pose
                            print(pose)
                            rototranslation_matrix1 = self.get_rototranslation_matrix(pose)
                            #translation_matrix = np.array([[1, 0, 0, 0],
                            #                       [0, 1, 0, 0],
                            #                       [0, 0, 1, 0],
                            #                       [0, 0, 0, 1]])
                            self.save_pose(rototranslation_matrix1, self.folders[arg] + "/pose/", arg)

                        self.counter[arg] += 1        

        
        

    def callback_image2(self, data, args):
        self.cv_image[args - 1] = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def callback_info(self, data, args):
        # Get the parameters info
        self.output_intrinsic_par = self.folders[args] + "/intrinsic_pars_file.yaml" 
        camera_info_K = np.array(data.K).reshape([3,3])
        camera_info_D = np.array(data.D)
        camera_info_height = data.height
        camera_info_width = data.width

        # Create the file yml
        file_content = "fx: {}\nfy: {}\ncx: {}\ncy: {}\nhas_dist_coeff: {}\ndist_k0: {}\ndist_k1: {}\ndist_px: {}\ndist_py: {}\ndist_k2: {}\ndist_k3: {}\ndist_k4: {}\ndist_k5: {}\nimg_width: {}\nimg_height: {}".format(camera_info_K[0,0],camera_info_K[1,1],camera_info_K[0,2],camera_info_K[1,2],1,camera_info_D[0],camera_info_D[1],camera_info_D[2],camera_info_D[3],camera_info_D[4],0,0,0,camera_info_width,camera_info_height)

        with open(self.output_intrinsic_par, 'w') as file:
            file.write(file_content)

    def odom_callback(self, msg):
        self.odom = msg
        #print("ODOM CHANGED")
        

    def save_pose(self, matrix, folder, camera_num):
        file_path = os.path.join(folder, f"{self.counter[camera_num]:04d}.csv")
        np.savetxt(file_path, matrix, delimiter=",")
        rospy.loginfo(f"Saved pose: {file_path}")

    def get_rototranslation_matrix(self, pose):
        
        # Extract necessary information from pose and camera_info
        position = pose.position
        orientation = pose.orientation

        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation)

        # Construct the 4x4 homogeneous transformation matrix
        translation_matrix = np.array([[1, 0, 0, position.x],
                                       [0, 1, 0, position.y],
                                       [0, 0, 1, position.z],
                                       [0, 0, 0, 1]])
    
        rototranslation_matrix = np.dot(rotation_matrix, translation_matrix)
        return rototranslation_matrix 

    def quaternion_to_rotation_matrix(self, quaternion):
            x = quaternion.x
            y = quaternion.y
            z = quaternion.z
            w = quaternion.w

            rotation_matrix = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w,0],
                                        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w,0],
                                        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2,0],
                                        [0,0,0,1]])

            return rotation_matrix


if __name__ == '__main__':
    try:
        # Set the number of cameras (change this value as needed)
        num_cameras = 2
        multi_camera_subscriber = MultiCameraImageSubscriber(num_cameras)
        multi_camera_subscriber.run()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
