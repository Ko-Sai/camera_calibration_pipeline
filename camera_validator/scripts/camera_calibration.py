import numpy as np
from collections import OrderedDict
import cv2 
import glob
import yaml
import sys
import os
import subprocess

class Cam_capture(object):
    def __init__(self, camera_list, input_image_file, cam_folder_name, output_yaml_file):

        self.row                 = 13
        self.col                 = 9
        self.checkerboard        = (self.row, self.col)
        
        self.camera_list         = camera_list
        self.input_image_file    = input_image_file
        self.cam_folder_name     = cam_folder_name
        self.output_yaml_file    = output_yaml_file
        self.output              = "/CAM-"

        self.set_pixel_distance  = 28.5
        self.allowable_pixel_err = 4
        
        self.image_width         = 1280
        self.image_height        = 720
        self.blur_threshold      = 60.0
        self.aruco_error         = 4
        self.aruco_x1_value      = 50
        self.aruco_x2_value      = 100
        self.aruco_yaw_value     = 0
        self.dist_matrix_list    = []
        self.int_matrix_list     = []

        self.cwd                 = os.getcwd()
        self.serials             = []
        self.filter              = "ID_SERIAL_SHORT="
        self.frame_count         = 0
        self.camera_count        = []
        self.camera_valid_count  = []
        self.camera_list_frame   = [[] for i in range(len(camera_list))]
        self.device_index        = None
        self.list_index          = []

        self.dictionary          = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters          = cv2.aruco.DetectorParameters_create()
        self.markerSizeInCM      = 6.7
        self.criteria            = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


    def represent_ordereddict(self, dumper, data):

        value = []
        for item_key, item_value in data.items():
            node_key = dumper.represent_data(item_key)
            node_value = dumper.represent_data(item_value)

            value.append((node_key, node_value))

        return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', value)
    
    def get_cam_serial(self, cam_id):

        # Getting serial number of each camera
        process = subprocess.Popen('udevadm info --name=/dev/video{} | grep {} | cut -d "=" -f 2'.format(cam_id, self.filter),
                             stdout=subprocess.PIPE, shell=True)
        (output, err) = process.communicate()
        process.status = process.wait()
        response = output.decode('utf-8')
        return response.replace('\n', '')
    
    def capture_image(self):

        # Checking for connected usb port
        for file in os.listdir("/sys/class/video4linux"):
            video_path = os.path.realpath("/sys/class/video4linux/" + file)
            self.device_index = video_path[-1]
            serial = self.get_cam_serial(int(self.device_index))
    
            # Checking even number of port index and serial no. char greater than 6(discarding webcam)
            if(int(self.device_index) % 2 == 0 and len(serial) > 6):
                self.list_index.append(self.device_index)
                self.serials.append(serial)
        
        # Initialize n number of cameras
        for cam_index in range(len(self.camera_list)):
            
            camera = cv2.VideoCapture(int(self.list_index[len(self.camera_list) -cam_index - 1]), cv2.CAP_V4L)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH,self.image_width);
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT,self.image_height);
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

            self.camera_count.append(camera)

        # for sampling variable
        self.frame_count = 0

        while True:
            for cam_index in range(len(self.camera_list)):
                
                ret, frame = self.camera_count[cam_index].read()          
                grayColor = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Set desired sampling count 10
                if(self.frame_count == 10):
                    self.camera_list_frame[cam_index].append(grayColor)

                    # Wait for all the connected cameras and then reset the sample count
                    if(cam_index + 1 == len(self.camera_list)):
                        self.frame_count = 0

                # Display the feeds
                cv2.imshow('Camera' + self.camera_list[cam_index] + ' feed' , grayColor)
            self.frame_count += 1

            # Check whether space button is pressed and if it is pressed, break the while loop
            if cv2.waitKey(1) & 0xFF == ord(' '):
                print("Stop recording")
                break 

         # Close the cameras
        for i in range(len(self.camera_list)):
          self.camera_count[i].release()

        print("Please put the arcuo marker stands...")

        while(True):
            if cv2.waitKey(1) & 0xFF == ord(' '):
                break

        cv2.destroyAllWindows()
        
    def preprocessing_and_calibration(self):
               
        local_cam_list_count = 0

        for camera_list in self.camera_list_frame:

            # Vector for 3D points
            obj_points = []
 
            # Vector for 2D points
            img_points = []

            objectp3d = np.zeros((1, self.checkerboard[0] * self.checkerboard[1], 3), np.float32)
            objectp3d[0, :, :2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2)

            print("Saving images for " + self.cam_folder_name + self.camera_list[local_cam_list_count])
            count = 0

            path_to_imgs = self.cwd + self.input_image_file + self.cam_folder_name + self.camera_list[local_cam_list_count]

            # Check the path exists, if not create the path
            if not os.path.exists(path_to_imgs):
                os.makedirs(path_to_imgs)

            # for sampling variable
            sampling_count = 0

            for i_frame in camera_list:

                # Store value for blurry laplacian
                fm = cv2.Laplacian(i_frame, cv2.CV_64F).var()

                # Find the pattern corner
                ret, corners = cv2.findChessboardCorners(
                                            i_frame, self.checkerboard,
                                            cv2.CALIB_CB_ADAPTIVE_THRESH
                                            + cv2.CALIB_CB_FAST_CHECK +
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)

                # Check whether checkerboard pattern is detected
                if(ret == True):
                    sampling_count += 1

                    # Set desired sampling count 2
                    if(sampling_count == 2):
                        sampling_count = 0

                        # Check the laplacian blur threshold
                        if(fm > self.blur_threshold):
                        
                            # Save the images in a specified location
                            cv2.imwrite( self.cwd + self.input_image_file + self.cam_folder_name + self.camera_list[local_cam_list_count] + "/frame%d.jpg" % count, i_frame)
                            count += 1

                            obj_points.append(objectp3d)

                            corners2 = cv2.cornerSubPix(
                                i_frame, corners, (11, 11), (-1, -1), self.criteria)

                            img_points.append(corners2)

            print("Done for " + self.cam_folder_name + self.camera_list[local_cam_list_count])

            print("Start calibration for " + self.cam_folder_name + self.camera_list[local_cam_list_count] + ' ...')

            ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
                obj_points, img_points, (self.image_width, self.image_height), None, None)

            print("Finish calibration for " + self.cam_folder_name + self.camera_list[local_cam_list_count])

            local_cam_list_count += 1

            self.dist_matrix_list.append(distortion)
            self.int_matrix_list.append(matrix)           

    def validation(self):

        # Local variables
        valid_aruco_flag = 1
        valid_pixel_cam_count = 0
        valid_pixel_flag = False
        save_img_count = 0

        for cam_index in range(len(self.camera_list)):

            camera_valid = cv2.VideoCapture(int(self.list_index[len(self.camera_list) -cam_index - 1]), cv2.CAP_V4L)
            camera_valid.set(cv2.CAP_PROP_FRAME_WIDTH,self.image_width);
            camera_valid.set(cv2.CAP_PROP_FRAME_HEIGHT,self.image_height);
            self.camera_valid_count.append(camera_valid)

        while True:

            for i_index in range(len(self.camera_list)):
                ret, test_frame = self.camera_valid_count[i_index].read()
                test_grayColor  = cv2.cvtColor(test_frame, cv2.COLOR_BGR2GRAY)  
                rectified_image = cv2.undistort(test_grayColor, self.int_matrix_list[i_index], self.dist_matrix_list[i_index], None, None)


                # Aruco marker detection
                (corners, ids, rejected) = cv2.aruco.detectMarkers(test_grayColor, self.dictionary, parameters=self.parameters)

                # Get translation and rotation of marker
                rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.markerSizeInCM, self.int_matrix_list[i_index], self.dist_matrix_list[i_index])

                # Now testing with two marker
                if(valid_aruco_flag <= len(self.camera_list)):

                    if(tvec is not None):
                        valid_aruco_x1   = tvec[0][0][2]
                        valid_aruco_x2   = tvec[1][0][2]
                        valid_aruco_yaw1 = rvec[0][0][2]
                        valid_aruco_yaw2 = rvec[1][0][2]
                        print("Valid x1 : " + str(valid_aruco_x1))
                        print("Valid x2 : " + str(valid_aruco_x2))
                        print("Valid yaw1 : " + str(valid_aruco_yaw1))
                        print("Valid yaw2 : " + str(valid_aruco_yaw2))

                        # Check whether calibration matrix is good by validating with pre-defined aruco detection x-distance value and yaw value
                        if(( ( (self.aruco_x2_value - self.aruco_error) < valid_aruco_x1 < (self.aruco_x2_value + self.aruco_error) ) or (  (self.aruco_x1_value - self.aruco_error) < valid_aruco_x1 < (self.aruco_x1_value + self.aruco_error) ) ) and ( ( (self.aruco_x2_value - self.aruco_error) < valid_aruco_x2 < (self.aruco_x2_value + self.aruco_error) ) or ( (self.aruco_x1_value - self.aruco_error) < valid_aruco_x2 < (self.aruco_x1_value + self.aruco_error) ) ) and ( (self.aruco_yaw_value - self.aruco_error) < valid_aruco_yaw1 < (self.aruco_yaw_value + self.aruco_error) ) and ( (self.aruco_yaw_value - self.aruco_error) < valid_aruco_yaw2 < (self.aruco_yaw_value + self.aruco_error) )):
                            print("Yes the matrix for " + self.cam_folder_name + self.camera_list[i_index] + " is fine")
                        else:
                            print("No the matrix for " + self.cam_folder_name + self.camera_list[i_index] + " is not fine")
                        valid_aruco_flag = 1 + valid_aruco_flag

                    else:
                        print("No aruco marker detected")
                        valid_aruco_flag = 1 + valid_aruco_flag

                cv2.imshow('Camera' + self.camera_list[i_index] + ' feed' , rectified_image)

                pressedKey = cv2.waitKey(1) & 0xFF
        
                # Check whether 'a' key is pressed, and if it is pressed, check pixel dst values for each square in checkerboard
                if pressedKey == ord('a'):
                    valid_pixel_flag = True
                    valid_pixel_cam_count = 0
                    save_img_count += 1
                    
                if(valid_pixel_flag and valid_pixel_cam_count < len(self.camera_list)):
                    
                    print("Start pixel validation for " + self.cam_folder_name + self.camera_list[i_index])
                    
                    # Local variables for resetting after every camera pixel check
                    valid_pixel_x_count = 0
                    valid_pixel_y_count = 0
                    x_pixel_list = []
                    y_pixel_list = []
                    valid_pixel_cam_count += 1

                    # path for yaml file and valid pixel images
                    save_path_valid = self.cwd + self.input_image_file + self.cam_folder_name + self.camera_list[i_index] + self.output + self.camera_list[i_index]
                    
                    # Check the path exists, if not create the path
                    if not os.path.exists(save_path_valid):
                        os.makedirs(save_path_valid)

                    image_valid_org = "/org_img_{}.jpg".format(save_img_count)
                    cv2.imwrite(save_path_valid + image_valid_org, rectified_image)
                
                    ret, corners = cv2.findChessboardCorners(
                                                    rectified_image, self.checkerboard,
                                                    cv2.CALIB_CB_ADAPTIVE_THRESH
                                                    + cv2.CALIB_CB_FAST_CHECK +
                                                    cv2.CALIB_CB_NORMALIZE_IMAGE)

                    if(ret == True):
                        corners2 = cv2.cornerSubPix(rectified_image, corners, (11,11), (-1,-1), self.criteria)

                        # Check pixel dst values for every sqaure in checkerboard
                        for n in range(self.row * self.col):

                            # for out of range error check
                            if(n < ((self.row * self.col)- self.row)):
                                x_pixel = corners2[n][0][0] - corners2[n+self.row][0][0]
                                x_pixel_list.append(abs(x_pixel))
                                
                                # Accept validation if the x pixel dst error is within allowable value
                                if(self.set_pixel_distance-self.allowable_pixel_err <= abs(x_pixel) <= self.set_pixel_distance+self.allowable_pixel_err):
                                    valid_pixel_x_count += 1
                            
                            # for out of range error check      
                            if(n < (self.row * self.col)-1 and (n+1)% self.row != 0):
                                y_pixel = corners2[n+1][0][1] - corners2[n][0][1]
                                y_pixel_list.append(abs(y_pixel))
                                
                                # Accept validation if the y pixel dst error is within allowable value
                                if(self.set_pixel_distance-self.allowable_pixel_err <= abs(y_pixel) <= self.set_pixel_distance+self.allowable_pixel_err):
                                    valid_pixel_y_count += 1

                            # Put text on img every 3 coordinates
                            if(n%3 == 0):
                                label = str( ( int(corners2[n][0][0]), int(corners2[n][0][1])  ) )
                                cv2.putText(rectified_image, label, (int(corners2[n][0][0]), int(corners2[n][0][1])), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255,0), 1);
                                    
                        print(max(x_pixel_list))
                        print(max(y_pixel_list))                
                        
                        # Accept overall validation if x and y pixel valid count has specific value
                        if(valid_pixel_x_count == (self.row*(self.col-1)) and valid_pixel_y_count == (self.col*(self.row-1))):
                            print("Success for" + self.cam_folder_name + self.camera_list[i_index])
                            print("-----------------------------------------------")

                        imgae_valid_label = "/label_img_{}.jpg".format(save_img_count)
                        cv2.imwrite(save_path_valid + imgae_valid_label, rectified_image)


            # Check whether space button is pressed and if it is pressed, break the while loop
            if pressedKey == ord(' '):
                print("Stop streaming")
                break

        # Close the cameras
        for i in range(len(self.camera_list)):
            self.camera_valid_count[i].release()
        cv2.destroyAllWindows()

        for i_index in range(len(self.camera_list)):

            save_path_valid = self.cwd + self.input_image_file + self.cam_folder_name + self.camera_list[i_index] + self.output + self.camera_list[i_index]
                    
            # Check the path exists, if not create the path
            if not os.path.exists(save_path_valid):
                os.makedirs(save_path_valid)
                
            # Set location for output yaml file
            output_file = self.cwd + self.input_image_file + self.cam_folder_name + self.camera_list[i_index] + self.output + self.camera_list[i_index] + '/' + self.output_yaml_file + '.yaml'

            yaml.add_representer(OrderedDict, self.represent_ordereddict)

            # Set data for yaml file which is suitable for usb_cam package
            data = OrderedDict(
                        camera_name = 'usb_cam',
                        image_width = self.image_width,
                        image_height = self.image_height,
                        camera_matrix = OrderedDict(
                        rows = 3,
                        cols = 3,
                        data = [float(self.int_matrix_list[i_index][0][0]), float(self.int_matrix_list[i_index][0][1]), float(self.int_matrix_list[i_index][0][2]), float(self.int_matrix_list[i_index][1][0]), float(self.int_matrix_list[i_index][1][1]), float(self.int_matrix_list[i_index][1][2]), float(self.int_matrix_list[i_index][2][0]), float(self.int_matrix_list[i_index][2][1]), float(self.int_matrix_list[i_index][2][2]) ]),
                        distortion_model = 'rational_polynomial',
                        distortion_coefficients = OrderedDict(
                        rows = 1,
                        cols = 5,
                        data = [float(self.dist_matrix_list[i_index][0][0]), float(self.dist_matrix_list[i_index][0][1]), float(self.dist_matrix_list[i_index][0][2]), float(self.dist_matrix_list[i_index][0][3]), float(self.dist_matrix_list[i_index][0][4]) ]),
                        projection_matrix = OrderedDict(
                        rows = 3,
                        cols = 4,
                        data = [float(self.int_matrix_list[i_index][0][0]), float(self.int_matrix_list[i_index][0][1]), float(self.int_matrix_list[i_index][0][2]), 0, float(self.int_matrix_list[i_index][1][0]), float(self.int_matrix_list[i_index][1][1]), float(self.int_matrix_list[i_index][1][2]), 0, float(self.int_matrix_list[i_index][2][0]), float(self.int_matrix_list[i_index][2][1]), float(self.int_matrix_list[i_index][2][2]), 0 ]),
                        rectification_matrix = OrderedDict(
                        rows = 3,
                        cols = 3,
                        data = [1, 0, 0, 0, 1, 0, 0, 0, 1]),
                        )

            # Write the data into yaml file 
            with open(output_file, 'w') as outfile:
                yaml.dump(data, outfile)

    def run(self):
        self.capture_image()
        self.preprocessing_and_calibration()
        self.validation()

def main():

    first_camera           = '-1' if len(sys.argv) < 2 else sys.argv[1] 
    second_camera          = '-1' if len(sys.argv) < 3 else sys.argv[2] 
    third_camera           = '-1' if len(sys.argv) < 4 else sys.argv[3] 
    image_dir_name         = '/calib_imgs' if len(sys.argv) < 5 else sys.argv[4]
    camera_dir_name        = '/camera' if len(sys.argv) < 6 else sys.argv[5]
    calibration_yaml_name  = 'usb_cam' if len(sys.argv) < 7 else sys.argv[6]

    camera_list = []
    if(first_camera != '-1'):
        camera_list.append(first_camera)
    if(second_camera != '-1'):
        camera_list.append(second_camera)
    if(third_camera != '-1'):
        camera_list.append(third_camera)

    cam_obj = Cam_capture(camera_list, image_dir_name, camera_dir_name, calibration_yaml_name)
    cam_obj.run()
    return 0

if __name__ == '__main__':                
    main()
