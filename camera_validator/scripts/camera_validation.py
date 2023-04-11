import cv2
import time
import numpy as np
import math
import glob
import yaml
import sys
import os
import subprocess

class Cam_capture(object):
    def __init__(self, camera_list, input_image_file, cam_folder_name, output_yaml_file):
        
        self.camera_list        = camera_list
        self.input_image_file   = input_image_file
        self.cam_folder_name    = cam_folder_name
        self.output_yaml_file   = output_yaml_file
        self.output             = "/CAM-"

        self.image_width        = 1280
        self.image_height       = 720

        self.camera_count       = []
        self.cwd                = os.getcwd()
        self.serials            = []
        self.filter             = "ID_SERIAL_SHORT="
        self.dist_matrix_list   = [[] for i in range(len(camera_list))]
        self.int_matrix_list    = [[] for i in range(len(camera_list))]
        
        self.count_frame        = 0
        self.device_index       = None
        self.list_index         = []
        self.dictionary         = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters         = cv2.aruco.DetectorParameters_create()
        self.markerSizeInCM     = 6.7
     
    def get_cam_serial(self, cam_id):

        # Getting serial number of each camera
        p = subprocess.Popen('udevadm info --name=/dev/video{} | grep {} | cut -d "=" -f 2'.format(cam_id, self.filter),
                             stdout=subprocess.PIPE, shell=True)
        (output, err) = p.communicate()
        p.status = p.wait()
        response = output.decode('utf-8')
        return response.replace('\n', '')

    def validation(self):
        
        for file in os.listdir("/sys/class/video4linux"):
            real_file = os.path.realpath("/sys/class/video4linux/" + file)
            self.device_index = real_file[-1]
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
            self.camera_count.append(camera)
            
            input_yaml_file = self.cwd + self.input_image_file + self.cam_folder_name + self.camera_list[cam_index] + self.output + self.camera_list[cam_index] + '/' + self.output_yaml_file + ".yaml"
            yaml_file = open(input_yaml_file)
            parsed_yaml_file = yaml.load(yaml_file)
            
            dist_usb_cam =   [[parsed_yaml_file["distortion_coefficients"]["data"][0],  parsed_yaml_file["distortion_coefficients"]["data"][1], parsed_yaml_file["distortion_coefficients"]["data"][2], parsed_yaml_file["distortion_coefficients"]["data"][3], parsed_yaml_file["distortion_coefficients"]["data"][4]]]
            mtx_usb_cam  =    [[parsed_yaml_file["camera_matrix"]["data"][0], 0.0, parsed_yaml_file["camera_matrix"]["data"][2]],
                    [0.0, parsed_yaml_file["camera_matrix"]["data"][4], parsed_yaml_file["camera_matrix"]["data"][5]],
                    [0.0,               0.0,               1.0]]

            dist_usb_cam = np.array(dist_usb_cam)
            mtx_usb_cam = np.array(mtx_usb_cam)
            
            self.dist_matrix_list[cam_index].append(dist_usb_cam)
            self.int_matrix_list[cam_index].append(mtx_usb_cam)

        while True:

            for i_index in range(len(self.camera_list)):
                ret, frame = self.camera_count[i_index].read()          
                grayColor = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                dst = cv2.undistort(grayColor, self.int_matrix_list[i_index][0], self.dist_matrix_list[i_index][0], None, None)

                cv2.imshow('Camera' + self.camera_list[i_index] + ' feed' , dst)

                # Aruco marker detection
                (corners, ids, rejected) = cv2.aruco.detectMarkers(grayColor, self.dictionary, parameters=self.parameters)

                # Get translation and rotation of marker
                rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.markerSizeInCM, self.int_matrix_list[i_index][0], self.dist_matrix_list[i_index][0])

                if(tvec is not None):
                    rvec = rvec * (180.0/math.pi)
                    print("----------------------------------------------------------------------------------------")
                    print(" Camera" + self.camera_list[i_index] + " value: "+ str(tvec[0][0][2]))
                else:
                    print("No aruco marker detected")

            if cv2.waitKey(1) & 0xFF == ord(' '):
                print("Stop the cameras")
                break

        # Close the cameras
        for i in range(len(self.camera_list)):
            self.camera_count[i].release()
        cv2.destroyAllWindows()

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
    cam_obj.validation()
    return 0

if __name__ == '__main__':                
    main()