import os
import numpy as np
import struct
import open3d
import cv2
import rospy

cm_per_pixel = 5.0
map_scale = 100.0 / cm_per_pixel
map_height = 800
map_width = 400

homography_mat = np.array([[-1.80727165e-01, -1.20425697e+00, 3.24824958e+02], [3.95458817e-05, -1.32397802e-01, -1.14451140e+02], [7.03755830e-05, -5.85066327e-03, 1.00000000e+00]])

def draw_rectangle(image, centre, theta, width, height):
    theta = - theta
    c, s = np.cos(theta), np.sin(theta)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    
    p1 = [ + width / 2,  + height / 2]
    p2 = [- width / 2,  + height / 2]
    p3 = [ - width / 2, - height / 2]
    p4 = [ + width / 2,  - height / 2]
    p1_new = np.dot(p1, R)+ centre
    p2_new = np.dot(p2, R)+ centre
    p3_new = np.dot(p3, R)+ centre
    p4_new = np.dot(p4, R)+ centre
    
    img = cv2.line(image, (int(p1_new[0, 0]), int(p1_new[0, 1])), (int(p2_new[0, 0]), int(p2_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p2_new[0, 0]), int(p2_new[0, 1])), (int(p3_new[0, 0]), int(p3_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p3_new[0, 0]), int(p3_new[0, 1])), (int(p4_new[0, 0]), int(p4_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p4_new[0, 0]), int(p4_new[0, 1])), (int(p1_new[0, 0]), int(p1_new[0, 1])), (255, 0, 0), 1)
    img = cv2.arrowedLine(img , (int((p2_new[0, 0] + p3_new[0, 0])/2), int((p2_new[0, 1] + p3_new[0, 1])/2)), (int((p1_new[0, 0] + p4_new[0, 0])/2), int((p1_new[0, 1] + p4_new[0, 1])/2)), (255, 0, 0), 1)
    
    return img

def make_BEV_map(vel_path, cali_path, img_path):
    pc_list = []
    vel_BEV_map = np.zeros((800,400,1))
    with open(vel_path, 'rb') as f:
        with open(cali_path, 'r') as g:
            
            lines = g.readlines()
            data = lines[-2].strip('\n').split(' ')[1:]
            imu_to_velo = [float(k) for k in data]
            imu_to_velo = np.array(data, dtype = np.float).reshape(3,4)
            imu_to_velo = np.append(imu_to_velo, [[0, 0, 0, 1]], 0)
            velo_to_imu = np.linalg.inv(imu_to_velo)

            data = lines[-3].strip('\n').split(' ')[1:]
            velo_to_cam = [float(k) for k in data]
            velo_to_cam = np.array(data, dtype = np.float).reshape(3,4)
            velo_to_cam = np.append(velo_to_cam, [[0, 0, 0, 1]], 0)

            data = lines[-4].strip('\n').split(' ')[1:]
            R0_rect = [float(k) for k in data]
            R0_rect = np.array(data, dtype = np.float).reshape(3,3)
            R0_rect = np.append(R0_rect, [[0, 0, 0]], 0)
            R0_rect = np.append(R0_rect, [[0],[0],[0],[1]], 1)

            data = lines[-6].strip('\n').split(' ')[1:]
            P2 = [float(k) for k in data]
            P2 = np.array(data, dtype = np.float).reshape(3,4)

            print(R0_rect)
            M = np.zeros((3,3), dtype = np.float)
            M[:2,:3] = R0_rect[:2,:3]
            M[2,2] = 1
            
            print(M)

            content = f.read()
            pc_iter = struct.iter_unpack('ffff', content)
            for idx, point in enumerate(pc_iter):
                BEV_point = np.matmul(velo_to_imu, np.transpose([point[0], point[1], point[2], 1.]))
                BEV_point_x = BEV_point[0] 
                BEV_point_y = BEV_point[1] 
                if (BEV_point_x*map_scale >= 0) and (BEV_point_x*map_scale < map_height):
                    if (BEV_point_y*map_scale >= -map_width/2) and (BEV_point_y*map_scale < map_width/2):
                        vel_BEV_map[(map_height-1) - int(BEV_point_x*map_scale),(map_width-1) - int(BEV_point_y*map_scale + map_width/2), 0] = 1

            img = cv2.imread(img_path)
            img_BEV_map = cv2.warpPerspective(img, M, (400, 800))
            img_BEV_map = cv2.warpPerspective(img, homography_mat, (400,800))
            img_BEV_map = cv2.flip(img_BEV_map, 0)

    return vel_BEV_map, img_BEV_map, np.linalg.inv(np.matmul(velo_to_cam, imu_to_velo))

def main():
    vel_dir = '/home/sunho/catkin_ws/src/hmgchallenge/3d_object_detection/kitti_data/training/velodyne/'
    cali_dir = '/home/sunho/catkin_ws/src/hmgchallenge/3d_object_detection/kitti_data/training/calib/'
    img_dir = '/home/sunho/catkin_ws/src/hmgchallenge/3d_object_detection/kitti_data/training/image_2/'
    label_dir = '/home/sunho/catkin_ws/src/hmgchallenge/3d_object_detection/kitti_data/training/label_2'

    filename = os.listdir(vel_dir)
    filename = sorted(filename)
    file_number = len(filename)

    #pcd = open3d.open3d.geometry.PointCloud()

    for i in range(file_number):
    
        vel_path = os.path.join(vel_dir, filename[i])
        cali_path = os.path.join(cali_dir, filename[i][:-3] + "txt")
        img_path = os.path.join(img_dir, filename[i][:-3] + "png")
        label_path = os.path.join(label_dir, filename[i][:-3] + "txt")
        print(vel_path)

        vel_BEV_map, img_BEV_map, cam_to_imu = make_BEV_map(vel_path, cali_path, img_path)
        
        with open(label_path, 'r') as f:
            lines = f.readlines()
            for idx in range(len(lines)):
                data = lines[idx].strip('\n').split(' ')
                
                dimensions = [float(k) for k in data[8:11]]
                dimensions = np.array(dimensions, dtype = np.float)
                
                location = [float(k) for k in data[11:14]]
                location = np.array([location[0], location[1], location[2] , 1], dtype = np.float)
                location = np.matmul(cam_to_imu, np.transpose(location))

                rotation = float(data[14])
                rotation = np.array(rotation, dtype = np.float)

                if data[0] == 'Car' or data[0] == 'Van' or data[0] == 'Truck' or data[0] == 'Misc':
                    #image, centre, theta, width, height
                    draw_rectangle(vel_BEV_map, np.array([(map_width-1) - location[1] * map_scale - map_width/2, (map_height-1) - location[0] * map_scale]), rotation, dimensions[2]*map_scale, dimensions[0]*map_scale)
                    draw_rectangle(img_BEV_map, np.array([(map_width-1) - location[1] * map_scale - map_width/2, (map_height-1) - location[0] * map_scale]), rotation, dimensions[2]*map_scale, dimensions[0]*map_scale)
                elif data[0] == 'Pedestrian':
                    draw_rectangle(vel_BEV_map, np.array([(map_width-1) - location[1] * map_scale - map_width/2, (map_height-1) - location[0] * map_scale]), rotation, dimensions[2]*map_scale, dimensions[0]*map_scale)
                    draw_rectangle(img_BEV_map, np.array([(map_width-1) - location[1] * map_scale - map_width/2, (map_height-1) - location[0] * map_scale]), rotation, dimensions[2]*map_scale, dimensions[0]*map_scale)


        cv2.imshow("img_BEV_map", img_BEV_map)
        cv2.imshow("vel_BEV_map", vel_BEV_map)
        cv2.waitKey(0)
        
        #pcd.points = open3d.open3d.utility.Vector3dVector(example)
        #open3d.open3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()