import os
import numpy as np
import cv2

pt1 = np.array([40, 10, -0.93, 1])
pt2 = np.array([40, -10, -0.93, 1])
pt3 = np.array([20, 10, -0.93, 1])
pt4 = np.array([20, -10, -0.93, 1])

#pts1 = np.float32([[190.27, 405.71], [185.34, 769.29], [-643.06, 7210.80], [-519.30, -5453.55]])
#pts2 = np.float32([[0, 800], [400, 800], [0, 400], [400, 400]])

pts1 = np.float32([[405.71, 190.27], [769.44, 185.34], [198.07, 215.70], [947.07, 205.45]])
pts2 = np.float32([[0, 800], [400, 800], [0, 400], [400, 400]])

if __name__ == '__main__':
    cali_dir = '/home/sunho/catkin_ws/src/hmgchallenge/3d_object_detection/kitti_data/training/calib/'
    img_dir = '/home/sunho/catkin_ws/src/hmgchallenge/3d_object_detection/kitti_data/training/image_2/'
    filename = os.listdir(cali_dir)
    filename = sorted(filename)
    file_number = len(filename)
    
    for i in range(file_number):
        cali_path = os.path.join(cali_dir, filename[i])
        img_path = os.path.join(img_dir, filename[i][:-3] + "png")

        with open(cali_path, 'r') as g: 
            img = cv2.imread(img_path)

            lines = g.readlines()

            data = lines[-2].strip('\n').split(' ')[1:]
            imu_to_velo = [float(k) for k in data]
            imu_to_velo = np.array(data, dtype = np.float).reshape(3,4)
            imu_to_velo = np.append(imu_to_velo, [[0, 0, 0, 1]], 0)

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

            pt1_projected = np.matmul(P2, np.matmul(R0_rect, np.matmul(np.matmul(imu_to_velo, velo_to_cam), pt1)))
            pt2_projected = np.matmul(P2, np.matmul(R0_rect, np.matmul(np.matmul(imu_to_velo, velo_to_cam), pt2)))
            pt3_projected = np.matmul(P2, np.matmul(R0_rect, np.matmul(np.matmul(imu_to_velo, velo_to_cam), pt3)))
            pt4_projected = np.matmul(P2, np.matmul(R0_rect, np.matmul(np.matmul(imu_to_velo, velo_to_cam), pt4)))

            
            print("%.2f %.2f"%(pt1_projected[0]/pt1_projected[2], pt1_projected[1]/pt1_projected[2]))
            print("%.2f %.2f"%(pt2_projected[0]/pt2_projected[2], pt2_projected[1]/pt2_projected[2]))
            print("%.2f %.2f"%(pt3_projected[0]/pt3_projected[2], pt3_projected[1]/pt3_projected[2]))
            print("%.2f %.2f"%(pt4_projected[0]/pt4_projected[2], pt4_projected[1]/pt4_projected[2]))

            M = cv2.getPerspectiveTransform(pts1, pts2)
            print(M)
            img_BEV = cv2.warpPerspective(img, M, (400,800))
            img_BEV = cv2.flip(img_BEV, 0)

            #cv2.imshow("img", img)
            #cv2.imshow("img_BEV", img_BEV)
            #cv2.waitKey(0)
        break        
        
        