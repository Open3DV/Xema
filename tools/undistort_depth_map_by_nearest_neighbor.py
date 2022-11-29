import numpy as np
import cv2
import sys

def depthmap_status(img):
    mask = (img > 0.001)
    valid_img = img[mask]
    valid_img = valid_img.reshape(-1)
    depth_min = np.percentile(valid_img, 2)
    depth_max = np.percentile(valid_img, 98)
    return depth_max, depth_min, mask

def Loading_Depth_From_Tiff(depth_file):
    depth = cv2.imread(depth_file, -1)
    depth = np.float32(np.array(depth))

    return depth


def Loading_Params_From_Txt(params_file):
    params = np.loadtxt(params_file)
    camera_mtx = np.zeros((3, 3))
    camera_mtx[0, 0] = params[0]
    camera_mtx[0, 2] = params[2]
    camera_mtx[1, 1] = params[4]
    camera_mtx[1, 2] = params[5]
    camera_mtx[2, 2] = 1
    camera_dist = params[9:14]

    return camera_mtx, camera_dist

def depth_undistortion_by_nearest_neighbor(depth, camera_mtx, camera_dist):
    rotation = np.array([1,0,0,0,1,0,0,0,1]).reshape(3, 3)
    map_x = []
    map_y = []
    pointcloud = []
    map_x, map_y = cv2.initUndistortRectifyMap(camera_mtx, camera_dist, rotation, camera_mtx, (depth.shape[1], depth.shape[0]), cv2.CV_32F)
    # 相同相机的map可计算保存一次，往后重复使用
    # cv2.imwrite('./mapx.tiff', map_x)
    # cv2.imwrite('./mapy.tiff', map_y)
    # map_x = cv2.imread('./mapx.tiff', -1)
    # map_y = cv2.imread('./mapy.tiff', -1)
    undistorted_depth = np.zeros(depth.shape, dtype=np.float32)
    for iy in range(depth.shape[0]):
        for ix in range(depth.shape[1]):
            # 先对图片去畸变
            ixiy = np.zeros((1, 1, 2), np.float32)
            #变换相机的像素到归一化的相机内参，计算点云信息
            ixiy[0, 0, 0] = (float(ix) - camera_mtx[0, 2]) / camera_mtx[0, 0]
            ixiy[0, 0, 1] = (float(iy) - camera_mtx[1, 2]) / camera_mtx[1, 1]
            z = depth[int(map_y[iy, ix] + 0.5), int(map_x[iy, ix] + 0.5)]
            if z > 0:
                x = ixiy[0, 0, 0] * z
                y = ixiy[0, 0, 1] * z
                pts = np.array([x, y, z])
                pointcloud.append(pts)
                undistorted_depth[iy, ix] = z
    return undistorted_depth, pointcloud

def save_undistortion_depth(depth_tiff_path, param_txt_path):
    camera_mtx, camera_dist = Loading_Params_From_Txt(param_txt_path)
    depth = Loading_Depth_From_Tiff(depth_tiff_path)
    depth_result, pointcloud = depth_undistortion_by_nearest_neighbor(depth, camera_mtx, camera_dist)

    output_depth_path = depth_tiff_path[:-5]+'_undistorted.tiff'
    output_xyz_path = depth_tiff_path[:-4]+'xyz'
    print(output_depth_path)
    cv2.imwrite(output_depth_path, depth_result)

    np.savetxt(output_xyz_path, pointcloud)

if __name__ == '__main__':
    argv_lenth = len(sys.argv)
    if argv_lenth == 1:
        print("使用示例：python undistort_depth_map_by_nearest_neighbor.py ./depth.tiff param.txt")
        exit(0)

    input_tiff_path = sys.argv[1]
    input_param_path = sys.argv[2]

    print(input_tiff_path)

    save_undistortion_depth(input_tiff_path, input_param_path)
