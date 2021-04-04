import numpy as np
from matplotlib import pyplot as plt

if __name__ == "__main__":
    result_path = "/home/guoxt/Documents/kitti_odom/data_odometry_gray/dataset/sequences/00/result.txt"
    gt_path = "/home/guoxt/Documents/kitti_odom/data_odometry_poses/dataset/poses/00.txt"
    
    gt_pose = np.array([[0,0,0]])
    with open(gt_path, 'r') as f:
        for line in f.readlines():
            p = line.split(' ')
            if(p[0]=='skip\n'):
                continue
            pose = [float(s) for s in p if s]
            pos = np.array([pose[3], pose[7], pose[11]])
            gt_pose = np.insert(gt_pose, -1, values=pos, axis=0)
    
    plt.plot(gt_pose[:,0], 'r')
    #plt.scatter(gt_pose[:,0], gt_pose[:,2], c='r')

    results_pose = np.array([[0,0,0]])
    with open(result_path, 'r') as f:
        for line in f.readlines():
            p = line.split(' ')
            pose = [float(s) for s in p[1:] if s]
            pos = np.array([pose[3], pose[7], pose[11]])
            results_pose = np.insert(results_pose, -1, values=pos, axis=0)
    
    plt.plot(results_pose[:,0],'b')
    #plt.scatter(results_pose[:,0], results_pose[:,2], c='b')
    plt.title("x value for FAST")
    plt.show()