#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import bisect

class RPECalculator:
    def __init__(self, gt_file):
        rospy.init_node('rpe_calculator', anonymous=True)
        self.gt_poses = self.load_ground_truth(gt_file)
        self.pose_history = []
        self.Q_0 = []
        self.Q_1 = []
        self.P_0 = []
        self.P_1 = []
        #self.delta_t = 0.33333  # Time interval for RPE computation
        self.delta_idx = 5  # Time steps interval for RPE computation
        self.idx = 0
        self.output_file = '/home/ros-noetic/src/ORB_SLAM3/results/rpe_results.txt'
        rospy.Subscriber('/Stereo_Inertial/pose', PoseStamped, self.pose_callback)

    def load_ground_truth(self, gt_file):
        gt_data = []
        with open(gt_file, 'r') as f:
            for line in f:
                values = list(map(float, line.strip().split()))
                #timestamp, x, y, z = values[:4]
                timestamp, x, y, z, qx, qy, qz, qw = values
                #gt_data.append((timestamp, np.array([x, y, z])))
                gt_data.append((timestamp, np.array([x, y, z, qx, qy, qz, qw])))
        return gt_data

    def find_closest_gt(self, timestamp):
        gt_timestamps = [entry[0] for entry in self.gt_poses]
        idx = bisect.bisect_left(gt_timestamps, timestamp)
        if idx == 0:
            return self.gt_poses[0]
        if idx == len(gt_timestamps):
            return self.gt_poses[-1]
        before = self.gt_poses[idx - 1]
        after = self.gt_poses[idx]
        return before if abs(before[0] - timestamp) < abs(after[0] - timestamp) else after

    def compute_rpe(self):
        
        return None
        

    def pose_callback(self, msg):
    
        #if self.idx == 0:
            #return
        
        gt_time = 0
        E = []
        e = 0
        
        if self.idx % self.delta_idx == 0:
    
            timestamp = msg.header.stamp.to_sec()
            aligned_gt = self.find_closest_gt(timestamp)
            gt_time, gt_pose = aligned_gt
            
            #print(gt_pose)
            
            P_tx, P_ty, P_tz = msg.pose.position.x + (-44.3478), msg.pose.position.y + (-45.0467), msg.pose.position.z + (-1.7019)
            P_qx, P_qy, P_qz, P_qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
            
            P_R = np.array([[1 - 2*P_qy**2 - 2*P_qz**2, 2*P_qx*P_qy - 2*P_qz*P_qw, 2*P_qx*P_qz + 2*P_qy*P_qw],
                [2*P_qx*P_qy + 2*P_qz*P_qw, 1 - 2*P_qx**2 - 2*P_qz**2, 2*P_qy*P_qz - 2*P_qx*P_qw],
                [2*P_qx*P_qz - 2*P_qy*P_qw, 2*P_qy*P_qz + 2*P_qx*P_qw, 1 - 2*P_qx**2 - 2*P_qy**2]
            ])
            
            self.P_1 = np.eye(4)
            self.P_1[:3, :3] = P_R
            self.P_1[:3, 3] = [P_tx, P_ty, P_tz]
            
            Q_tx, Q_ty, Q_tz = gt_pose[0], gt_pose[1], gt_pose[2]
            Q_qx, Q_qy, Q_qz, Q_qw = gt_pose[3], gt_pose[4], gt_pose[5], gt_pose[6]
            
            Q_R = np.array([[1 - 2*Q_qy**2 - 2*Q_qz**2, 2*Q_qx*Q_qy - 2*Q_qz*Q_qw, 2*Q_qx*Q_qz + 2*Q_qy*Q_qw],
                [2*Q_qx*Q_qy + 2*Q_qz*Q_qw, 1 - 2*Q_qx**2 - 2*Q_qz**2, 2*Q_qy*Q_qz - 2*Q_qx*Q_qw],
                [2*Q_qx*Q_qz - 2*Q_qy*Q_qw, 2*Q_qy*Q_qz + 2*Q_qx*Q_qw, 1 - 2*Q_qx**2 - 2*Q_qy**2]
            ])
            
            self.Q_1 = np.eye(4)
            self.Q_1[:3, :3] = Q_R
            self.Q_1[:3, 3] = [Q_tx, Q_ty, Q_tz]
            
            
            if self.idx != 0:
                #print(self.idx, self.Q_0)
                E = np.linalg.inv(np.linalg.inv(self.Q_0) @ self.Q_1) @ (np.linalg.inv(self.P_0) @ self.P_1)
                rpe = np.linalg.norm(E[:3, 3])
                #with open(self.output_file, 'a') as f:
                    #f.write(f"{gt_time:.6f} {rpe:.6f}\n")
                #rospy.loginfo(f"Saved RPE at {gt_time:.6f}: {rpe:.6f}")
                
                rospy.loginfo(f"{Q_tx:.6f}, {P_tx:.6f}| {Q_ty:.6f}, {P_ty:.6f}| {Q_tz:.6f}, {P_tz:.6f}| {rpe:.6f}")
                
            self.P_0 = self.P_1
            self.Q_0 = self.Q_1
            
            #print(self.idx, Q_0)
        
        self.idx = self.idx+1
        #self.Q.append(Q)
        #self.P.append(P)
        
        
        
        
        #Q_inv = np.linalg.inv(Q)
        #Q_delta = np.linalg.inv(Q)
        #P_inv = np.linalg.inv(P)
        
        #received_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        #self.pose_history.append((timestamp, received_pose))
        #self.pose_history = [entry for entry in self.pose_history if timestamp - entry[0] <= self.delta_t]

        #rpe_result = self.compute_rpe()
        #if rpe_result:
        #    t, rpe = rpe_result
        #    with open(self.output_file, 'a') as f:
        #        f.write(f"{t:.6f} {rpe:.6f}\n")
        #    rospy.loginfo(f"Saved RPE at {t:.6f}: {rpe:.6f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        gt_file = '/home/ros-noetic/datasets/MADMAX/00A6/A-6_ground_truth/groundtruth.txt'
        node = RPECalculator(gt_file)
        node.run()
    except rospy.ROSInterruptException:
        pass
