#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
from math import abs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from orb_slam3.msg import KeyPointArray  # Make sure your custom messages are built
from datetime import datetime
from shapely.geometry import MultiPoint, Point, Polygon, LineString
from shapely.ops import unary_union, polygonize
from scipy.spatial import Delaunay

bridge = CvBridge()

t_img = None
image = None
latest_keypoints = None

output_dir = "/tmp/concave_hulls"
os.makedirs(output_dir, exist_ok=True)



# --------------------------
# Concave Hull via Alpha Shape
# --------------------------
def alpha_shape(points, alpha=1.0):
    if len(points) < 4:
        return MultiPoint(points).convex_hull

    def add_edge(edges, i, j):
        if (i, j) in edges or (j, i) in edges:
            return
        edges.add((i, j))

    coords = np.array(points)
    tri = Delaunay(coords)
    edges = set()
    for ia, ib, ic in tri.simplices:
        pa, pb, pc = coords[ia], coords[ib], coords[ic]
        a = np.linalg.norm(pa - pb)
        b = np.linalg.norm(pb - pc)
        c = np.linalg.norm(pc - pa)
        s = (a + b + c) / 2.0
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        circum_r = a * b * c / (4.0 * area)
        if circum_r < 1.0 / alpha:
            add_edge(edges, ia, ib)
            add_edge(edges, ib, ic)
            add_edge(edges, ic, ia)

    edge_lines = [LineString([coords[i], coords[j]]) for i, j in edges]
    triangles = polygonize(edge_lines)
    return unary_union(list(triangles))

def keypoints_callback(msg):
    global latest_keypoints

    keypoints = [(kp.x, kp.y) for kp in msg.keypoints]
    latest_keypoints = np.array(keypoints, dtype=np.float32)

    


    #if image is not None and t_img is not None:
        #process_and_save(latest_keypoints, image, t_img)


def glare_callback(msg):
    global image, t_img

    image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    t_img = msg.header.stamp.to_sec()
    
    H = 480
    W = 640
    
    if abs(t_img - 1543764680.019643545) < 0.03 or abs(t_img - 1543764680.301051855) < 0.03 or abs(t_img - 1543764680.579606295) < 0.03 or abs(t_img - 1543764680.859512091) < 0.03):
        hull = alpha_shape(latest_keypoints, alpha=0.02)
        
        mask = np.zeros((H, W), dtype=np.uint8)
        for y in range(H):
            for x in range(W):
                if hull.contains(Point(x, y)):
                    mask[y, x] = 1

        hull_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        if isinstance(hull, Polygon):
            hull_coords = np.array(hull.exterior.coords, dtype=np.int32)
            cv2.polylines(hull_img, [hull_coords], isClosed=True, color=(0, 255, 0), thickness=2)
        else:
            for poly in hull.geoms:  # Access individual polygons using .geoms
                hull_coords = np.array(poly.exterior.coords, dtype=np.int32)
                cv2.polylines(hull_img, [hull_coords], isClosed=True, color=(0, 255, 0), thickness=2)

    # Save image
    filename = f"hull_{t_img:.6f}.png"
    cv2.imwrite(filename, hull_img)
    rospy.loginfo(f"Saved concave hull image: {filename}")

'''
def process_and_save(keypoints, image, timestamp):
    if keypoints.shape[0] < 3:
        rospy.logwarn("Not enough keypoints for concave hull.")
        return

    hull = concave_hull(keypoints.tolist(), k=5)
    if hull is None or len(hull) < 3:
        rospy.logwarn("Failed to compute concave hull.")
        return

    # Prepare image
    color_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for pt in keypoints:
        cv2.circle(color_img, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)

    hull = hull.reshape((-1, 1, 2))
    cv2.polylines(color_img, [hull], isClosed=True, color=(0, 0, 255), thickness=2)

    # Save image
    filename = f"{output_dir}/hull_{timestamp:.6f}.png"
    cv2.imwrite(filename, color_img)
    rospy.loginfo(f"Saved concave hull image: {filename}")
'''


def main():
    rospy.init_node('keypoint_hull_saver', anonymous=True)
    rospy.Subscriber("/Stereo_Inertial/keypoints_current_frame", KeyPointArray, keypoints_callback, queue_size=1)
    rospy.Subscriber("/camera/left/image_raw", Image, glare_callback, queue_size=1)

    rospy.loginfo("KeyPoint Hull Saver Node Started")
    rospy.spin()


if __name__ == '__main__':
    main()

