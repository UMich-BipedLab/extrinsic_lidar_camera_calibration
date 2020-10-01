import rosbag
import cv2
import sys
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

# This simply visualizes the image, by hovering the curser over the image, the respective corner pixel can be read and put int getBagData.m file. (Format: [x_top, x_left, x_right, x_bottom; y_top, y_left, y_right, y_bottom; 1, 1, 1, 1])

if __name__ == '__main__':
    if (len(sys.argv) < 3): 
      print("not enough input arguments , usage: 'python bag2opencvCorners.py <bagfile_path> <undistorted_image_topic>'")
    else:
      bagfile_name = sys.argv[1]
      image_topic = sys.argv[2] #"/alphasense_driver_ros/cam1/undistorted"
    
    bagfile = bagfile_name
    bag = rosbag.Bag(bagfile)

    bridge = CvBridge()

    for topic, image, t in bag.read_messages(topics=image_topic):
      cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
      plt.figure(1)
      #plt.imshow(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
      plt.imshow(cv_image, cmap='gray', vmin=0, vmax=255)
      plt.show()
