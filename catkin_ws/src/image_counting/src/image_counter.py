#!/usr/bin/python3

from std_msgs.msg import Empty, String
from sensor_msgs.msg import CompressedImage
import rospy

import os 

file_path_publisher = None

def get_cwd():
  return os.path.dirname(os.path.realpath(__file__))

counter = 0

def reset(*args, **kwargs):
    global counter
    counter = 0
  
def is_prime(num):
    if num > 1:
        for i in range(2, int(num/2)+1):
            if (num % i) == 0:
              return False
        else:
            return True
    else:
        return False

def on_cam_img_msg(msg):
    global counter
    counter += 1
    if is_prime(counter):
      print(counter, 'is prime')
      path = f'{os.getcwd()}/{counter}.jpg'
      with open(path, 'wb') as f:
        f.write(msg.data)
        f.flush()
      
      if file_path_publisher is not None:
        file_path_publisher.publish(path)
      

def main():
  global file_path_publisher 
  rospy.init_node('image_counter', anonymous=True)
  file_path_publisher = rospy.Publisher('/img_path', String, queue_size=10)
  image_subscriber = rospy.Subscriber('/cam_img', CompressedImage, on_cam_img_msg)
  reset_subscriber = rospy.Subscriber('/reset', Empty, reset)
  rospy.spin()
  
if __name__ == "__main__":
    main()