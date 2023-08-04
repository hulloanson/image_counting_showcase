#!/usr/bin/python3

from std_msgs.msg import Empty, String
from sensor_msgs.msg import CompressedImage
import rospy

import os 

os.chdir(os.path.dirname(os.path.realpath(__file__)))

def is_prime(num):
    if num > 1:
        for i in range(2, int(num/2)+1):
            if (num % i) == 0:
              return False
        else:
            return True
    else:
        return False
class ImageCounter:
  
  def __init__(self):
    self.file_path_publisher = None
    self.counter = 0

  def reset(self, *args, **kwargs):
      self.counter = 0
    

  def on_cam_img_msg(self, msg):
      self.counter += 1
      if is_prime(self.counter):
        path = f'{os.getcwd()}/{self.counter}.jpg'
        with open(path, 'wb') as f:
          f.write(msg.data)
          f.flush()
        
        if self.file_path_publisher is not None:
          self.file_path_publisher.publish(path)
        
  def run(self):
    rospy.init_node('image_counter', anonymous=True)
    self.file_path_publisher = rospy.Publisher('/img_path', String, queue_size=10)
    self.image_subscriber = rospy.Subscriber('/cam_img', CompressedImage, self.on_cam_img_msg)
    self.reset_subscriber = rospy.Subscriber('/reset', Empty, self.reset)
    rospy.spin()

def main():
  counter = ImageCounter()
  counter.run()
    
if __name__ == "__main__":
    main()