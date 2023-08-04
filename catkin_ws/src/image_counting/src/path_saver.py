#!/usr/bin/python3

from std_msgs.msg import Empty, String
from sensor_msgs.msg import CompressedImage
import rospy

import os 

os.chdir(os.path.dirname(os.path.realpath(__file__)))

file_path_publisher = None

counter = 0

txt_path = f'{os.getcwd()}/img_paths.txt'
txt_file_handle = None

def clear_txt_file(*args, **kwargs):
    if txt_file_handle is not None:
      txt_file_handle.truncate(0)
  
def on_img_path(msg):
    global counter
    counter += 1
    global txt_file_handle
    if txt_file_handle is not None:
      txt_file_handle.write(msg.data)
      txt_file_handle.write('\n')
      txt_file_handle.flush()
  
def main():
  global file_path_publisher 
  global txt_file_handle
  txt_file_handle = open(txt_path, 'a')
  clear_txt_file()
  rospy.init_node('image_counter', anonymous=True)
  image_path_subscriber = rospy.Subscriber('/img_path', String, on_img_path)
  reset_subscriber = rospy.Subscriber('/reset', Empty, clear_txt_file)
  rospy.spin()
  if txt_file_handle is not None:
    txt_file_handle.close()
  
if __name__ == "__main__":
    main()