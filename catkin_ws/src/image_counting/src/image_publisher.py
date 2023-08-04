#!/usr/bin/python3

import rospy
from sensor_msgs.msg import CompressedImage
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import os

os.chdir(os.path.dirname(os.path.realpath(__file__)))

publisher = None

pipeline = None

mainloop = None

def publish_compressed_image(data):
    if publisher is not None:
        publisher.publish(CompressedImage(format="jpeg", data=data))

def on_new_sample(sink):
    sample = sink.emit('pull-sample')
    if not isinstance(sample, Gst.Sample):
        print("pull-sampled got us sth other than a Gst.Sample:", sample)
        return Gst.FlowReturn.ERROR

    # print('Got buffer', sample.get_buffer())
    # print('Got caps', sample.get_caps())

    buffer = sample.get_buffer()
    if buffer is not None:
        success, map_info = buffer.map(Gst.MapFlags.READ)

        if not success:
            raise Exception('Could not read image')

        publish_compressed_image(map_info.data)

        buffer.unmap(map_info)

    return Gst.FlowReturn.OK

def make_gst_pipeline():
    Gst.init(None)
    try:
        device = rospy.get_param("device")
    except KeyError:
        device = "/dev/video0"
        
    if not os.path.exists(device):
        raise Exception("video device", device, "does not exist")
    pipeline = Gst.parse_launch(f"v4l2src device={device} ! decodebin ! videoconvert ! jpegenc ! appsink emit-signals=True name=appsink")
    appsink = pipeline.get_by_name('appsink')
    appsink.connect("new-sample", on_new_sample)
    res = pipeline.set_state(Gst.State.PLAYING)
    if res == Gst.StateChangeReturn.FAILURE:
        raise Exception("Failed to set pipeline to playing state")
    
    print("Initiated pipeline and set it to play.")
    return pipeline

def stop_pipeline():
    global pipeline
    if pipeline is None:
        return
    print("Stoping pipeline..", pipeline)
    res = pipeline.set_state(Gst.State.NULL)
    if res == Gst.StateChangeReturn.FAILURE:
        print("Could not stop pipeline")
    print("Stopped pipeline")

def stop_and_quit():
    rospy.signal_shutdown("Quitting")
    stop_pipeline()
    global mainloop
    print('Stopping mainloop', mainloop)
    exit(0)
    
def handle_bus_message(bus):
    message = bus.timed_pop_filtered(1 * Gst.MSECOND, Gst.MessageType.ANY)
    if message:
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print("There was an error with the element", message.src.get_name(), ":", err)
            stop_and_quit()
        elif message.type == Gst.MessageType.EOS:
            print("Camera stream has ended.")
            stop_and_quit()
        elif message.type == Gst.MessageType.STATE_CHANGED:
            if isinstance(message.src, Gst.Pipeline):
                old_state, new_state, _ = message.parse_state_changed()
                print("Stream has changed from", old_state.value_nick, "to", new_state.value_nick)

    GLib.idle_add(handle_bus_message, bus)
    
def setup_ros():
    global publisher
    rospy.init_node('image_publisher', anonymous=True, disable_signals=True)
    
    publisher = rospy.Publisher('/cam_img', CompressedImage, queue_size=10)
    
def on_signal_stop(signum, frame):
    stop_and_quit()
    
def main():
    global pipeline
    global mainloop
    pipeline = make_gst_pipeline()
    bus = pipeline.get_bus()

    mainloop = GLib.MainLoop()
    
    GLib.idle_add(handle_bus_message, bus)
    
    setup_ros()
    
    try:
        mainloop.run()
    except KeyboardInterrupt:
        stop_and_quit()

if __name__ == "__main__":
    main()
