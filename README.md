# kinectv2-setup
Documents setting up kinect v2 on jetson products


# Overview
Depth images also known as RGB-D images can be used to enhance the information of a scene. Typical pixel values relay color space information and this is used to derive features. Depth image pixel values tell you the distance that pixel is from the camera.This information can be synchronized with an RGB image for additional features or used on its own to estimate 3D spaces among other applications. There are typically two ways do calculate depth; using TOF sensor or using 2 calibrated IR cameras with triangulation.

Depth image processing hardware has made improvements in quality and price. Currently, the best consumer grade depth processors are manufactured by Microsoft and were originally used to supplement the xbox gaming experience. Developers realized these products were a cheaper way into the depth processing hardware and built opensource api's into the Xbox Kinect V1, and V2 depth camera modules. These API's are in C and there are also python bindings as well.

The Xbox Kinect modules were discontinued and Microsoft has revived the hardware in the commerical space with the Azure Kinect. The older Kinect V1 and V2 modules are still powerful and can be bought second had around $40 as of this writing.The Azure Kinect was release in late June 2019 and retails $399.99 and has much better resolution. These devices have RGB cameras, TOF/IR sensors for depth processing, and microphones.The V1 and V2 modules can stream 1080p RGB image, 424x512 Depth image, and 424X512 IR images simultaneously at 30FPS over USB3. The Azure Kinect has improved performance and throughput but at a much higher cost. Azure Kinect currently only offers C api for developers with python in the works.

Several other manufactures offer depth processing devices, but are not at the same level as Microsoft, but cost less.

Intel RealSense
StereoLabs Zed
Orbbec astra
Other hardware providers also have the sensors for integrated applications


# Setup
Steps to set up the Kinect V2 were taken from https://github.com/valdivj/jetson-nano-yolov2-darkflow project where object detection and depth applications were blended.

What you need :

Microsoft Kinect V2 ~ $40 at local gaming store
Microsoft Kinect V2 powered USB pack ~$25 amazon
Nvidia Nano
Monitor
Keyboard
Mouse
Optionally ESP Install

## Setup python
NOTE: It is best to start with a freshly flashed Nano or Jetson device. The OPEN GL libraries/paths should not have been altered or this likely won't work. If you need to set up the Nano to run headless and then you will have to start the X service. Kinect relies on OPEN GL and linux doesn't load OPEN GL libraries if X isn't started.

You will need to have OpenCV installed. I used 4.1. Get the scripts provided by nvidia here: https://github.com/AastaNV/JEP/tree/master/script

Called, install_opencv4.1.1_Jetson.sh the release numbers will change as the script is periodically updated. This is the easiest way to get the correct opencv on the Nano.

You can git clone or open the script as raw and copy paste onto your Nano. Run the script. It can take several hours to finish.

pip install Cython to your python3 environment that has opencv.


## Setting up Linux box for kinect v2

Clone the openKinect git project
git clone https://github.com/OpenKinect/libfreenect2.git

cd into wherever you cloned libfreenect2
cd libfreenect2

Get linux dependencies
sudo apt-get install build-essential cmake pkg-config

sudo apt-get install libusb-1.0-0-dev

sudo apt-get install libturbojpeg0-dev

sudo apt-get install libglfw3-dev

sudo apt-get install libopenni2-dev

create build directory and cd into it
mkdir build

cd build

build freenect2
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2

make

make install

sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

cd into the build directory
cd libfreenect2/build

Test is works
sudo ./bin/Protonect

With the kinect plugged in you should see the Xbox symbol light up, and 3 infrared lights should glow in front on the Kinect V2.

NOTE: Currently you will need a monitor plugged in for this to work.

Setting up Python Bindings
cd ~

git clone https://github.com/r9y9/pylibfreenect2.git

edit your bashrc
sudo ~/.bashrc

add the following exports to your bashrc file, the paths may need to change depeneding on where you set up libfreenect2
export LIBFREENECT2_INSTALL_PREFIX=/home/$USER/libfreenect2/

export LD_LIBRARY_PATH=/home/$USER/freenect2/lib:$LIBFREENECT2_INSTALL_PREFIX:$LD_LIBRARY_PATH

save your bashrc file and source it
source ~/.bashrc

Copy "config.h" & "export.h" from /home/$USER/libfreenect2/build/libfreenect2/ and install them in: /home/$USER/libfreenect2/include/libfeenect2/

to test run: ${LIBFREENECT2_INSTALL_PREFIX}include/libfreenect2/config.h if everything is in right place it will return: bash: /home/$USER/libfreenect2/include/libfreenect2/config.h:permission denied

copy "lib" folder from :/home/$USER/libfreenect2/build/ and put in: /home/$USER/libfreenect2/

pip install pylibfreenect2 into the environment with opencv

pip install git+https://github.com/r9r9/pylibfreenect2

if this doesn't work, you may have to edit the setup.py line 16
libfreenect2_install_prefix = os.environ.get(
"LIBFREENECT2_INSTALL_PREFIX", "/usr/local/")
change this to the path where you set up libfreenect, I have my path in the comment below
libfreenect2_install_prefix = os.environ.get(
"LIBFREENECT2_INSTALL_PREFIX", "/home/scott/libfreenect2")
then manually run the setup
cd /home/$USER/pyliqbfreenect2/examples

to test install run :$python selective_stream.py

### Setup for ESP

To send depth images to ESP we can use the ezpubsub.py library

I export the following environment variables before starting the ESP Server

Create the following file envkinect.sh


 
 ```bash
 #! /bin/bash
export LIBFREENECT2_INSTALL_PREFIX=/home/scott/libfreenect2/
export DFESP_HOME=/home/scott/al6_deploy/SASEventStreamProcessingEngine/6.2
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$DFESP_HOME/lib:$DFESP_HOME/lib/tk:$DFESP_HOME/ssl/lib:/home/scott/freenect2/lib:$LIBFREENECT2_INSTALL_PREFIX:$LD_LIBRARY_PATH
source envkinect.sh
```


Create the following ESP model to read in the depth images called kinectmodel.xml

```xml
<engine>
  <projects>
    <project name="Project" pubsub="auto" threads="8">
      <contqueries>
        <contquery name="contquery" timing-threshold='50' >
          <windows>
            <window-source index="pi_EMPTY" insert-only="true" name="w_data">
              <schema>
                <fields>
                  <field key="true" name="id" type="int64" />
                  <field key="false" name="_color_image_" type="blob"/>
                  <field key="false" name="_depth_image_" type="blob"/>
                </fields>
        </schema>
 
            </window-source>
 
            <window-counter name = "count" count-interval='5 seconds'>
 
               </window-counter>
 
       </windows>
 
       <edges>
 
               <edge role="data" source="w_data" target="count" />
 
          </edges>
 
        </contquery>
      </contqueries>
    </project>
  </projects>
</engine>
```



Start the esp server: $DFESP_HOME/bin/dfesp_xml_server -http 9910 -pubsub 9912 -model file://kinectmodel.xml


If you have a display connected then you don't need to work about the X service.

In another terminal source envkinect.sh

Create the python publisher script below and save it

```python
#! /usr/bin/python3
import cv2
import os
import numpy as np
import ezpubsub as ezps
import base64
import time
import sys
import signal
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
 
 
 
def signal_handler(sig, frame):
    global terminate
    terminate=True
 
def array2bytestring(array):
    return cv2.imencode('.jpg', array)[1].tostring()
 
 
if __name__ == "__main__":
    ezps.init_pubsub()
 
    schema = ezps.Schema.from_string("id*:int64, _color_image_:blob, _depth_image_:blob")
 
    pubUrl = "dfESP://192.168.1.39:9912/Project/contquery/w_data"
 
    publisher = ezps.Publisher(pubUrl)
 
 
    try:
        from pylibfreenect2 import OpenGLPacketPipeline
        pipeline = OpenGLPacketPipeline()
    except:
        try:
            from pylibfreenect2 import OpenCLPacketPipeline
            pipeline = OpenCLPacketPipeline()
        except:
            from pylibfreenect2 import CpuPacketPipeline
            pipeline = CpuPacketPipeline()
    print("Packet pipeline:", type(pipeline).__name__)
 
    enable_depth = True
    enable_rgb = True
    fn = Freenect2()
    num_devices = fn.enumerateDevices()
    if num_devices == 0:
        print("No device connected!")
        sys.exit(1)
    serial = fn.getDeviceSerialNumber(0)
    device = fn.openDevice(serial, pipeline=pipeline)
    types = 0
    if enable_rgb:
        types |= FrameType.Color
    if enable_depth:
        types |= (FrameType.Ir | FrameType.Depth)
    listener = SyncMultiFrameListener(types)
 
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)
    if enable_rgb and enable_depth:
        device.start()
    else:
        device.startStreams(rgb=enable_rgb, depth=enable_depth)
 
    start_time = time.monotonic()
    count=int(start_time)
    terminate=False
    signal.signal(signal.SIGINT, signal_handler)
 
 
    while True:
        if terminate:
            break
        frames = listener.waitForNewFrame()
 
        if enable_rgb:
            color = frames["color"]
            color = color.asarray()
            color_string = array2bytestring(color)
        if enable_depth:
            #ir = frames["ir"]
            depth = frames["depth"]
            depth = (depth.asarray()*255/4500.0).astype(np.uint8)
            depth_string = array2bytestring(depth)
 
 
 
        #print(depth_string)
 
        row = {'id': count, '_depth_image_': depth_string, '_color_image_': color_string }
        ev = ezps.Event.from_row(schema, row)
        publisher.publish_row(row)
        count+=1
        listener.release(frames)
 
    print("Closing camera\n")
    device.stop()
    device.close()
    print("Shutting down pubsub")
    ezps.shutdown_pubsub()
    end_time = time.monotonic()
    total_frames = int(count)-int(start_time)
    total_time = end_time - start_time
    frames_per_second = int(total_frames/total_time)
    print("Published {} frames in {} seconds, averaging ~ {} fps ".format(total_frames, total_time, frames_per_second))
```

### Setting Up to Run Headless
To run Kinect when no display is kinected you will need to follow the directions here:

https://gist.github.com/shehzan10/8d36c908af216573a1f0

Essentially you need to get the X server running so that the OPEN GL libaries are loaded

/usr/bin/X :0 &

export DISPLAY=:0

Before you start your kinect publisher. If you are running headless and the kinect V2 doesn't have the 3 IR lights illuminated then it is not running likely due to the OPEN GL libraries not loading.

### Visualizing in Python
The maximum depth of the Kinect V2 is 4.5 meters. In our python publisher we scaled the 16 bit depth data to uint8. If we want to calculate depths from the uint8 image in ESP then we need to just reverse the scaling. The Kinect V2 has a practical range of 1.2–3.5 m. This means that it gives the best resolution at these distances. Its minimum range is 0.5 m. In the depth images produced objects closer than 0.5 m will have a value of 0 and objects greater than 4.5 m will have a value of 0. Also any pixel where signal was not able to be reflected will produce a value of zero. Usually noise will be 0. Some types of geometries are not collected accurately because the light is never reflected back so these will also produce 0.


```python
import esppy
import cv2
import numpy as np
# make connection to server on http port
 
def bytestring2array(bstring):
    nparr = np.frombuffer(bstring, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
 
    return img
 
def depth_pixel_to_inch(pix):
    return pix*4500/255/25.4
 
con = esppy.ESP("192.168.1.39",9910)
 
data_table = con.get_window("w_data")
data_table.subscribe(mode="streaming", limit=5)
 
tmp_table = data_table.tail().iloc[-1].copy()
 
tmp_image = bytestring2array(tmp_table._depth_image_)
 
tmp2 = tmp_image.copy()
tmp_bw = tmp2[:,:,0]
plt.hist(tmp_bw.ravel())
 
# part of my head is at 150, 275
print(tmp_image[150,275, 0])
# this printed 55
print(depth_pixel_to_inch(55)
# prints 38.2, which seems reasonable
 
 
 
# color stream
data_table.streaming_images("_color_image_")
# depth stream, shown in gray scale
data_table.streaming_images("_depth_image_")
```
