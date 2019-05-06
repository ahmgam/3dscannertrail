# 3dscannertrail
This project was a draft for creating a single camera 3d scanner using opencv-python

the main idea of project is using low cost usb camera to produce 3d scanning for any object

to create full scanning; object has to be scanned from all directions, so i designed a mechanical frame to give the camera 360 degree view for the scanned object 

<img src="https://i.ibb.co/d2WtXZ8/mc.png" >

the frame designed to keep the camera focusing on the center of the table

the idea i wanted to test is computing depth map using 2 frames with closed angles ex: 5 degrees, so if i got 8 images from 4 directions, i can then have full scan from this vertical angle 

the accuracy will depend on the number of views i took to the object from different agnles , i intended to make this elemnt controlled by the user of the scanner

i tried the code here , but i had some problems that i couldn't get a valid disparity map until now 

i used this images for calipration of the camera 

left side :

<img src=https://i.ibb.co/xz5SXNC/Leftside.jpg>

right side : 

<img src=https://i.ibb.co/zb8mvK0/Rightside.jpg>

i tried to calbrate the camera by taking 2 pictures from carfully spicified points and orientations , for testing the code if it works fine 

i used this samble to check the disparity map algorithm after calibrating 
left side : 

<img src=https://i.ibb.co/k1Xhkdz/Leftobject.jpg>

right side 

<img src=https://i.ibb.co/mhnGBgG/Rightobject.jpg>

after undistortion process; i got this 2 images (befor corping):
left side :

<img src=https://i.ibb.co/Wpjkr7S/befleftundis.jpg>

right side : 

<img src=https://i.ibb.co/F3BvqVM/befrightundistor.jpg>

the left side looks fine, but the right side don't , after corping; the right side totally vanished, the image is blank

the left side after corping : 

<img src=https://i.ibb.co/cwLwJKk/leftundis.jpg>

the code shows error then, as the left side images dosen't has the same size of the right side image, i relized that rightROI gives [0 0 0 0] , i used the calibration data of the left side , i got this for the right side image  :

before croping : 

<img src=https://i.ibb.co/cCqL3fQ/befrightundistor.jpg>

after croping :

<img src=https://i.ibb.co/rQrRmhJ/rightundistor.jpg>

the disparity map : 

<img src=https://i.ibb.co/tbrHMZ4/disparity.jpg>

it's meaningless.

possible reasons for this error (in my opinion):

1- i have to use stereocamera, as the spacing between the two lenses is constant, the view then would be the same for any captured images

2- the tested spacing between the two viewes of the camera is too large for the algorithm to have a reasonable results

3- there is a problem in right side calibration .

4- there is a problem in stereoBM configuration 

5- finally - there may be a problem in the disparity map encoding, some data may be lost in conversion between the output format and the final format (float32).

so if anyone could help solving this problem, i would be greatful .

thanks in advance.
