# SAND-GRAIN-SIZE-EVALUATOR
drive link that shows working: https://drive.google.com/drive/folders/1xgOGHSTq3k1kqSckc6GRWQICgBZ23wUu
this project is based on calculating sand grain sizes  based on computer vision 
my approach was to first fix the height of the camera to obtain a standard photo that had consistent/same real area/land covered per pixel 
to achieve that i placed 5 ultrasonic sensors at the back and sorted their readings in an array and calculated the mean distance taking only those readings into account that were in the range of median/1.5 to median*1.5 because dune beaches have irregular surface so i had to assume a virtual plane and now since the height was fixed i added a mpu6050 accelerometer/gyroscope to make it so that the camera only takes photos when the device is perpendicular to the ground and the mean distance is 9-11cm then i calculated the real area of image using the hfov and vfov of the camera to calibrate area/px and then after using water segmentation to find out on an average how many pixels does one sand grain occupies i calculated the grain diameter 
now to showcase  this data i used a neo6mnv2 gps that utilizes ublox gps chip to get coordinates and stored all this data in a csv file 

now what exactly does my upload.py file do??
 it checks if the device is connected to the internet by pinging 8.8.8.8 (google's dns server) and if it is online then it pushes all the csv data to a mongodb backend server deployed on render otherwise it waits for the device to come online 

 Hardware USED:
    RPI 2b+ (main processing and controlling)
    arduino leonardo (Takes all the sensor readings)
    5x ultrasonic sensors
    1 usb webcam
    rpi 3.5inch tft display
    neo6mnv2 gps sensor
    mpu6050 accelerometer/gyroscope
    
    
