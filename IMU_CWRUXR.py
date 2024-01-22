"""
Created on Mon Sept 20th 2023

@author: jxl2261 
"""
# CWRUXR and other ncessary imports
# CWRUXR version: cwruxr-sdk 1.2.1
from cwruxr_sdk.common import *
from cwruxr_sdk.client import *
from cwruxr_sdk.object_message import *
from cwruxr_sdk.anchor_message import *
from cwruxr_sdk.material_message import *
import serial

# Room Information, Anchor, and Client creation for CWRUXR
ENDPOINT = "https://cwruxrstudents.azurewebsites.net/api/v2/"
ROOM_ID = "Jia"
ANCHOR_ID = "Anchor1"
cwruxrClient = Client(ENDPOINT, ROOM_ID, ANCHOR_ID)

#Functions for CWRUXR object setup----------------------------------------------------------------------------------
def createCube(NewPos,NewRot): #input vector3 position in m/s^2 and quarternion rotation to create a cube
    cube = PrimitiveMessage(
    id = "cube1",
    source = PRIMITIVE_CUBE,
    pose = Pose(
        position = NewPos,
        rotation = NewRot,
        scale= Vector3(.3,.1,.2)),
    isManipulationOn = True,
    )
    return cube

#CWRUXR text data displays
def accelTextDisplay(accelx,accely,accelz):
    acceleration_text = TextMessage(
        id = "accel_text",
        pose = Pose(
            position=Vector3(0.7,1.6,0),
            euler=Vector3(0,0,0),
            scale=Vector3(0.5,0.5,0.5)
        ),
        active = True,
        parameters = TextParameters(
            text = "Acceleration in x (m/s^2): {}\n".format(accelx)+
                   "Acceleration in y (m/s^2): {}\n".format(accely)+
                   "Acceleration in z (m/s^2): {}\n".format(accelz),
            fontSize = 1,
            width= 2,
            height= 1,
            color= Color(255,255,0,255)
        )
    )
    return acceleration_text

def velTextDisplay(velx,vely,velz):
    velocity_text = TextMessage(
        id = "vel_text",
        pose = Pose(
            position=Vector3(0.7,1.3,0),
            euler=Vector3(0,0,0),
            scale=Vector3(0.5,0.5,0.5)
        ),
        active = True,
        parameters = TextParameters(
            text = "Velocity in x (m/s): {}\n".format(velx)+
                   "Velocity in y (m/s): {}\n".format(vely)+
                   "Velocity in z (m/s): {}\n".format(velz),
            fontSize = 1,
            width= 2,
            height= 1,
            color= Color(255,255,0,255)
        )
    )
    return velocity_text

def posTextDisplay(posx,posy,posz):
    position_text = TextMessage(
        id = "pos_text",
        pose = Pose(
            position=Vector3(0.7,1,0),
            euler=Vector3(0,0,0),
            scale=Vector3(0.5,0.5,0.5)
        ),
        active = True,
        parameters = TextParameters(
            text = "Position in x (m): {}\n".format(posx)+
                   "Position in y (m): {}\n".format(posy)+
                   "Position in z (m): {}\n".format(posz),
            fontSize = 1,
            width= 2,
            height= 1,
            color= Color(255,255,0,255)
        )
    )
    return position_text

def gyrTextDisplay(gyrx,gyry,gyrz):
    gyr_text = TextMessage(
        id = "gyr_text",
        pose = Pose(
            position=Vector3(0.7,1.3,0),
            euler=Vector3(0,0,0),
            scale=Vector3(0.5,0.5,0.5)
        ),
        active = True,
        parameters = TextParameters(
            text = "Angular Velocity in x (radians): {}\n".format(gyrx)+
                   "Angular Velocity in y (radians): {}\n".format(gyry)+
                   "Angular Velocity in z (radians): {}\n".format(gyrz),
            fontSize = 1,
            width= 2,
            height= 1,
            color= Color(255,255,0,255)
        )
    )
    return gyr_text

def magTextDisplay(magx,magy,magz):
    mag_text = TextMessage(
        id = "mag_text",
        pose = Pose(
            position=Vector3(0.7,1,0),
            euler=Vector3(0,0,0),
            scale=Vector3(0.5,0.5,0.5)
        ),
        active = True,
        parameters = TextParameters(
            text = "Magnetic field in x (uT): {}\n".format(magx)+
                   "Magnetic field in y (uT): {}\n".format(magy)+
                   "Magnetic field in z (uT): {}\n".format(magz),
            fontSize = 1,
            width= 2,
            height= 1,
            color= Color(255,255,0,255)
        )
    )
    return mag_text


# Open the serial port----------------------------------------------------------------------------------------------
ser = serial.Serial('COM9', 115200)
print (ser)

#initialize a position
NewPos=Vector3(0,1,0)

#-----------------------------------------------run time-----------------------------------------------------------
# Serial data method:
# The C++ code in Arduino IDE is set up such that one full cycle of reading sensor datawill output the data in 
# the following format:
#   Acc (m/s^2) [ -0.78,0.24,83.38 ] 
#   Gyr (Radians) [ 687.56,572.97,-458.37 ] 
#   Compass (uT) [ 92.00,23.00,376.00 ] 
#   Rotation Quaternion[ 0.984,0.025,0.025,-0.176 ] 
#
#   We are using .readline() from the serial library to read in serial Port data, problem is each new line contains
#   different string data and will need to be each processed differently to turn into differentiable variables for
#   creating different CWRUXR objects. In order to read and parse that serial data appropriately, in the loop we 
#   can tell python to update data everytime. 

try:
    processing_acceleration = False     #set up flag for processing serial data 
    accel_data = []

    while True:
        data=ser.readline().decode()

        # Check if the line contains accelerometer data
        if 'Acc (m/s^2)' in data:
            processing_acceleration = True  #sets flag to true indicate start of a reading cycle
            # Extract Acceleration data from raw data by first finding start and end of the data string
            accel_data = [float(val) for val in data.split('[')[1].split(']')[0].split(',')] 
            print("Acceleration Data (m/s^2):", accel_data)
            
            accelText=accelTextDisplay(accel_data[0],accel_data[1],accel_data[2])
        
        elif processing_acceleration:
            if 'Rotation Quaternion' in data:
                # Extract rotational quaternion data from raw data by first finding start and end of the data string
                rotQ_data=[float(val) for val in data.split('[')[1].split(']')[0].split(',')]          
                print("Rotational Quaternion:", rotQ_data)

                #create Quaternion data type that CWRUXR message can comprehend
                NewRot=Quaternion(rotQ_data[1],rotQ_data[3],rotQ_data[2],-rotQ_data[0])
                cube=createCube(NewPos,NewRot)
                
                #Axis=showAxis(newRot)

                #Post all the objects that have been updated by the end of this data cycle
                cwruxrClient.PostObjectBulk([cube,accelText,gyrText,magText])

                #reset flag for beginning of the next data reading cycle with accleration data again
                processing_acceleration = False

            elif 'Gyr (Radians)' in data:
            # Extract Gyrometer data from raw data by first finding start and end of the data string
                gyro_data=[float(val) for val in data.split('[')[1].split(']')[0].split(',')]
                print("Gyroscope Values (Radians):", gyro_data)
                
                gyrText=gyrTextDisplay(gyro_data[0],gyro_data[1],gyro_data[2])

            elif 'Compass (uT)' in data:
                # Extract Magnetometer data from raw data by first finding start and end of the data string
                mag_data=[float(val) for val in data.split('[')[1].split(']')[0].split(',')]
                print("Magnetometer Values (uT):", mag_data)
                
                magText=magTextDisplay(mag_data[0],mag_data[1],mag_data[2])


except KeyboardInterrupt:
    # Close the serial port when the script is interrupted
    ser.close()


