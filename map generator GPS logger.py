import orjson
import math
import time
import datetime
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tkinter
import scipy
from scipy import interpolate
import numpy as np
#import pyrosm 
#from pyrosm import OSM
#from pyrosm import get_data
from bs4 import BeautifulSoup
import string

# Hardcoded Settings ############################
Zoom_level_min = 0.005


# User Settings #################################
Max_Node_Distance = 500 # Metres
Start_Datetime = [2017,1,1,0,0,0] # Y,M,D,H,M,S
End_Datetime = [2030,12,31,23,59,59] # Y,M,D,H,M,S
Draw_Line = True
Animated = False
Frames = 365*4
Resolution = (1080*4, 1080*4)
Minimumm_Accuracy = 16
center_point = [43.46371, -80.52992]
p1_lat = center_point[0]
p1_long = center_point[1]
GPS_Data_Files = ["E:\Programming\Projects\maps\LocationData.json"] #, "E:\Programming\Projects\maps\Location History 08-07-2023.json"]

# Spline Control Points ####################
Zoom = [0.002, 0.002]
Zoom_frames = [0, 365*3]
Zoom_slopes = [0, 0]

Lat_center_points = [43.29, 43.29]
Lat_center_frames = [0, 365*3]
Lat_center_slopes = [0, 0]

Long_center_points = [-80.27, -80.27]
Long_center_frames = [0, 365*3]
Long_center_slopes = [0, 0]


# Loads data #########################
location_lists = []

# f = open(r"F:\DATA_280226.gpx", "r")

import xml.etree.ElementTree as ET
tree = ET.parse('F:\DATA_080326.gpx')
root = tree.getroot()

i = 0

"""
while i < len(root[71][2]):
    print(root[71][2][i].attrib["lat"])
    print(root[71][2][i].attrib["lon"])
    print(root[71][2][i][0].text)
    print(root[71][2][i][1].text)
    
    i+=2
"""

#print(len(root))
#print(len(root[1]))
#print(len(root[1][2]))

i = 1
j = 2
k = 0

number_of_points = 0


for i in range(1,len(root)):
    for k in range(0,len(root[i][j]),3):
        #print(root[i][j][k].attrib["lat"])
        #print(root[i][j][k].attrib["lon"])
        #print(root[i][j][k][0].text)
        #print(root[i][j][k][1].text)
        number_of_points += 1

#print(number_of_points)

data_points = np.zeros((number_of_points, 7))

x = 0

for i in range(1,len(root)):
    for k in range(0,len(root[i][j]),4):
        data_points [x][0] = i-1                                                                # track segment number
        data_points [x][1] = float(root[i][j][k].attrib["lat"])                                 # latitude
        data_points [x][2] = float(root[i][j][k].attrib["lon"])                                 # longitude
        data_points [x][3] = float(root[i][j][k][0].text)                                       # elevation
        data_points [x][4] = float(root[i][j][k+1].text) + float(root[i][j][k][1].text[-5:-1])  # unix time with decimal
        data_points [x][5] = float(root[i][j][k+2].text)                                        # HDOP
        data_points [x][6] = float(root[i][j][k+3].text)                                  # speed over land km\h
        x += 1
       

#for z in range(number_of_points):
    #print(data_points[z])


# Functions ###########################
lat_height = 0.02
long_width = Resolution[0]/Resolution[1] * lat_height


lat1 = p1_lat - lat_height/2
lat2 = p1_lat + lat_height/2
long1 = p1_long - long_width/2
long2 = p1_long + long_width/2


stepsizelong = (long2 - long1)/Resolution[0]
stepsizelat = (lat2 - lat1)/Resolution[1]

img = Image.new('RGBA', (Resolution[0], Resolution[1]), (0, 0, 0, 255))

draw = ImageDraw.Draw(img)


for i in range(number_of_points-1):
    distance = (2*6378.137*1000) * math.asin(math.sqrt((math.sin((math.radians(data_points[i][1] - data_points[i+1][1]))/2)**2) + math.cos(math.radians(data_points[i][1])) * math.cos(math.radians(data_points[i+1][1])) * (math.sin(math.radians(data_points[i][2] - data_points[i+1][2])/2)**2)))
    
    if i % 100 == 0:
        print((i/number_of_points)*100)

    if distance <= 750:
        lat_offset = math.sin(math.radians(data_points[i][1]-p1_lat)) / math.sin(math.radians(lat_height/2))
        long_offset = (math.cos(math.radians(data_points[i][1]-p1_lat)) / math.sin(math.radians(lat_height/2))) * math.sin(math.radians(data_points[i][2]-p1_long))
        lat_offset2 = math.sin(math.radians(data_points[i+1][1]-p1_lat)) / math.sin(math.radians(lat_height/2))
        long_offset2 = (math.cos(math.radians(data_points[i+1][1]-p1_lat)) / math.sin(math.radians(lat_height/2))) * math.sin(math.radians(data_points[i+1][2]-p1_long))

        font = ImageFont.truetype(r'c:\Users\Brian\Desktop\AdobeGothicStd-Bold.otf', 50)

        if Draw_Line == True:
            Fill = (int(255*(data_points[i][6]/15)),255-int(255*(data_points[i][6]/15)),0)

            draw.line(
                        (
                        long_offset*Resolution[0]/2+Resolution[0]/2, 
                        -lat_offset*Resolution[1]/2+Resolution[1]/2,
                        long_offset2*Resolution[0]/2+Resolution[0]/2, 
                        -lat_offset2*Resolution[1]/2+Resolution[1]/2
                        ), 
                        fill = Fill 
                    )   
            
img.save("Static GPS tracker test.png")