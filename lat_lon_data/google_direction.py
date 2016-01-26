# coding: UTF-8

import re
from string import *
from urllib import urlopen


#-----------始点終点情報の緯度経度の記入--------------------
origin_lat = 35.578240
origin_lon = 139.544407
destination_lat = 35.581206
destination_lon = 139.536964
#---------------------------------------------------------

origin = str(origin_lat) + "," + str(origin_lon) 
destination = str(destination_lat) + "," + str(destination_lon)


url = "http://maps.googleapis.com/maps/api/directions/json?origin="+origin+"&destination="+destination+"&sensor=false"

indir = "/home/amsl/AMSL_ros_pkg/rwrc13/rwrc13_waypoint_manager/waypoint_data/"
infile = indir + "waypoint.txt"
#outfile = indir + "waypoint_latlon.txt"
outfile_ = indir + "waypoint_latlon_.txt"

f = open(infile, "r")
line = f.readline()
o = open('outfile_', 'w')

i = 0
j = 0
l = 0
latlist = ["0","0"]
#for line in urlopen(url):
while line:
      if re.search('lat', line):
         #print i
         if i>=2:
            str = line
            lat = strip(str.rsplit(' ', 1)[1])
            lat = strip(lat.rsplit(',', 1)[0])
            latlist.append (lat)
            j=0
            l = 0
            while j < i:
               #print j
               #print latlist[j]
               if latlist[i] == latlist[j]:
                  l = 1
                  j = i
               j += 1
    
            if l==0 and i>=4:#初め4つのlat,lon dataは始点終点の座標が入っているから除いた
               o.write(latlist[i])
               #print latlist[i]
         #elif i<=1:
            #print ""
         i = i+1   

      elif re.search('lng', line):
         if i>=5:
            str = line
            lng = strip(str.rsplit(' ', 1)[1])
            if l==0:
               o.write(' ' + lng + '\n')
               #print lng
         #elif i<=2:
            #print ""

      line = f.readline()

f.close
print "Complete!"
o.close
'''
indat = open(infile, "r")

inline = indat.readline()

data_dic = {}

print ("Start reading data.")

f = open('outfile_', 'w')
'''
'''
while inline:
 
 #   data_list = []
 l = 26
 if re.search('Lat', inline):
  lat = strip(re.split(' ', inline)[18])
  f.write(lat)
  print l
  #print ("if bunn.")
  print lat
 elif re.search('Long', inline):
  lng = strip(re.split(' ', inline)[1])
  f.write(lng + '\n')
  #print ("if bunn22.")
  print lng

#     data_list = [lat,lng]
#     data_dic[uid] = data_list
 inline = indat.readline()
indat.close()
f.close()

o = open(outfile, "w")

o.write("latitude,longitude\n")

print ("start writing an output file.")

for d in data_dic.keys():
 
 unpacked = data_dic[d]
 data_str = ",".join([elem for elem in unpacked])
 o.write(str(d) + "," + data_str + "\n")

o.close()

'''
