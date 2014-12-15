import sys,csv
from math import *

def futureGPSposition(d,lat,lng,heading):
    radiusEarthKm = 6371
    radiusEarthM = radiusEarthKm * 1000
    bearing = radians(heading)
    
    newlat = degrees((d/radiusEarthM) * cos(bearing)) + lat
    newlng = degrees((d/(radiusEarthM*sin(radians(newlat)))) * sin(bearing)) + lng
    return newlat,newlng
    
def simpleWithin(coordinates,lat,lng):
    print "Checking if",lat,",",lng,"is within perimeter"
    lat = degrees(lat)
    lng = degrees(lng)
    j = len(coordinates)-1
    pointStatus = False
    for i in xrange(0,len(coordinates)):
        lati = degrees(float(coordinates[i][0]))
        latj = degrees(float(coordinates[j][0]))
        lngi = degrees(float(coordinates[i][1]))
        lngj = degrees(float(coordinates[j][1]))
        
        if (lngi < lng and lngj >= lng) or (lngj < lng and lngi >= lng):
            if lati + (lng - lngi) / (lngj - lngi) * (latj - lati) < lat:
                pointStatus = not(pointStatus)
        j = i
    return pointStatus

if len(sys.argv) < 2:
    print "No CSV file given"
    exit(0)

coordinates = []
f = open(sys.argv[1],"r")
try:
    reader = csv.reader(f)
    for row in reader:
        print row
        coordinates.append(row)
finally:
    f.close()

print simpleWithin(coordinates,53.30729,-6.23220)
print simpleWithin(coordinates,53.3077,-6.2330)
# Fly the drone towards the center of the pitch
# from the top left hand corner of it
lat,lng = futureGPSposition(76.92,53.3077,-6.2330,136.1714)
print "future Lat,Lng:",lat,lng
print simpleWithin(coordinates,lat,lng)
