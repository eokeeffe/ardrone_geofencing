from math import *

radiusEarthKm = 6371
radiusEarthM  =  radiusEarthKm*1000
d = 15

angleRadHeading = radians(185.4114)
lat0 = 53.3077
lng0 = -6.2330
Speed = 15
time_interval = 5

kmDistance = Speed * (time_interval/1000) / 3600

distRatio = kmDistance/radiusEarthM
distRatioSine = sin(distRatio)
distRatioCosine = cos(distRatio)

startLatRad = lat0 #radians(lat0)
startLongRad = lng0#radians(lng0)

startLatCos = cos(startLatRad)
startLatSin = sin(startLongRad)

endLatRad = asin(
    (startLatSin*distRatioCosine) +
    (startLatCos*distRatioSine * cos(angleRadHeading))
)

endLongRad = startLongRad + atan2(sin(angleRadHeading) *
    distRatioSine * startLatCos, 
    distRatioCosine - startLatSin * sin(endLatRad)
)

newLat = degrees(endLatRad)
newLong = degrees(endLongRad)
print "Radians"
print "Future Lat:",endLatRad
print "Future Lon:",endLongRad
print "Degrees"
print "Future Lat:",newLat
print "Future Lon:",newLong


lat2 = degrees((d/radiusEarthM) * cos(angleRadHeading)) + lat0
long2 = degrees((d/(radiusEarthM*sin(radians(lat2)))) * sin(angleRadHeading)) + lng0

print "Second Implementation"
print "Lat:",lat2,",Lng:",long2
