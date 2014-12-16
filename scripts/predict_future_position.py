from math import *

R = 6371
d = 15/1000

angleRadHeading = radians(185.4114)

lat = 53.3077
lon = -6.2330

lat1 = radians(lat) #Current lat point converted to radians
lon1 = radians(lon) #Current long point converted to radians

lat2 = asin(sin(lat1)*cos(d/R) + cos(lat1)*sin(d/R)*cos(angleRadHeading))
lon2 = lon1 + atan2(cos(d/R)-sin(lat1)*sin(lat2), sin(angleRadHeading)*sin(d/R)*cos(lat1))

print "%.6f" % lat2,"%.6f" % lon2

lat2 = degrees(lat2)
lon2 = degrees(lon2)

print "Projection"
print "%.6f" % lat2,"%.6f" % lon2
