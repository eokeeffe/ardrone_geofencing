import math
R = 6371 # in km

y = [-6.23327135789736,
-6.233952638987148,
-6.233823892954433,
-6.233282086733425]

x = [53.30946090774625,
53.30948334414638,
53.30966283492298,
53.30965642455111]

print "Calculating area"

# dist = arccos(sin(lat1) * sing(lat2) + cos(lat1) * cos(lat2) * cos(lon1-lon2)) * R

dist = []
for i in range(0,len(x)-1):
    dist.append( 
        math.acos(
            math.sin(math.radians(x[i])) * 
            math.sin(math.radians(x[i+1])) + 
            math.cos(math.radians(x[i])) * 
            math.cos(math.radians(x[i+1])) * 
            math.cos(math.radians(y[i]) - math.radians(y[i+1]))
        ) * R   
    )

# last distance to complete polygon
dist.append(
    math.acos(
            math.sin(math.radians(x[0])) * 
            math.sin(math.radians(x[-1])) + 
            math.cos(math.radians(x[0])) * 
            math.cos(math.radians(x[-1])) * 
            math.cos(math.radians(y[0]) - math.radians(y[-1]))
     ) * R  
)

for d in dist: print d

print "total area = ", sum(dist),"Km squared"
