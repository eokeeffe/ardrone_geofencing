from math import *

init_lat = 53.3077
init_lon = -6.2330
s = 15
bearing = 185.4114

def vincenty_direct(init_bearing,distance,lat,lon):
    '''
        Vincenty Direct Formula
        @init_bearing = initial bearing
        @distance = distance from point
        @lat = current latitude
        @lon = current longitude
        
        returns latitude,longitude,final bearing
    '''
    angleRadHeading = radians(init_bearing)
    lat0 = radians(lat) #Current lat point converted to radians
    lon0 = radians(lon) #Current long point converted to radians
    
    a = 6378388
    b = 6356911.946
    f = 1.0/297.0
    
    sina1 = sin(angleRadHeading)
    cosa1 = cos(angleRadHeading)
    
    tanU1 = (1-f) * tan(lat0)
    cosU1 = 1.0 / sqrt((1+tanU1*tanU1))
    sinU1 = tanU1 * cosU1
    
    sigma1 = atan2(tanU1,cosa1)
    sina = cosU1 * sina1
    cosSqa = 1 - sina*sina
    uSq = cosSqa * (a*a-b*b)/(b*b)
    A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)))
    B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)))
    
    cos2sigmaM = 0.0
    sinsigma = 0.0
    cossigma = 0.0
    delta_sigma = 0.0
    
    sigma = distance / (b*A)
    sigma_transpose = 0.0
    iterations = 0
    value = True
    while value:
        cos2sigmaM = cos(2*sigma1+sigma)
        sinsigma = sin(sigma)
        cossigma = cos(sigma)
        delta_sigma = B*sinsigma*(cos2sigmaM+B/4*(cossigma*(-1+2*cos2sigmaM*cos2sigmaM)-B/6*cos2sigmaM*(-3+4*sinsigma*sinsigma)*(-3+4*cos2sigmaM*cos2sigmaM)))
        sigma_transpose = sigma
        sigma = distance / (b*A) + delta_sigma
        if fabs(sigma-sigma_transpose) > 1e-12 and ++iterations<200:
            value = False
    if iterations>=200:
        raise Exception("Formula failed to converge")
    
    x = sinU1*sinsigma - cosU1*cossigma*cosa1
    lat2 = atan2(sinU1*cossigma + cosU1*sinsigma*cosa1,(1-f)*sqrt(sina*sina+x*x))
    delta = atan2(sinsigma*sina1,cosU1*cossigma - sinU1*sinsigma*cosa1)
    
    C = f/16.0*cosSqa*(4+f*(4-3*cosSqa))
    L = delta - (1-C) * f * sina * (sigma + C*sina*(cos2sigmaM+C*cossigma*(-1+2*cos2sigmaM*cos2sigmaM)))
    lon2 = (lon0+L+3*pi)%(2*pi) - pi#normalise to -180...+180
    
    a2 = atan2(sina,-x)
    a2 = (a2+2*pi)%(2*pi)#normalise to 0...360
    
    return degrees(lat2),degrees(lon2),degrees(a2)

print vincenty_direct(bearing,s,init_lat,init_lon)
