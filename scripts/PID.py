from datetime import datetime

class PidController:
    def __init__(self,kp,ki,kd=0):
        self.kp,self.ki,self.kd=kp,ki,kd
        self.i=0
        self.d=0
        self.prev=0
        self.time=0
    def work(self,e):
        time_now = datetime.now().microsecond
        DT = time_now - self.time
        self.i += DT*e
        self.d = (e-self.prev)/DT
        self.prev=e
        self.time = time_now
        
        return self.kp*e+self.ki*self.i+self.kd*self.d

class AdvController():
    def __init__(self,kp,ki,kd=0, clamp=(-1e10,1e10), smooth=1):
        self.kp,self.ki,self.kd=kp,ki,kd
        self.i=0
        self.d=0
        self.prev=0
        self.unclamped=True
        self.clamp_lo,self.clamp_hi = clamp
        self.alpha = smooth
        self.time=0
    def work(self,e):
        time_now = datetime.now().microsecond
        DT = time_now - self.time
        if self.unclamped:
            self.i += DT*e
        self.d = (self.alpha*(e-self.prev)/DT + (1.0-self.alpha)*self.d)
        u = self.kp*e+self.ki*self.i+self.kd*self.d
        self.unclamped = (self.clamp_lo<u<self.clamp_hi)
        self.prev = e
        return u

yaw = PidController(1.0,0.0,0.30)
print yaw.work(180)
print yaw.work(70)
print yaw.work(40)
del yaw
yaw = AdvController(1.0,0.0,0.30)
print yaw.work(180)
print yaw.work(70)
print yaw.work(40)
