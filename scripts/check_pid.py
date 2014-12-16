import time

class PID:
    def __init__(self,p,i,d):
        self.kp = p
        self.ki = i
        self.kd = d
        self.last_error = float("inf")
        self.last_time = 0.0
        self.error_sum = 0.0
    def reset(self):
        self.last_error = float("inf")
        self.last_time = 0.0
        self.error_sum = 0.0
    def getCommand(self,input_value):
        timer = time.time()
        dt = (timer-self.last_time) / 1000
        de = 0.0
        if self.last_time != 0.0:
            if self.last_error < float("inf"):
                de = (input_value - self.last_error) / dt
            self.error_sum += input_value * dt
        self.last_time = timer
        self.last_error = input_value
        output = self.kp * input_value + self.ki * self.error_sum + self.kd * de
        return output
    def get_error_sum(self):
        return self.error_sum
        
        
yawPID = PID(1.0,0.0,0.30)
print yawPID.getCommand(-53.450774)
print yawPID.getCommand(-53.450773)
print yawPID.getCommand(-53.450772)
print yawPID.getCommand(-53.450771)
print yawPID.getCommand(-53.450770)
print yawPID.getCommand(-53.450769)
print yawPID.get_error_sum()
