class Kalman:
    def __init__(self):
        self.QAngle = 0.003   #  Q is the process noise covariance
        self.QBias = 0.001
        self.RMeasure = 0.01   # R is the measurement noise covariance
        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.A=[[0.0,0.0],[0.0,0.0]]


    def get_angle(self, new_angle, new_rate, dt):
        
        self.rate = new_rate - self.bias;   
        self.angle += dt * self.rate;

        self.A[0][0] += dt * (dt*self.A[1][1] -self.A[0][1] - self.A[1][0] + self.QAngle)
        self.A[0][1] -= dt * self.A[1][1]
        self.A[1][0] -= dt * self.A[1][1]
        self.A[1][1] += self.QBias * dt

        y = new_angle - self.angle
        
        # Covariance
        s = self.A[0][0] + self.RMeasure

        # Kalman Gain
        K=[0.0,0.0]
        K[0] = self.A[0][0]/s
        K[1] = self.A[1][0]/s

        # UAdate the Angle
        self.angle += K[0] * y
        self.bias  += K[1] * y

        # Calculate estimation error covariance and uAdate the error covariance
        A00TempA = self.A[0][0]
        A01TempA = self.A[0][1]

        self.A[0][0] -= K[0] * A00TempA;
        self.A[0][1] -= K[0] * A01TempA;
        self.A[1][0] -= K[1] * A00TempA;
        self.A[1][1] -= K[1] * A01TempA;

        return self.angle


    def set_angle(self,angle):
        self.angle = angle

    def set_QAngle(self,QAngle):
        self.QAngle = QAngle

    def set_QBias(self,QBias):
        self.QBias = QBias

    def set_RMeasure(self,RMeasure):
        self.RMeasure = RMeasure

    def get_Rate():
        return self.rate

    def get_QAngle():
        return self.QAngle

    def get_QBias():
        return self.QBias

    def get_RMeasure():
        return self.RMeasure
