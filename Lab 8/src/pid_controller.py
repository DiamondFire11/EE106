class PIDController:
    def __init__(self, p, d, i):
        self.kp = p
        self.kd = d
        self.ki = i

        # Set point
        self.set_point = 0
        self.err = 0

        # Derivative prev err
        self.prev_err = 0

        # Integral error tracker
        self.integrator = 0

    def proportional(self):
        return self.kp * self.err

    def integral(self):
        self.integrator += self.err
        return self.ki * self.integrator

    def derivative(self):
        der_val = self.kd * (self.err - self.prev_err)
        self.prev_err = self.err
        return der_val

    def get_control_signal(self, output):
        self.err = output - self.set_point
        signal = self.proportional() - self.derivative() + self.integral()
        return signal if signal <= 0.30 else 0.30

    def update_set_point(self, new_set_point):
        self.set_point = new_set_point
