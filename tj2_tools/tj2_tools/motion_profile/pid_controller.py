class PIDController:
    def __init__(self, **kwargs):
        self.kp = kwargs.get("kp", 1.0)
        self.ki = kwargs.get("ki", 0.0)
        self.kd = kwargs.get("kd", 0.0)
        self.kf = kwargs.get("kf", 0.0)
        self.i_zone = kwargs.get("i_zone", 0.0)
        self.i_max = kwargs.get("i_max", 0.0)
        self.epsilon = kwargs.get("epsilon", 1E-9)
        self.tolerance = kwargs.get("tolerance", 0.0)

        self.i_accum = kwargs.get("i_accum", 0.0)
        self.prev_error = kwargs.get("prev_error", 0.0)
    
    def reset(self):
        self.i_accum = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        if abs(error) < self.tolerance:
            return 0.0
        output = 0.0
        output += self.calculate_p(error)
        output += self.calculate_i(error)
        output += self.calculate_d(error, dt)
        output += self.calculate_f(setpoint)

        return output

    def calculate_p(self, error):
        if abs(self.kp) < self.epsilon:
            return 0.0
        return self.kp * error

    def calculate_i(self, error):
        if abs(self.ki) < self.epsilon:
            return 0.0
        if self.i_zone <= self.epsilon:
            self.i_accum = 0.0
        elif abs(error) < self.i_zone:
            self.i_accum += error
        
        if self.i_max > self.epsilon:
            if self.i_accum > 0.0:
                self.i_accum = min(self.i_accum, self.i_max / self.ki)
            else:
                self.i_accum = max(self.i_accum, -self.i_max / self.ki)

        return self.ki * self.i_accum

    def calculate_d(self, error, dt):
        if abs(self.kd) < self.epsilon:
            return 0.0
        if dt < 0.0:
            return 0.0
        output = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        return output

    def calculate_f(self, setpoint):
        if abs(self.kf) < self.epsilon:
            return 0.0
        return self.kf * setpoint

