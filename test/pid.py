from simple_pid import PID


class SMDSystemSim:
    def __init__(self, mass, k, c) -> None:
        self.mass = mass
        self.k = k
        self.c = c

    def get_acceleration(self, force, v0, x0):
        return (force - self.k*x0 - self.c * v0) / self.mass

    def get_velocity(self, a, v0, time_step):
        return a * time_step + v0

    def get_position(self, a, v0, x0, time_step):
        return a*time_step**2 + v0*time_step + x0


class PIDController:
    def __init__(self, kp=1, ki=1, kd=1, max_signal=1000) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target = 0
        self.signal = 0
        
        self.accumulator = 0
        self.last_reading = 0
        
        self.sampple_rate = 0.1
        self.max_signal = max_signal
        
    def adjust_signal(self, feedback_value):
        error = self.target - feedback_value
        self.accumulator += error
        self.signal = self.kp * error + self.ki * self.accumulator + self.kd * (feedback_value - self.last_reading) / self.sampple_rate

        if self.signal > self.max_signal:
            self.signal = self.max_signal
        elif self.signal < -self.max_signal:
            self.signal = -self.max_signal
            
        self.last_reading = feedback_value

    def run_simulation(self, system_simulator, duration, command):
        command_idx = 0
        
        x = [0]
        v = [0]
        
        if command[0][0] == 0:
            pass
        

pid = PID(1, 0.1, 0.05, setpoint=1)

if __name__ == "__main__":
    sys = SMDSystemSim()