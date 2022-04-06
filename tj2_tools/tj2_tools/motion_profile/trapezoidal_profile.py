import math
from .pid_controller import PIDController


class TrapezoidalProfile:
    def __init__(self, pid_config, trapezoid_config):
        self.pid = PIDController(**pid_config)
        self.max_speed = trapezoid_config.get("max_speed", 1.0)
        self.max_accel = trapezoid_config.get("max_accel", 1.0)
        self.reset_time_threshold = trapezoid_config.get("reset_time_threshold", 0.2)
        self.position_jump_threshold = self.max_speed / trapezoid_config.get("position_jump_threshold", 20.0)

        self.last_command_pos = 0.0
        self.last_command_vel = 0.0
    
    def set_target_velocity(self, velocity):
        self.target_velocity = velocity
    
    def set_target_position(self, position):
        self.target_position = position

    def reset(self, current_position):
        """
        Resets the trapezoidal controller, including the associated position PID. Should be called
        after this controller hasn't been used for a long time.
        """
        self.last_command_pos = current_position
        self.pid.reset()

    def calculate_velocity(self, current_position, dt):
        """
        Calculates the velocity output following the trapezoidal profile.
        """
        # Make sure the elapsed time and difference in commanded position is reasonable
        if dt > self.reset_time_threshold or abs(self.last_command_pos - current_position) > self.position_jump_threshold:
            self.last_command_pos = current_position
        
        # Determine if the position target is ahead of or behind the current position
        forwards = self.target_position > self.last_command_pos

        # Calculate the command velocity based on the current velocity and max acceleration
        command_vel = self.calculate_accelerated_vel(self.last_command_vel, forwards, dt)

        # Limit the command velocity based on the max speed
        command_vel = self.apply_max_velocity_limit(command_vel)

        # Limit the command velocity such that it has time to deccelerate to hit the target at the right velocity
        command_vel = self.apply_decceleration_limit(command_vel, forwards)

        self.last_command_vel = command_vel

        # Determine the position to command
        command_pos = self.calculate_command_position(command_vel, dt)

        # Apply the position PID
        command_vel += self.pid.update(current_position, command_pos, dt)

        self.last_command_pos = command_pos

        return command_vel

    def calculate_accelerated_vel(self, current_vel, forwards, dt):
        """
        Calculates the command velocity based on the current velocity and max acceleration.
        """
        added_vel = math.copysign(dt * self.max_accel, 1.0 if forwards else -1.0)
        return current_vel + added_vel

    def apply_max_velocity_limit(self, current_vel):
        """
        Limits the command velocity such that it does not exceed the max speed limit.
        """
        return min(self.max_speed, max(-self.max_speed, current_vel))

    def apply_decceleration_limit(self, current_vel, forwards):
        """
        Limit the command velocity such that it has time to deccelerate to hit the target at the right velocity.
        """
        # v^2 = v0^2 + 2ax -> v0 = sqrt(v^2 - 2ax)
        limit_vel = math.copysign(self.target_velocity ** 2.0, self.target_velocity) + 2 * self.max_accel * (self.target_position - self.last_command_pos)
        limit_vel = math.copysign(math.sqrt(abs(limit_vel)), limit_vel)
        if forwards:
            return min(limit_vel, current_vel)
        else:
            return max(limit_vel, current_vel)

    def calculate_command_position(self, current_vel, dt):
        """
        Determine the command position based on the last command position and the velocity.
        """
        return self.last_command_vel + dt * current_vel
