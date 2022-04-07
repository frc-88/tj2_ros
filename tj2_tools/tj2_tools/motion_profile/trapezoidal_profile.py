import math
from .pid_controller import PIDController

class State:
    def __init__(self, position=0.0, velocity=0.0):
        self.position = position
        self.velocity = velocity

    @classmethod
    def from_state(cls, other):
        if not isinstance(other, cls):
            raise ValueError("%s is not of type %s" % (repr(other), cls))
        self = cls()
        self.position = other.position
        self.velocity = other.velocity
        return self
    
    def __repr__(self) -> str:
        return "%s(position=%s, velocity=%s)" % (self.__class__.__name__, self.position, self.velocity)

    __str__ = __repr__


class TrapezoidalProfile:
    def __init__(self, pid_config, trapezoid_config):
        self.pid = PIDController(**pid_config)
        self.max_speed = trapezoid_config.get("max_speed", 1.0)
        self.max_accel = trapezoid_config.get("max_accel", 1.0)

        self.initial = State()
        self.goal = State()

        self.direction = 1.0
        self.end_accel = 0.0
        self.end_full_speed = 0.0
        self.end_decel = 0.0

        self.prev_t = 0.0

        self.reset(State(), State())
    
    def reset_position(self, goal_position, initial_position=0.0):
        self.reset(State(goal_position), State(initial_position))

    def reset(self, goal: State, initial: State = None):
        if initial is None:
            initial = State()
        self.prev_t = 0.0
        self.direction = -1.0 if self.should_flip_accel(initial, goal) else 1.0
        self.initial = self.direct(initial)
        self.goal = self.direct(goal)

        if self.initial.velocity > self.max_speed:
            self.initial.velocity = self.max_speed

        # Deal with a possibly truncated motion profile (with nonzero initial or
        # final velocity) by calculating the parameters as if the profile began and
        # ended at zero velocity
        cutoff_begin = self.initial.velocity / self.max_accel
        cutoff_dist_begin = cutoff_begin * cutoff_begin * self.max_accel / 2.0

        cutoff_end = self.goal.velocity / self.max_accel
        cutoff_dist_end = cutoff_end * cutoff_end * self.max_accel / 2.0

        # Now we can calculate the parameters as if it was a full trapezoid instead
        # of a truncated one

        full_trapezoid_dist = \
            cutoff_dist_begin + (self.goal.position - self.initial.position) + cutoff_dist_end
        acceleration_time = self.max_speed / self.max_accel

        full_speed_dist = \
            full_trapezoid_dist - acceleration_time * acceleration_time * self.max_accel

        # Handle the case where the profile never reaches full speed
        if full_speed_dist < 0:
            acceleration_time = math.sqrt(full_trapezoid_dist / self.max_accel)
            full_speed_dist = 0.0

        self.end_accel = acceleration_time - cutoff_begin
        self.end_full_speed = self.end_accel + full_speed_dist / self.max_speed
        self.end_decel = self.end_full_speed + acceleration_time - cutoff_end

    def calculate(self, t):
        """
        Calculate the correct position and velocity for the profile at a time t where the beginning of
        the profile was at time t = 0.
        """
        result = State.from_state(self.initial)

        if t < self.end_accel:
            result.velocity += t * self.max_accel
            result.position += (self.initial.velocity + t * self.max_accel / 2.0) * t
        elif t < self.end_full_speed:
            result.velocity = self.max_speed
            result.position += \
                (self.initial.velocity + self.end_accel * self.max_accel / 2.0) * self.end_accel \
                    + self.max_speed * (t - self.end_accel)
        elif t <= self.end_decel:
            result.velocity = self.goal.velocity + (self.end_decel - t) * self.max_accel
            time_left = self.end_decel - t
            result.position = \
                self.goal.position \
                    - (self.goal.velocity + time_left * self.max_accel / 2.0) * time_left
        else:
            result = self.goal

        return self.direct(result)
    
    def calculate_command_velocity(self, current_position, t):
        dt = t - self.prev_t
        self.prev_t = t
        return self.pid.update(self.calculate(t).position, current_position, dt)

    def time_left_until(self, target: float):
        """
        Returns the time left until a target distance in the profile is reached.
        """
        position = self.initial.position * self.direction
        velocity = self.initial.velocity * self.direction

        end_accel = self.end_accel * self.direction
        end_full_speed = self.end_full_speed * self.direction - end_accel

        if target < position:
            end_accel = -end_accel
            end_full_speed = -end_full_speed
            velocity = -velocity

        end_accel = max(end_accel, 0.0)
        end_full_speed = max(end_full_speed, 0.0)

        acceleration = self.max_accel
        deceleration = -self.max_accel

        dist_to_target = abs(target - position)
        if dist_to_target < 1e-6:
            return 0.0

        accel_dist = velocity * end_accel + 0.5 * acceleration * end_accel * end_accel

        if end_accel > 0:
            decel_velocity = math.sqrt(abs(velocity * velocity + 2 * acceleration * accel_dist))
        else:
            decel_velocity = velocity

        full_speed_dist = self.max_speed * end_full_speed
        decel_dist = 0.0

        if accel_dist > dist_to_target:
            accel_dist = dist_to_target
            full_speed_dist = 0.0
            decel_dist = 0.0
        elif accel_dist + full_speed_dist > dist_to_target:
            full_speed_dist = dist_to_target - accel_dist
            decel_dist = 0
        else:
            decel_dist = dist_to_target - full_speed_dist - accel_dist

        accel_time = \
            (-velocity + math.sqrt(abs(velocity * velocity + 2 * acceleration * accel_dist))) \
                / acceleration

        decel_time = \
            (-decel_velocity \
                    + math.sqrt( \
                        abs(decel_velocity * decel_velocity + 2 * deceleration * decel_dist))) \
                / deceleration

        full_speed_time = full_speed_dist / self.max_speed

        return accel_time + full_speed_time + decel_time
        
    def total_time(self):
        """
        Returns the total time the profile takes to reach the goal.
        """
        return self.end_decel

    def is_finished(self, t):
        """
        Returns true if the profile has reached the goal.
        
        The profile has reached the goal if the time since the profile started has exceeded the
        profile's total time.
        """
        return t >= self.total_time()
    
    def should_flip_accel(self, initial: State, goal: State):
        """
        Returns true if the profile inverted.
        The profile is inverted if goal position is less than the initial position.
        """
        return initial.position > goal.position
        

    def direct(self, state: State):
        """
        Flip the sign of the velocity and position if the profile is inverted
        """
        result = State.from_state(state)
        result.position = result.position * self.direction
        result.velocity = result.velocity * self.direction
        return result
