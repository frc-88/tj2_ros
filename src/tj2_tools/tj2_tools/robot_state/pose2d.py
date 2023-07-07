from .robot_state import State


class Pose2d(State):
    def project_twist_forward(self, twist: Velocity, time_delta: float) -> "Pose2d":
        dx = twist.linear.x * time_delta
        dy = twist.linear.y * time_delta
        dtheta = twist.angular.z * time_delta

        if abs(dtheta) < 1e-9:
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta
            c = 0.5 * dtheta
        else:
            s = math.sin(dtheta) / dtheta
            c = (1.0 - math.cos(dtheta)) / dtheta

        tx = dx * s - dy * c
        ty = dx * c + dy * s

        forwarded: Pose2d = self.transform_by(Pose2d(tx, ty, dtheta))
        return forwarded
