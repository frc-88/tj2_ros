import numpy as np

class SwerveState:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vt = 0.0

        self.x = 0.0
        self.y = 0.0
        self.t = 0.0

    def __str__(self):
        return f"SwerveState(vx={self.vx}, vy={self.vy}, vt={self.vt}, x={self.x}, y={self.y}, t={self.t})"

    __repr__ = __str__


class SwerveKinematics:
    def __init__(self, module_positions):
        self.module_positions = module_positions

        self.state = SwerveState()

        # Matrix format:
        # [
        #     [1, 0, -module_n0_y],
        #     [0, 1, module_n0_x],
        #     [1, 0, -module_n1_y],
        #     [0, 1, module_n1_x],
        #     ...
        # ]
        self.inverse_kinematics = []
        for row in module_positions:
            x, y = row
            self.inverse_kinematics.append([1.0, 0.0, -y])
            self.inverse_kinematics.append([0.0, 1.0, x])
        self.inverse_kinematics = np.array(self.inverse_kinematics)
        self.forward_kinematics = np.linalg.pinv(self.inverse_kinematics)
        print(self.inverse_kinematics.tolist())
        print(self.forward_kinematics.tolist())

    def module_to_chassis_speeds(self, module_speeds):
        module_states_matrix = []
        for row in module_speeds:
            azimuth, wheel_vel = row
            vx = wheel_vel * np.cos(azimuth)
            vy = wheel_vel * np.sin(azimuth)

            module_states_matrix.append(vx)
            module_states_matrix.append(vy)

        #     print(vx, vy)
        # print()

        module_states_matrix = np.array(module_states_matrix)
        chassis_vector = np.dot(self.forward_kinematics, module_states_matrix)
        self.state.vx = chassis_vector[0]
        self.state.vy = chassis_vector[1]
        self.state.vt = chassis_vector[2]

        return self.state

    def estimate_pose(self, dt):
        if dt <= 0.0:
            return self.state

        dx = self.state.vx * dt
        dy = self.state.vy * dt
        dtheta = self.state.vt * dt

        sin_theta = np.sin(dtheta)
        cos_theta = np.cos(dtheta)

        if abs(dtheta) < 1e-9:
            # Transformation from twist to pose can be indeterminant when angular velocity is zero.
            # Use taylor series approximation to mitigate this problem.
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta
            c1 = 0.5 * dtheta
            c2 = -0.5 * dtheta
        else:
            s = sin_theta / dtheta
            c1 = (1 - cos_theta) / dtheta
            c2 = (cos_theta - 1) / dtheta

        # Matrix format:
        # [
        #     [cos(th), -sin(th), 0],
        #     [sin(th), cos(th), 0],
        #     [0, 0, 1]
        # ]
        pose_rotation_matrix = np.array([
            [cos_theta, -sin_theta, 0.0],
            [sin_theta, cos_theta, 0.0],
            [0.0, 0.0, 1.0]
        ])

        # Matrix format:
        # [
        #     [sin(dth)/dth, (cos(dth) - 1)/dth, 0],
        #     [(1 - cos(dth))/dth, sin(dth)/dth, 0],
        #     [0, 0, 1]
        # ]
        pose_translation_matrix = np.array([
            [s, c1, 0.0],
            [c2, s, 0.0],
            [0.0, 0.0, 1.0]
        ])

        pose_vector = np.array([dx, dy, dtheta])

        delta_pose_vector = np.dot(pose_translation_matrix, np.dot(pose_rotation_matrix, pose_vector))
        # delta_pose_vector = np.dot(pose_rotation_matrix, np.dot(pose_translation_matrix, pose_vector))

        self.state.x += delta_pose_vector[0]
        self.state.y += delta_pose_vector[1]
        self.state.t += delta_pose_vector[2]

        return self.state
