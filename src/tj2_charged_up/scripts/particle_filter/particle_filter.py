import warnings
import numpy as np
from numpy.random import randn, random, uniform, normal
import scipy.stats
from threading import Lock


G_CONST = 9.81


def predict(particles, input_std_error, num_particles, u, dt):
    """
    This is a static function so that it can be fed through numba's "jit" function (see jit_particle_filter.py)

    move according to control input u (velocity of robot and velocity of object)
    with noise std
    u[0, 1, 2] = linear_vx, linear_vy, angular_z
    """
    x_0 = particles[:, 0]
    y_0 = particles[:, 1]
    th_0 = particles[:, 2]

    vx_u = u[0]
    vy_u = u[1]
    vt_u = u[2]

    vx_sd_u = input_std_error[0]
    vy_sd_u = input_std_error[1]
    vt_sd_u = input_std_error[2]
    
    vx_a = vx_u * np.cos(th_0) - vy_u * np.sin(th_0)
    vy_a = vx_u * np.sin(th_0) + vy_u * np.cos(th_0)

    x_1 = x_0 + vx_a * dt + normal(0.0, vx_sd_u, num_particles)
    y_1 = y_0 + vy_a * dt + normal(0.0, vy_sd_u, num_particles)
    th_1 = th_0 + vt_u * dt + normal(0.0, vt_sd_u, num_particles)

    particles[:, 0] = x_1
    particles[:, 1] = y_1
    particles[:, 2] = th_1


class ParticleFilter:
    def __init__(self, num_particles, measure_std_error, input_std_error):
        self.lock = Lock()
        self.num_states = 3  # x, y, theta
        self.particles = np.zeros((num_particles, self.num_states))
        self.num_particles = num_particles
        self.measure_std_error = measure_std_error
        self.input_std_error = np.array(input_std_error)

        self.measure_distribution = scipy.stats.norm(0.0, self.measure_std_error)
        self.weights = np.array([])

        self.initialize_weights()

    def reset(self):
        with self.lock:
            self.particles = np.zeros((self.num_particles, self.num_states))
        self.initialize_weights()

    def set_parameters(self, num_particles, measure_std_error, input_std_error):
        with self.lock:
            self.num_particles = num_particles
            self.measure_std_error = measure_std_error
            self.measure_distribution = scipy.stats.norm(0.0, self.measure_std_error)
            self.input_std_error = np.array(input_std_error)
            
        self.reset()

    def initialize_particles(self, initial_distribution_type, initial_range, initial_state=None):
        if initial_state is None:
            initial_state = np.zeros((self.num_states, 1), dtype=np.float64)
        if initial_distribution_type == "gaussian":
            self.create_gaussian_particles(initial_state, initial_range)
        elif initial_distribution_type == "uniform":
            self.create_uniform_particles(initial_state, initial_range)
        else:
            warnings.warn(f"Invalid distribution type: {initial_distribution_type}. Defaulting to gaussian.")
            self.create_gaussian_particles(initial_state, initial_range)

    def is_initialized(self):
        with self.lock:
            return not np.all(self.particles == 0.0)

    def initialize_weights(self):
        # initialize with uniform weight
        with self.lock:
            self.weights = np.ones(self.num_particles)
            self.weights /= np.sum(self.weights)

    def create_uniform_particles(self, initial_state, state_range):
        assert len(initial_state) == self.num_states
        assert len(state_range) == self.num_states

        self.initialize_weights()
        with self.lock:
            for state_num in range(self.num_states):
                min_val = initial_state[state_num] - state_range[state_num]
                max_val = initial_state[state_num] + state_range[state_num]
                self.particles[:, state_num] = uniform(min_val, max_val, size=self.num_particles)

    def create_gaussian_particles(self, mean, var):
        self.initialize_weights()
        with self.lock:
            for state_num in range(self.num_states):
                self.particles[:, state_num] = mean[state_num] + randn(self.num_particles) * var[state_num]

    def predict(self, u, dt):
        """
        move according to control input u (velocity of robot and velocity of object)
        with noise std
        u[0, 1, 2] = linear_vx, linear_vy, angular_z
        """
        with self.lock:
            predict(self.particles, self.input_std_error, self.num_particles, u, dt)

    def update(self, z):
        """Update particle filter according to measurement z (robot global position from tag: [x, y, theta])"""
        self.weights.fill(1.0)  # debateable as to whether this is detrimental or not (shouldn't weights be preserved between measurements?)
        with self.lock:
            distances = np.linalg.norm(self.particles - z, axis=1)
            
            self.weights *= self.measure_distribution.pdf(distances)

            self.weights += 1.e-300  # avoid divide by zero error
            self.weights /= np.sum(self.weights)  # normalize

    def neff(self):
        with self.lock:
            return 1.0 / np.sum(np.square(self.weights))

    def resample(self):
        # indices = self.simple_resample()
        indices = self.systematic_resample()

        # resample according to indices
        self.particles = self.particles[indices]
        self.weights = self.weights[indices]
        self.weights /= np.sum(self.weights)  # normalize

    def resample_from_index(self, indices):
        assert len(indices) == self.num_particles

        with self.lock:
            self.particles = self.particles[indices]
            self.weights = self.weights[indices]
            self.weights /= np.sum(self.weights)

    def estimate(self):
        """ returns mean and variance """
        mu = self.mean()
        with self.lock:
            var = np.average((self.particles - mu) ** 2, weights=self.weights, axis=0)

        return mu, var

    def mean(self):
        """ returns weighted mean position"""
        with self.lock:
            return np.average(self.particles, weights=self.weights, axis=0)

    def check_resample(self):
        neff = self.neff()
        with self.lock:
            if neff < self.num_particles / 2.0:
                self.resample()
                return True
            else:
                return False

    def simple_resample(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # avoid round-off error
        indices = np.searchsorted(cumulative_sum, random(self.num_particles))
        return indices

    def systematic_resample(self):
        cumulative_sum = np.cumsum(self.weights)
        indices = np.zeros(self.num_particles, 'int')
        t = np.linspace(0, 1.0 - 1.0 / self.num_particles, self.num_particles) + random() / self.num_particles

        i, j = 0, 0
        while i < self.num_particles and j < self.num_particles:
            while cumulative_sum[j] < t[i]:
                j += 1
            indices[i] = j
            i += 1

        return indices
