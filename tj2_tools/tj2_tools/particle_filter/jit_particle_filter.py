import time
import numpy as np
from numpy.random import randn, random, uniform
import scipy.stats

from numba import njit

from .particle_filter import ParticleFilter, predict


jit_predict = njit(predict)


@njit
def jit_update(particles, z, num_particles):
    # weight according to how far away the particle is from the measurement in x, y, z
    diff = particles - z
    distances = np.zeros(num_particles)

    for index in range(num_particles):
        distances[index] = np.linalg.norm(diff[index])
    return distances


@njit
def jit_normalize_weights(weights):
    weights += 1.e-300  # avoid divide by zero error
    weights /= np.sum(weights)  # normalize
    return weights


@njit
def jit_systematic_resample(weights, num_particles):
    cumulative_sum = np.cumsum(weights)
    indices = np.zeros(num_particles, np.int32)
    t = np.linspace(0, 1.0 - 1.0 / num_particles, num_particles) + random() / num_particles

    i, j = 0, 0
    while i < num_particles and j < num_particles:
        while cumulative_sum[j] < t[i]:
            j += 1
        indices[i] = j
        i += 1

    return indices


@njit
def jit_resample(particles, weights, num_particles):
    indices = jit_systematic_resample(weights, num_particles)

    # resample according to indices
    particles = particles[indices]
    weights = weights[indices]
    weights /= np.sum(weights)  # normalize
    return particles, weights


class JitParticleFilter(ParticleFilter):
    def __init__(self, serial, num_particles, measure_std_error, input_std_error, stale_filter_time):
        super(JitParticleFilter, self).__init__(serial, num_particles, measure_std_error, input_std_error,
                                                stale_filter_time)

    def predict(self, u, dt):
        jit_predict(self.particles, self.input_std_error, self.num_particles, u, dt)

    def update(self, z, is_stale=False):
        # self.weights.fill(1.0)
        distances = jit_update(self.particles, z, self.num_particles)
        self.weights *= self.measure_distribution.pdf(distances)
        self.weights = jit_normalize_weights(self.weights)
        self.last_measurement_time = time.time()

    def resample(self):
        self.particles, self.weights = jit_resample(self.particles, self.weights, self.num_particles)
