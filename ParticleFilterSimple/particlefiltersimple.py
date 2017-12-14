#
# This module contains some basics about the Particle filter (based on the Udacity class by Sebastian Thrun)
#

from math import *
import random
import matplotlib.pyplot as plt
import plot_pf_robot

# tamanho do mapa
# min 80 max 200
world_size = 70.0
num_of_particles = 300 # qnto mais particulas mais rapida a localizacao
obstacles = [] 
num_obstacles = 5
steps = 30  # numero de movimentos do robo

obstacles = [[10.0, 10.0], [10.0, 35.0], [10.0, 60.0],
             [35.0, 10.0], [35.0, 35.0], [35.0, 60.0],
             [60.0, 10.0], [60.0, 35.0], [60.0, 60.0]]

# cria os obstaculos(fake) de forma randomica. Util para um mapa de tamanho 300 por ex
# while num_obstacles > 0:
#     obstacles.append([random.randint(1, world_size), random.randint(1, world_size)])
#     # print num_obstacles
#     num_obstacles = num_obstacles - 1

class RobotClass:
    def __init__(self):

        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi


    # Adiciona ruido ao movimento (Caso tirar o ruido o robot se encontra mais rapidamete)
    def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)


    # Gera a leitura da distancia dos obstaculos (Fake: Imita o laser do robot)
    def fake_laser(self):
        distArray = []

        for i in range(len(obstacles)):
            dist = sqrt((self.x - obstacles[i][0]) ** 2 + (self.y - obstacles[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            distArray.append(dist)

        return distArray


    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 3 * pi:
            raise ValueError('Orientation must be in [0..2pi]')

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)


    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cannot move backwards')

        # theta
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 3 * pi

        # x,y
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size
        y %= world_size

        # set particle
        res = RobotClass()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)

        return res


    @staticmethod
    def gaussian(mu, sigma, x):
        # Gaussian
        # Kalman and Bayes - livro
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


    def update(self, measurement):
        prob = 1.0 # max 3

        for i in range(len(obstacles)):
            dist = sqrt((self.x - obstacles[i][0]) ** 2 + (self.y - obstacles[i][1]) ** 2)
            prob *= self.gaussian(dist, self.sense_noise, measurement[i])
        return prob


    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def predict(particles, num_of_particles):
    p2 = []
    for i in range(num_of_particles):
        p2.append( particles[i].move(0.1, 2.) )
    return p2


# systematic_resample
def resample(weights, particles, num_of_particles):
    p3 = []

    index = int(random.random() * num_of_particles)
    c_sum = 0.0
    # pega a particula com maior peso (mais relevante)
    most_relevant_part = max(weights)
    # print most_relevant_part, ' most_relevant_part '
    # print weights

    # while i < N:
    #     if positions[i] < cumulative_sum[j]:
    #         indexes[i] = j
    #         i += 1
    #     else:
    #         j += 1

    for i in range(num_of_particles):
        c_sum += random.random() * most_relevant_part

        while c_sum > weights[index]:
            c_sum -= weights[index]
            index = (index + 1) % num_of_particles

        p3.append(particles[index])

    # print p3
    return p3  


def main():

    fake_robot = RobotClass()
    fake_robot = fake_robot.move(0.1, 2.)
    scan = fake_robot.fake_laser()

    particles = []

    for i in range(num_of_particles):
        robot = RobotClass()
        robot.set_noise(0.05, 0.08, 5.0)
        particles.append(robot)

    for t in range(steps):
        fake_robot = fake_robot.move(0.1, 2.)
        scan = fake_robot.fake_laser()

        # predict
        p2 = predict(particles, num_of_particles)
        particles = p2

        # da peso as particulas do filtro conforme a leitura do robot no mapa (Usando gaussiana)
        weights = []
        for i in range(num_of_particles):
            weights.append(particles[i].update(scan))

        # Resampling
        p3 = resample(weights, particles, num_of_particles)
        
        # particulas realocadas conforme resampling
        particles = p3
        # plota na imagem
        plot_pf_robot.visualization(fake_robot, t, p2, p3, weights, world_size, obstacles)

    print 'Array de particulas = ', particles # Mostra no console onde cada particula se encontra no mapa

if __name__ == "__main__":
    main()
