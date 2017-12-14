from math import *
import matplotlib.pyplot as plt

def visualization(robot, step, p, pr, weights, world_size, obstacles):
    step = step + 1
    print step
    plt.figure("Mapa do Robo", figsize=(10., 10.))
    plt.title('Movimento do Robo ' + str(step))

    # draw coordinate grid for plotting
    grid = [0, world_size, 0, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size), 5)])
    plt.yticks([i for i in range(0, int(world_size), 5)])

    # draw particles
    for ind in range(len(p)):

        # particle
        circle = plt.Circle((p[ind].x, p[ind].y), 1., facecolor='#e63900', edgecolor='#e63900', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(p[ind].x, p[ind].y, 2*cos(p[ind].orientation), 2*sin(p[ind].orientation), alpha=1., facecolor='#ff9900', edgecolor='#ff9900')
        plt.gca().add_patch(arrow)

    for ind in range(len(pr)):

        # particle
        circle = plt.Circle((pr[ind].x, pr[ind].y), 1., facecolor='#0033cc', edgecolor='#0033cc', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(pr[ind].x, pr[ind].y, 2*cos(pr[ind].orientation), 2*sin(pr[ind].orientation), alpha=2., facecolor='#ffffff', edgecolor='#0033cc')
        plt.gca().add_patch(arrow)

    # fixed obstacles of known locations
    for lm in obstacles:
        circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#1a0500', edgecolor='#000000')
        plt.gca().add_patch(circle)

    # robot's location
    circle = plt.Circle((robot.x, robot.y), 2., facecolor='#ccff33', edgecolor='#ffffff')
    plt.gca().add_patch(circle)

    # robot's orientation
    arrow = plt.Arrow(robot.x, robot.y, 2*cos(robot.orientation), 2*sin(robot.orientation), alpha=0.8, facecolor='#ffffff', edgecolor='#000000')
    plt.gca().add_patch(arrow)

    plt.savefig("output/" + str(step) + ".png")
    plt.close() 