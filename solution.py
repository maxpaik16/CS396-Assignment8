
import numpy as np
import pyrosim.pyrosim as pyrosim
import os

import time
import random
import constants as c


class SOLUTION:

    def __init__(self, ID):
        self.myID = ID
        self.num_joints = random.randint(3, 10)
        self.links_with_sensors = [1]
        for i in range(2, self.num_joints + 2):
            if random.random() > .5:
                self.links_with_sensors.append(i)

        #print(self.links_with_sensors)

        self.weights = np.random.rand(len(self.links_with_sensors), self.num_joints)
        self.weights *= 2
        self.weights -= 1

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Robot()
        self.Create_Brain()

        with open("sensors{}.txt".format(self.myID), 'w') as f:
            for n in self.links_with_sensors:
                f.write(str(n))

        os.system('python simulate.py {} {} 2&>1 &'.format(directOrGUI, self.myID))

        #os.system('python simulate.py {} {}'.format(directOrGUI, self.myID))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists('fitness{}.txt'.format(self.myID)):
            time.sleep(.01)
        with open('fitness{}.txt'.format(self.myID), 'r') as f:
            self.fitness = float(f.read())
        # print(self.fitness)
        os.system('rm {}'.format('fitness{}.txt'.format(self.myID)))
        os.system('rm {}'.format('sensors{}.txt'.format(self.myID)))

    def Evaluate(self, directOrGUI):
        self.Create_World()
        self.Create_Robot()
        self.Create_Brain()
        os.system('python simulate.py {} {} &'.format(directOrGUI, self.myID))

    def Create_World(self):
        pyrosim.Start_SDF('world.sdf')

        pyrosim.End()

        while not os.path.exists('world.sdf'):
            time.sleep(.01)

    def Create_Robot(self):
        pyrosim.Start_URDF("body.urdf")

        lastsize = [.5 * random.random(), .5 * random.random(), .5 * random.random()]
        lastpos = [0, 0, 1 + lastsize[2] / 2]
        pyrosim.Send_Cube(
            name="Link1",
            pos=lastpos,
            size=lastsize,
            color=1 in self.links_with_sensors
        )

        pyrosim.Send_Joint(
            name="Link1_Link2",
            parent="Link1",
            child="Link2",
            type="revolute",
            position=[lastsize[0] / 2, 0, 1 + lastsize[2]],
            jointAxis="0 1 0"
        )

        newsize = [.5 * random.random(), .5 * random.random(), .5 * random.random()]
        pyrosim.Send_Cube(
            name="Link2",
            pos=[newsize[0]/2, 0, -newsize[2]/2],
            size=newsize,
            color=2 in self.links_with_sensors
        )

        lastsize = newsize
        center_relative = [lastsize[0] / 2, 0, -lastsize[2] / 2]

        for i in range(2, self.num_joints + 1):

            direction = random.randint(0, 7)
            newsize = [.5 * random.random(), .5 * random.random(), .5 * random.random()]

            if direction == 0:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0] + lastsize[0] / 2, center_relative[1],
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="0 1 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[newsize[0] / 2, 0,
                         -newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [newsize[0] / 2, 0, -newsize[2] / 2]

            elif direction == 1:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0] + lastsize[0] / 2, center_relative[1],
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="0 1 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[newsize[0] / 2, 0,
                         newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [newsize[0] / 2, 0, newsize[2] / 2]

            elif direction == 2:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0], center_relative[1] + lastsize[1] / 2,
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="1 0 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[0, newsize[1] / 2,
                         -newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [0, newsize[1] / 2, -newsize[2] / 2]

            elif direction == 3:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0], center_relative[1] + lastsize[1] / 2,
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="1 0 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[0, newsize[1] / 2,
                         newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [0, newsize[1] / 2, newsize[2] / 2]

            if direction == 4:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0] - lastsize[0] / 2, center_relative[1],
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="0 1 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[-newsize[0] / 2, 0,
                         -newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [-newsize[0] / 2, 0, -newsize[2] / 2]

            elif direction == 5:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0] - lastsize[0] / 2, center_relative[1],
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="0 1 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[-newsize[0] / 2, 0,
                         newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [-newsize[0] / 2, 0, newsize[2] / 2]

            elif direction == 6:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0], center_relative[1] - lastsize[1] / 2,
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="1 0 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[0, -newsize[1] / 2,
                         -newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [0, -newsize[1] / 2, -newsize[2] / 2]

            elif direction == 7:

                pyrosim.Send_Joint(
                    name="Link{}_Link{}".format(i, i + 1),
                    parent="Link{}".format(i),
                    child="Link{}".format(i + 1),
                    type="revolute",
                    position=[center_relative[0], center_relative[1] - lastsize[1] / 2,
                              center_relative[2] + lastsize[2] / 2],
                    jointAxis="1 0 0"
                )

                pyrosim.Send_Cube(
                    name="Link{}".format(i + 1),
                    pos=[0, -newsize[1] / 2,
                         newsize[2] / 2],
                    size=newsize,
                    color=i + 1 in self.links_with_sensors
                )

                center_relative = [0, -newsize[1] / 2, newsize[2] / 2]

            lastsize = newsize

        pyrosim.End()

        while not os.path.exists('body.urdf'):
            time.sleep(.01)

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork('brain{}.nndf'.format(self.myID))

        num_sensors = 0
        for i in self.links_with_sensors:
            pyrosim.Send_Sensor_Neuron(name=num_sensors, linkName='Link{}'.format(i))
            num_sensors += 1

        for i in range(1, self.num_joints+1):
            pyrosim.Send_Motor_Neuron(name=i + num_sensors, jointName='Link{}_Link{}'.format(i, i+1))

        for i in range(num_sensors):
            for j in range(self.num_joints):
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + num_sensors, weight=self.weights[i, j])

        pyrosim.End()

        while not os.path.exists('brain{}.nndf'.format(self.myID)):
            time.sleep(.01)

    def Mutate(self):
        row = random.randint(0, len(self.links_with_sensors) - 1)
        column = random.randint(0, self.num_joints - 1)

        self.weights[row, column] = 2 * random.random() - 1

    def Set_ID(self, val):
        self.myID = val

