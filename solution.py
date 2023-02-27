
import numpy as np
import pyrosim.pyrosim as pyrosim
import os

import time
import random
import constants as c


class SOLUTION:

    def __init__(self, ID):
        self.myID = ID
        #self.num_joints = np.random.randint(3, )
        self.num_joints = 4
        self.links_with_sensors = [1]
        for i in range(2, self.num_joints + 2):
            if np.random.random() > .5:
                self.links_with_sensors.append(i)

        self.link_sizes = [[np.random.random(), np.random.random(), np.random.random()] for i in range(self.num_joints + 1)]
        self.link_posns = []
        self.directions = []
        self.joint_posns = []
        self.link_to_centers = []
        self.joint_axes = []
        self.Create_Robot_Data()

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

    def Create_Robot_Data(self):

        lastpos = [0, 0, 1 + self.link_sizes[0][2] / 2]
        self.link_posns.append(lastpos)
        self.joint_posns.append([self.link_sizes[0][0] / 2, 0, 1+self.link_sizes[0][2]])
        self.joint_axes.append("0 1 0")
        self.directions.append(0)
        self.link_to_centers.append(lastpos)

        self.link_posns.append([self.link_sizes[1][0]/2, 0, -self.link_sizes[1][2]/2])
        center_relative = [self.link_sizes[1][0] / 2, 0, -self.link_sizes[1][2]/2]
        self.link_to_centers.append(center_relative)

        for i in range(2, self.num_joints + 1):

            direction = np.random.randint(0, 8)
            lastsize = self.link_sizes[i - 1]
            newsize = self.link_sizes[i]

            while np.abs(direction - self.directions[-1]) == 4:
                direction = np.random.randint(0, 8)

            self.directions.append(direction)
            if direction == 0:

                self.joint_posns.append([center_relative[0] + lastsize[0] / 2, center_relative[1],
                              center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("0 1 0")
                self.link_posns.append([newsize[0] / 2, 0, -newsize[2] / 2])

                center_relative = [newsize[0] / 2, 0, -newsize[2] / 2]
                self.link_to_centers.append(center_relative)

            elif direction == 1:

                self.joint_posns.append([center_relative[0] + lastsize[0] / 2, center_relative[1],
                              center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("0 1 0")
                self.link_posns.append([newsize[0] / 2, 0,
                         newsize[2] / 2])

                center_relative = [newsize[0] / 2, 0, newsize[2] / 2]
                self.link_to_centers.append(center_relative)

            elif direction == 2:

                self.joint_posns.append([center_relative[0], center_relative[1] + lastsize[1] / 2,
                              center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("1 0 0")
                self.link_posns.append([0, newsize[1] / 2,
                         -newsize[2] / 2])

                center_relative = [0, newsize[1] / 2, -newsize[2] / 2]
                self.link_to_centers.append(center_relative)

            elif direction == 3:

                self.joint_posns.append([center_relative[0], center_relative[1] + lastsize[1] / 2,
                                         center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("1 0 0")
                self.link_posns.append([0, newsize[1] / 2,
                                        newsize[2] / 2])

                center_relative = [0, newsize[1] / 2, newsize[2] / 2]
                self.link_to_centers.append(center_relative)

            if direction == 4:

                self.joint_posns.append([center_relative[0] - lastsize[0] / 2, center_relative[1],
                                         center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("0 1 0")
                self.link_posns.append([-newsize[0] / 2, 0,
                                        -newsize[2] / 2])

                center_relative = [-newsize[0] / 2, 0, -newsize[2] / 2]
                self.link_to_centers.append(center_relative)

            elif direction == 5:

                self.joint_posns.append([center_relative[0] - lastsize[0] / 2, center_relative[1],
                                         center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("0 1 0")
                self.link_posns.append([-newsize[0] / 2, 0,
                                        newsize[2] / 2])

                center_relative = [-newsize[0] / 2, 0, newsize[2] / 2]
                self.link_to_centers.append(center_relative)

            elif direction == 6:

                self.joint_posns.append([center_relative[0], center_relative[1] - lastsize[1] / 2,
                                         center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("1 0 0")
                self.link_posns.append([0, -newsize[1] / 2,
                                        -newsize[2] / 2])

                center_relative = [0, -newsize[1] / 2, -newsize[2] / 2]
                self.link_to_centers.append(center_relative)

            elif direction == 7:

                self.joint_posns.append([center_relative[0], center_relative[1] - lastsize[1] / 2,
                                         center_relative[2] + lastsize[2] / 2])
                self.joint_axes.append("1 0 0")
                self.link_posns.append([0, -newsize[1] / 2,
                                        newsize[2] / 2])

                center_relative = [0, -newsize[1] / 2, newsize[2] / 2]
                self.link_to_centers.append(center_relative)

    def Create_Robot(self):
        pyrosim.Start_URDF("body{}.urdf".format(self.myID))

        pyrosim.Send_Cube(
            name="Link1",
            pos=self.link_posns[0],
            size=self.link_sizes[0],
            color=1 in self.links_with_sensors
        )

        for i in range(self.num_joints):
            pyrosim.Send_Joint(
                name="Link{}_Link{}".format(i + 1, i + 2),
                parent="Link{}".format(i + 1),
                child="Link{}".format(i + 2),
                type="revolute",
                position=self.joint_posns[i],
                jointAxis=self.joint_axes[i]
            )

            pyrosim.Send_Cube(
                name="Link{}".format(i + 2),
                pos=self.link_posns[i + 1],
                size=self.link_sizes[i + 1],
                color=i + 2 in self.links_with_sensors
            )

        pyrosim.End()

        while not os.path.exists('body{}.urdf'.format(self.myID)):
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
                #print(self.weights.shape)
                #print(self.num_joints)
                #print(num_sensors)
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + num_sensors, weight=self.weights[i, j])

        pyrosim.End()

        while not os.path.exists('brain{}.nndf'.format(self.myID)):
            time.sleep(.01)

    def Mutate(self):
        which = np.random.randint(0, 2)
        #which = 0

        if which == 0:
            row = np.random.randint(0, len(self.links_with_sensors))
            column = np.random.randint(0, self.num_joints)

            self.weights[row, column] = 2 * np.random.random() - 1

        else:
            which = np.random.randint(0, 2)

            if which == 0:
                if self.num_joints <= 1:
                    row = np.random.randint(0, len(self.links_with_sensors))
                    column = np.random.randint(0, self.num_joints)

                    self.weights[row, column] = 2 * np.random.random() - 1
                    return
                self.joint_posns.pop()
                self.link_posns.pop()
                self.link_sizes.pop()
                self.link_to_centers.pop()
                self.joint_axes.pop()
                self.directions.pop()
                if self.num_joints + 1 in self.links_with_sensors:
                    self.links_with_sensors.remove(self.num_joints + 1)

                self.num_joints -= 1
                old_weights = self.weights
                self.weights = np.zeros((len(self.links_with_sensors), self.num_joints))
                self.weights = old_weights[:self.weights.shape[0], :self.weights.shape[1]]

            else:
                if self.num_joints >= 4:
                    row = np.random.randint(0, len(self.links_with_sensors))
                    column = np.random.randint(0, self.num_joints)

                    self.weights[row, column] = 2 * np.random.random() - 1
                    return
                newsize = [np.random.random(), np.random.random(), np.random.random()]
                direction = np.random.randint(0, 8)
                lastsize = self.link_sizes[-1]
                self.link_sizes.append(newsize)
                self.num_joints += 1

                if np.random.random() > .5:
                    self.links_with_sensors.append(self.num_joints + 1)

                while np.abs(direction - self.directions[-1]) == 4:
                    direction = np.random.randint(0, 8)

                self.directions.append(direction)
                center_relative = self.link_to_centers[-1]

                if direction == 0:

                    self.joint_posns.append([center_relative[0] + lastsize[0] / 2, center_relative[1],
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("0 1 0")
                    self.link_posns.append([newsize[0] / 2, 0, -newsize[2] / 2])

                    center_relative = [newsize[0] / 2, 0, -newsize[2] / 2]
                    self.link_to_centers.append(center_relative)

                elif direction == 1:

                    self.joint_posns.append([center_relative[0] + lastsize[0] / 2, center_relative[1],
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("0 1 0")
                    self.link_posns.append([newsize[0] / 2, 0,
                                            newsize[2] / 2])

                    center_relative = [newsize[0] / 2, 0, newsize[2] / 2]
                    self.link_to_centers.append(center_relative)

                elif direction == 2:

                    self.joint_posns.append([center_relative[0], center_relative[1] + lastsize[1] / 2,
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("1 0 0")
                    self.link_posns.append([0, newsize[1] / 2,
                                            -newsize[2] / 2])

                    center_relative = [0, newsize[1] / 2, -newsize[2] / 2]
                    self.link_to_centers.append(center_relative)

                elif direction == 3:

                    self.joint_posns.append([center_relative[0], center_relative[1] + lastsize[1] / 2,
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("1 0 0")
                    self.link_posns.append([0, newsize[1] / 2,
                                            newsize[2] / 2])

                    center_relative = [0, newsize[1] / 2, newsize[2] / 2]
                    self.link_to_centers.append(center_relative)

                if direction == 4:

                    self.joint_posns.append([center_relative[0] - lastsize[0] / 2, center_relative[1],
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("0 1 0")
                    self.link_posns.append([-newsize[0] / 2, 0,
                                            -newsize[2] / 2])

                    center_relative = [-newsize[0] / 2, 0, -newsize[2] / 2]
                    self.link_to_centers.append(center_relative)

                elif direction == 5:

                    self.joint_posns.append([center_relative[0] - lastsize[0] / 2, center_relative[1],
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("0 1 0")
                    self.link_posns.append([-newsize[0] / 2, 0,
                                            newsize[2] / 2])

                    center_relative = [-newsize[0] / 2, 0, newsize[2] / 2]
                    self.link_to_centers.append(center_relative)

                elif direction == 6:

                    self.joint_posns.append([center_relative[0], center_relative[1] - lastsize[1] / 2,
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("1 0 0")
                    self.link_posns.append([0, -newsize[1] / 2,
                                            -newsize[2] / 2])

                    center_relative = [0, -newsize[1] / 2, -newsize[2] / 2]
                    self.link_to_centers.append(center_relative)

                elif direction == 7:

                    self.joint_posns.append([center_relative[0], center_relative[1] - lastsize[1] / 2,
                                             center_relative[2] + lastsize[2] / 2])
                    self.joint_axes.append("1 0 0")
                    self.link_posns.append([0, -newsize[1] / 2,
                                            newsize[2] / 2])

                    center_relative = [0, -newsize[1] / 2, newsize[2] / 2]
                    self.link_to_centers.append(center_relative)


                old_weights = self.weights
                self.weights = np.random.random((len(self.links_with_sensors), self.num_joints))
                self.weights[:old_weights.shape[0], :old_weights.shape[1]] = old_weights

    def Set_ID(self, val):
        self.myID = val

