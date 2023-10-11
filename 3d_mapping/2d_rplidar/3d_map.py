#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from mavsdk import System
import asyncio
import quaternion

class PointMapper:
    def __init__(self):
        self.cord = []
        self.head = 0
        self.poin=None
        self.rang=None
        self.flag=False
        self.b=0
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.dr_or=[]
        self.drone = System()

    def map(self, point_quaternions, dr_or):
        rotated_point_quaternions = [dr_or * p * np.conjugate(dr_or) for p in point_quaternions]
        rotated_points = np.array([q.vec for q in rotated_point_quaternions])
        self.cord.append(rotated_points)

    def count(self, data):
        if self.flag==False:

         self.rang = np.array(data.ranges)
         n = len(self.rang)
         cord_og = np.zeros((n, 3))
         alpha = data.angle_increment
         print(self.dr_or)
         self.poin = [rang if 1 / rang != 0 else 0 for rang in self.rang]
         for i in range(n):
    # Check if rang[i] is a valid number and alpha * i is within a valid range
         
            cord_og[i][0] = self.poin[i] * np.sin(alpha * i)
            cord_og[i][1] = self.poin[i] * np.cos(alpha * i)
            cord_og[i][2] = 0

         point_quaternions = [quaternion.from_float_array([0, x, y, z]) for x, y, z in cord_og]
         self.map(point_quaternions, self.dr_or)
        elif self.flag==True:
            return
            
    async def run(self):
        await self.drone.connect(system_address="udp://:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break
        rospy.init_node('map')
        rospy.Subscriber('/laser/scan', LaserScan, self.count)
        rate=rospy.Rate(10) 
        i = 0
        while i < 3001:
            i += 1

            async for h_deg in self.drone.telemetry.heading():
                self.head = h_deg
                break
            async for od in self.drone.telemetry.odometry():
                self.w = od.q.w
                self.x = od.q.x
                self.y = od.q.y
                self.z = od.q.z
                self.dr_or = quaternion.from_float_array([self.w, self.x, self.y, self.z])
                break
           
            self.b+=1
            print(f'iter.{self.b}')
            rate.sleep()
            if i>=2000:
                self.flag=True
                break
        return

    def plot_points(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        print(np.shape(self.cord))
        # Plot the data points
        for cord_set in self.cord:
            ax.scatter(cord_set[:, 0], cord_set[:, 1], cord_set[:, 2],  color='red')

        # Set labels for the axes
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        # Show the plot
        plt.show(block=True)

async def main():
    point_mapper = PointMapper()
    await point_mapper.run()
    point_mapper.plot_points()

if __name__ == "__main__":
    asyncio.run(main())
