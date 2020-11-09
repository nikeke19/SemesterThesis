#! /usr/bin/env  python

import pandas as pd
import rospy
import numpy as np
from pathlib import Path
from mabi_msgs.srv import LoadMapRequest, LoadMap, WriteCenteredOcGrid, WriteCenteredOcGridRequest

files_start_goal = []


def set_up_file_list(n_goals, n_starts, worlds_array=[0]):
    for world in worlds_array:
        for goal in range(n_goals):
            for start in range(n_starts):
                file_trajectory = Path(
                    "/home/nick/Data/Table/world_" + str(world) + "_goal_" + str(goal) + "_start_" + str(start)
                    + "_condition.txt")

                # If Data file exists, add to path
                if file_trajectory.is_file():
                    files_start_goal.append([file_trajectory.absolute(), world, goal, start])


def load_new_map(name):
    rospy.wait_for_service("load_map")
    print("service load map found")
    try:
        srv_client_load_map = rospy.ServiceProxy("load_map", LoadMap)
        msg = LoadMapRequest(name)
        response = srv_client_load_map(msg)
        if response.map_loaded:
            print("suceeded loading map")
            return True
        else:
            print("failed")
            return False
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False


def rewrite_oc_grids():
    rospy.wait_for_service("write_oc_grid_to_file")
    print("service write_oc_grid_to_file found")
    srv_client_oc_grid = rospy.ServiceProxy("write_oc_grid_to_file", WriteCenteredOcGrid)
    req = WriteCenteredOcGridRequest()
    old_world = -10
    for path, world, goal, start in files_start_goal:
        print("________________________")
        print("world_" + str(world) + "_start_" + str(start) + "_goal_" + str(goal))
        print("________________________")
        if old_world != world:
            old_world = world
            if load_new_map("/home/nick/mpc_ws/src/perceptive_mpc/maps/esdfs/world_" + str(world) + ".esdf"):
                print("Going 40s to sleep while loading map")
                rospy.sleep(40)
                print("Woke up from sleep")
            else:
                print("Error, Map can't be loaded!", " world_", str(world), "_goal_", str(goal), "_start_", str(start))
                return -1

        req.name = "world_" + str(world) + "_goal_" + str(goal) + "_start_" + str(start)
        req.resolution = 0.2
        req.center = find_center(path.absolute())
        try:
            response = srv_client_oc_grid(req)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return -1


def find_center(name):
    df = pd.read_csv(name)
    x = np.array(df)
    x_start = x[0, 0:2]
    x_end = x[0, 9:11]
    center = (x_start + x_end) / 2

    # return[0.0, 0.0]
    return [center[0], center[1]]


if __name__ == '__main__':
    rospy.init_node('target_pose_publisher', anonymous=True)
    set_up_file_list(n_goals=70, n_starts=11, worlds_array=[3])
    # set_up_file_list(n_goals=1, n_starts=5, worlds_array=[0, 1, 2, 3])
    # set_up_file_list(n_goals=1, n_starts=5, worlds_array=[3])
    rewrite_oc_grids()
    print("Finished writing data")
    rospy.spin()
