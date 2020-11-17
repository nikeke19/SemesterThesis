#! /usr/bin/env  python

import pandas as pd
import rospy
import numpy as np
from pathlib import Path
from mabi_msgs.srv import LoadMapRequest, LoadMap, WriteCenteredOcGrid, WriteCenteredOcGridRequest


def set_up_file_list(n_goals, n_starts, worlds=[]):
    files_start_goal = []
    for world in worlds:
        for goal in range(n_goals):
            for start in range(n_starts):
                file_trajectory = Path(
                    "/home/nick/Data/Table/world_" + str(world) + "/world_" + str(world) + "_goal_" + str(goal) +
                    "_start_" + str(start) + "_condition.txt")

                # If Data file exists, add to path
                if file_trajectory.is_file():
                    files_start_goal.append([file_trajectory.absolute(), world, goal, start])
    return files_start_goal


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


def rewrite_oc_grids(n_goals, n_starts, worlds=[]):
    files_start_goal = set_up_file_list(n_goals, n_starts, worlds)
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
                rospy.sleep(20)
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


########################################################################################################################
## Rewrite OC_GRID from center file
########################################################################################################################

def find_centers_from_file(world):
    path = "/home/nick/Data/Table/world_" + str(world) + "/world_" + str(world) + "_center.csv"
    # path = "home/nick/Data/Table/world_0/world_0_center.csv"
    df = pd.read_csv(path, header=None)
    oc_grid_list = []
    for row in df.iterrows():
        # print("hi")
        oc_grid_list.append([row[1][0], row[1][1], row[1][2]])
    return oc_grid_list


def rewrite_centered_grid_from_list(world):
    rospy.wait_for_service("write_oc_grid_to_file")
    print("service write_oc_grid_to_file found")
    srv_client_oc_grid = rospy.ServiceProxy("write_oc_grid_to_file", WriteCenteredOcGrid)
    req = WriteCenteredOcGridRequest()

    oc_grid_list = find_centers_from_file(world)
    if load_new_map("/home/nick/mpc_ws/src/perceptive_mpc/maps/esdfs/world_" + str(world) + ".esdf"):
        print("Going 40s to sleep while loading map")
        rospy.sleep(20)
        print("Woke up from sleep")
    else:
        print("Error, Map can't be loaded!", " world_", str(world))
        return -1

    for row in oc_grid_list:
        print("________________________")
        print(row[0])
        print("________________________")

        req.name = row[0]
        req.resolution = 0.2
        req.center = [row[1], row[2]]
        # print(req.center)
        try:
            response = srv_client_oc_grid(req)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return -1


if __name__ == '__main__':
    # rospy.init_node('target_pose_publisher', anonymous=True)


    # rewrite_oc_grids(n_goals=70, n_starts=11, worlds=[3])
    rewrite_centered_grid_from_list(world=5)

    print("Finished writing data")
    # rospy.spin()
