
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    # TODO: Add the odometry factor between X(4) and X(5) to the graph (BetweenFactorPose2)
    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph
    
    # Find out what the last pose in our initial estimate is
    # When X(3) is the highest key present, we are making X(4); when X(4) is the highest, we make X(5)

    existing_poses = [ i for i in range(1,100) if initial_estimate.exists(X(i))]
    last_pose_idx = max(existing_poses)
    new_pose_idx = last_pose_idx + 1

    # Rotate 45 degrees anti-clockwise and move 2m, then rotate 45 degrees more
    odometry_delta = gtsam.Pose2(2.0, 0.0, math.pi/2.0)

    # Add odometry factor between last and new pose
    graph.add(gtsam.BetweenFactorPose2(X(last_pose_idx), X(new_pose_idx), odometry_delta, ODOMETRY_NOISE))

    # Find initial estimate of previous pose
    prev_pose_estimate = initial_estimate.atPose2(X(last_pose_idx))

    # Extrapolate new pose
    new_pose_estimate = prev_pose_estimate.compose(odometry_delta)

    initial_estimate.insert(X(new_pose_idx), new_pose_estimate)

    return graph, initial_estimate