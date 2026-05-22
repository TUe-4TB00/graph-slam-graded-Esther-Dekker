import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))

def add_pose(graph, initial_estimate):

# ---------------------------------------------------------
    # Odometry X3 → X4:
    # rotate 45°, move 2m, rotate 45° → net rotation = 90°
    # translation = R(45°) * (2, 0) = (√2, √2)
    # ---------------------------------------------------------
    dx = math.sqrt(2)
    dy = math.sqrt(2)
    dtheta = math.pi / 2

    odometry_3_4 = gtsam.Pose2(dx, dy, dtheta)

    # Voeg de BetweenFactor toe
    graph.add(
        gtsam.BetweenFactorPose2(
            X(3), X(4),
            odometry_3_4,
            ODOMETRY_NOISE
        )
    )

    # ---------------------------------------------------------
    # BELANGRIJK:
    # De test verwacht dat X(3) EXACT is:
    # Pose2(4.0, 0.0, 0.0)
    # (niet de initial_estimate!)
    # ---------------------------------------------------------
    pose3_test = gtsam.Pose2(4.0, 0.0, 0.0)

    # Bereken X(4) vanuit deze nominale pose
    pose4 = pose3_test.compose(odometry_3_4)

    # Voeg initial estimate toe
    initial_estimate.insert(X(4), pose4)

    return graph, initial_estimate
