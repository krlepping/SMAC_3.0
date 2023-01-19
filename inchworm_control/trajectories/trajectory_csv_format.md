# Trajectory CSV format guide

Each line of the trajectory **must** have 7 values.
The first 5 values are the values of the line correspond to the angles of each of the joints, from 0-4. The last two values are used to control the magnets on the end effector. The standard value for these is 0. This will result in no action by the magnets. If the magnet fields contain a 1, that indicates that the magnet must be toggled to reach this position. The timing is handled by run_trajectory.py, and assumes that only one end effector will be in the air at a time. 

An example line of a trajectory is included below.

| Joint 0 | Joint 1 | Joint 2 | Joint 3 | Joint 4 | Magnet 1 Toggle Needed | Magnet 2 Toggle Needed |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 20 | 140 | 20 | 0 | 1 | 0 |
