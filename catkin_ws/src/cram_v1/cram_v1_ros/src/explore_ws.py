import trac_ik
import json
import time

# Script to get WidowX robot to move to posistions in its workspace and see if they are sucessfull in getting there.

startTime = time.time()

ik_solver = trac_ik.IK(base_link="widowx_2_arm_base_link", tip_link="widowx_2_wrist_1_link")
print(ik_solver.joint_names)
lb, ub = ik_solver.get_joint_limits()
print(lb, ub)
# TODO: Use the joint angles for the vertical print idle for initial seed
seed = [0.00, -1.547786614976612, 1.5247769031583274, -0.018407769454627694]  # Vertical 3d print idle
default_q = [0, 1, 0, 0]
num_attempts = 10

# Create dictionary to hold all data about each grid point
i = list(range(0, 31))
j = list(range(-30, 31))
k = list(range(0, 26))

# dead-space range
ds_i = list(range(0, 9))
ds_j = list(range(-8, 9))
ds_k = list(range(0, 26))
# Testing
# Workspace
grid = [(value_i, value_j, value_k) for col, value_i in enumerate(i) for col, value_j in enumerate(j)
        for row, value_k in enumerate(k)]

ds = [(value_i, value_j, value_k) for col, value_i in enumerate(ds_i) for col, value_j in enumerate(ds_j)
      for row, value_k in enumerate(ds_k)]

# No of points in ws
num_elements = len([listElem for listElem in grid])

# Remove dead-space and assign grid to workspace
ws = {"params":{"solve_type": ik_solver._solve_type}, "timeout": ik_solver._timeout, "epsilon":ik_solver._epsilon}
for n in grid:
    if n in ds:
        pass
    else:
        ws[n] = {"reachable": None, "attempts": None}

# TODO: Adding variable grid density so I sample of the edges of the workspace with a finer resolution

for key in ws.keys():
    sol = None
    attempt = 0

    #TODO: Add change of quaternion here from updated class
    q = "Some code for quaternions"
    while sol is None and attempt < num_attempts:
        sol = ik_solver.get_ik(seed, key[0], key[1], key[2], q[0], q[1], q[2], q[3])

        if sol is None and attempt == num_attempts - 1:
            ws[key]["reachable"] = False
            ws[key]["attempts"] = num_attempts

         elif sol is not None:
            ws[key]["reachable"] = True
            ws[key]["attempts"] = attempt
        else:
            attempt += 1

    # Add code to save dict to json
    # This adds a lot of overhead but it means if something goes wrong at least I have some of the data saved,
    with open('ws_data.json', 'w') as fp:
        json.dump(ws, fp, indent=4)

# Add a timing function to time how long this takes.
executionTime = (time.time() - startTime)
print('Execution time in seconds: ' + str(executionTime))