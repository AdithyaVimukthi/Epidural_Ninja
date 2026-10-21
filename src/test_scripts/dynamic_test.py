
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path

from dynamics import ArmDynamics
import numpy as np

dyn = ArmDynamics("left")

q = [0, 0, 0, 45, 0, 0, 0]

tau_g = dyn.gravity_torque(q)      # Nm, one per joint
# J = dyn.jacobian_world([0, -30, 0, 45, 0, 20, 0])          # 6x7, world-aligned at TCP
# tau = dyn.ee_force_to_torque(q_deg, [0, 0, 5, 0, 0, 0])    # 5N in world +z
print(tau_g)

joint_tauG_enable = [0,0,0,0,1,1,0]

tau_g = np.array(tau_g) * np.array(joint_tauG_enable)

print(tau_g)