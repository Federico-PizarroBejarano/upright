import time

import numpy as np

from upright_control.manager import ControllerModel
from upright_control import bindings
from upright_control.wrappers import TargetTrajectories

import IPython


class MPSFControllerManager:
    def __init__(self, model, timestep):
        self.model = model

        # compute EE pose
        self.model.update(x=self.model.settings.initial_state)

        # reference pose trajectory
        self.ref = self._create_empty_trajectory()

        self.timestep = timestep

        # MPC
        self.model.settings.input_weight = np.eye(self.model.settings.dims.robot.u)
        self.model.settings.state_weight = np.zeros((self.model.settings.dims.x(),
                                                     self.model.settings.dims.x()))
        self.model.settings.end_effector_weight = np.zeros((6, 6))
        self.mpc = bindings.ControllerInterface(self.model.settings)
        self.mpc.reset(self.ref)

        self.last_planning_time = -np.infty
        self.x_opt = np.zeros(self.model.settings.dims.x())
        self.u_opt = np.zeros(self.model.settings.dims.u())

        # time at which replanning was done
        self.replanning_times = []

        # duration the replanning took in real time (i.e. how fast did the
        # optimizer run)
        self.replanning_durations = []


    @classmethod
    def from_config(cls, config, x0=None):
        model = ControllerModel.from_config(config, x0=x0)

        # control should be done every timestep
        timestep = config["tracking"]["min_policy_update_time"]

        return cls(model, timestep)


    def update(self, ref):
        """Update the reference trajectory."""
        self.ref = ref
        self.mpc.reset(self.ref)


    def warmstart(self):
        """Do the first optimize to get things warmed up."""
        x0 = self.model.settings.initial_state
        u0 = np.zeros(self.model.settings.dims.u())
        self.mpc.setObservation(0, x0, u0)

        self.mpc.advanceMpc()
        self.last_planning_time = 0


    def step(self, t, x, desired_input):
        """
        MPSF step with dynamic desired input calculation.

        Args:
            t: Current time
            x: Current state
            desired_input: Desired input
        """
        self.update_desired_input(t, x, desired_input)

        # Set observation
        self.mpc.setObservation(t, x, self.u_opt)

        # Replan if needed
        if t >= self.last_planning_time + self.timestep:
            t0 = time.time()
            self.mpc.advanceMpc()
            t1 = time.time()

            self.last_planning_time = t
            self.replanning_times.append(t)
            self.replanning_durations.append(t1 - t0)

        # Evaluate current solution
        try:
            self.mpc.evaluateMpcSolution(t, x, self.x_opt, self.u_opt)
        except:
            IPython.embed()

        return self.x_opt, self.u_opt


    def get_mpc_trajectory(self):
        """Get the full optimal trajectory found by MPC."""
        ts = bindings.scalar_array()
        xs = bindings.vector_array()
        us = bindings.vector_array()
        self.mpc.getMpcSolution(ts, xs, us)
        return np.array(ts), np.array(xs), np.array(us)


    def _create_empty_trajectory(self):
        """Create an empty trajectory."""
        ts = bindings.scalar_array()
        xs = bindings.vector_array()
        us = bindings.vector_array()

        ts.push_back(0)
        r_ew_w, Q_we = self.model.robot.link_pose()
        self.initial_pose = (r_ew_w, Q_we)
        x = np.concatenate((r_ew_w, Q_we, [0]))
        xs.push_back(x)
        us.push_back(np.zeros(self.model.settings.dims.u()))

        return TargetTrajectories(ts, xs, us)


    def update_desired_input(self, t, x, desired_input):
        """
        Update the desired input trajectory dynamically.

        Args:
            t: Current time
            x: Current state
            desired_input: Desired input
        """

        # Create time trajectory
        ts = bindings.scalar_array()
        xs = bindings.vector_array()
        us = bindings.vector_array()

        ts.push_back(t)
        r_ew_w, Q_we = self.initial_pose
        x = np.concatenate((r_ew_w, Q_we, [0]))
        xs.push_back(x)
        us.push_back(desired_input)

        # Create new trajectory and update MPC
        new_ref = TargetTrajectories(ts, xs, us)
        self.ref = new_ref
        self.mpc.setTargetTrajectories(new_ref)
