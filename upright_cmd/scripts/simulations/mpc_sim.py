#!/usr/bin/env python3
"""Closed-loop upright simulation using Pybullet."""
import datetime

import numpy as np
from pyb_utils.frame import debug_frame_world

import upright_sim as sim
import upright_core as core
import upright_control as ctrl
import upright_cmd as cmd


def main():
    cli_args = cmd.cli.sim_arg_parser().parse_args()

    # load configuration
    config = core.parsing.load_config(cli_args.config)
    sim_config = config["simulation"]
    ctrl_config = config["controller"]

    # start the simulation
    timestamp = datetime.datetime.now()
    env = sim.simulation.UprightSimulation(
        config=sim_config,
        timestamp=timestamp,
        video_name=cli_args.video,
        gui=False,
        extra_gui=sim_config.get("extra_gui", False),
    )

    # initial time, state, input
    t = 0.0
    q, v = env.robot.joint_states()
    a = np.zeros(env.robot.nv)
    x = np.concatenate((q, v, a))
    u = np.zeros(env.robot.nu)

    # controller
    ctrl_manager = ctrl.manager.ControllerManager.from_config(ctrl_config, x0=x)
    dims = ctrl_manager.model.settings.dims
    ref = ctrl_manager.ref

    # frames for desired waypoints
    if sim_config.get("show_debug_frames", False):
        for r_ew_w_d, Q_we_d in ref.poses():
            debug_frame_world(0.2, list(r_ew_w_d), orientation=Q_we_d, line_width=3)

    v_cmd = np.zeros_like(v)
    a_est = np.zeros_like(a)

    # simulation loop
    while t <= env.duration:
        q, v = env.robot.joint_states(add_noise=False)
        x = np.concatenate((q, v, a_est))

        # compute policy - MPC is re-optimized automatically when the internal
        # MPC timestep has been exceeded
        u = ctrl_manager.step(t, x)[1]
        u_cmd = u[: dims.robot.u]

        # integrate the command
        # it appears to be desirable to open-loop integrate velocity like this
        # to avoid PyBullet not handling velocity commands accurately at very
        # small values
        v_cmd = v_cmd + env.timestep * a_est + 0.5 * env.timestep ** 2 * u_cmd
        a_est = a_est + env.timestep * u_cmd

        # generated velocity is in the world frame
        env.robot.command_velocity(v_cmd, bodyframe=False)

        t = env.step(t, step_robot=False)[0]


if __name__ == "__main__":
    main()
