# Experiment Configuration

The configuration for each experiment (simulated or hardware) is specified
using a YAML file, which are typically stored in `upright_cmd/config/`.

## Including other YAML files

Often only a few configuration parameters change between experiments, so we
would like to be able to extend a general shared YAML file with only these few
differences. This is done using the `include` key, the value of which is block
sequence; each element of the sequence contains `package`, the name of the ROS
package, and `path`, the relative path to the YAML file from that package. The
element can also contain the `key` key, which specifies the key that included
YAML parameters should be placed under in the overall configuration hierarchy.

For example, suppose I have my YAML file `mine.yaml` and I want to include the
parameters from `shared.yaml` located at `upright_cmd/config/shared.yaml`,
where `upright_cmd` is a ROS package. Then in `mine.yaml` (typically at the
top), I write:
```yaml
include:
  -
    package: upright_cmd
    path: config/shared.yaml
```

Now suppose that I also want to include some control parameters from the file
`upright_cmd/config/controller.yaml`, and I want these parameters to be nested
under the `controller` key. Then I can augment the `include` statement about
to
```yaml
include:
  -
    package: upright_cmd
    path: config/shared.yaml
  -
    key: controller
    package: upright_cmd
    path: config/controller.yaml
```

When one file includes another, any keys present in both take the value from
the file doing the including; in other words, the values in the included file
are overwritten. You can include as many files into another as desired.

## Parameters

The top-level keys for the upright project are
```yaml
controller  # Controller parameters.
simulation  # Simulation parameters.
logging     # How/where to log data (only used for simulation).
```

### Logging

Nested under `logging` are the following keys:
```yaml
timestep: float  # How often to record data (in seconds).
log_dir:  str    # The absolute path to directory in which to save data.
```
These values are used by the `DataLogger` in
`upright_core/src/upright_core/logging.py`.

### Controller
Nested under `controller` are the following keys:
```yaml
gravity: list of float, length 3    # Gravity vector.

# Upright uses OCS2's auto-differentiation + code generation to automatically compute gradients of costs and constraints.
# Set this to `true` to recompile each time, or `false` to skip this step.
# Only set to `false` if running the same controller setup repeatedly (i.e., same object arrangments and constraints; the waypoints can change).
recompile_libraries: bool

# Enable extra debugging information. Currently, this is used to print and publish more information from the MRT node in `upright_ros_interface/src/mrt_node.cpp`.
debug: bool

# The solver to use. Currently only `SQP` (sequential quadratic programming) is supported.
solver_method: str

# Settings for the model predictive controller.
mpc:
  time_horizon: float, non-negative  # Optimization time horizon.
  debug_print: bool                  # Set `true` to print extra information.
  cold_start: bool                   # Set `false` to warm start the solver at each control iteration.

# Settings for sytem state estimation.
# Estimation is done with a Kalman filter.
# Noise is assumed isotropic, so variance can be represented with a single scalar.
estimation:
  robot_init_variance: float, non-negative         # Initial pose variance.
  robot_process_variance: float, non-negative      # Process variance.
  robot_measurement_variance: float, non-negative  # Measurement variance.

# Settings for low-level reference tracking.
tracking:
  rate: int, non-negative                      # Controller frequency [Hz].
  min_policy_update_time: float, non-negative  # Don't switch the MPC policy more often than this [s].

  # State feedback gains.
  # These should be set to zero if sqp.use_feedback_policy = true, since the controller computes its own optimal feedback policy in that case.
  kp, kv, ka: float, non-negative

  # For each of these options, if it is `true`, then the controller with stop when the state, input, or EE position limits are violated, respectively.
  enforce_state_limits: bool
  enforce_input_limits: bool
  enforce_ee_position_limits: bool

  # Margins for violation of the state, input, and EE position bounds.
  state_violation_margin: float, non-negative
  input_violation_margin: float, non-negative
  ee_position_violation_margin: float, non-negative

  # Set to `true` when doing projectile experiments.
  use_projectile: bool

# Settings for the sequential quadratic programming solver.
sqp:
  dt: float, non-negative            # Time step of the optimized trajectory [s].
  sqp_iteration: int, positive       # Max number of SQP iterations per solve.
  init_sqp_iteration: int, positive  # Max number of SQP iterations during first solve.

  # Convergence parameters.
  delta_tol: float, non-negative
  cost_tol: float, non-negative

  # Set `true` for MPC to compute a linear feedback policy.
  use_feedback_policy: bool

  # Set true to project the state and input onto the lower-dimensional space defined by the affine inequality constraints.
  project_state_input_equality_constraints: bool

  # Print solver information.
  print_solver_status: bool
  print_solver_statistics: bool
  print_line_search: bool

  # Settings for the HPIPM QP solver used to solve each QP.
  hpipm:
    iter_max: int, positive  # Max number of iterations.
    warm_start: bool         # Set `true` to warm start the solver.
    slacks:                  # Slack variable settings.
      enabled: bool          # Enable slack variables.

      # "Lower" slacks are associated with lower-bounded inequality constraints; "upper" slacks with the upper-bounded ones.
      upper_L2_penalty: float, non-negative  # L2 penalty on upper slacks (default: 100).
      lower_L2_penalty: float, non-negative  # L2 penalty on lower slacks (default: 100).
      upper_L1_penalty: float, non-negative  # L1 penalty on upper slacks (default: 0).
      lower_L1_penalty: float, non-negative  # L1 penalty on lower slacks (default: 0).
      upper_low_bound: float  # Lower bound on the upper slacks (default: 0).
      lower_low_bound: foat  # Lower bound on the lower slacks (default: 0).

# Settings for the balancing constraints.
balancing:
  enabled: bool  # Set `true` to enable the balancing constraints.

  # Name of the arrangement of objects being balanced.
  arrangement: str

  # `Hard` for hard constraints; `soft` to enforce constraints via cost penalties.
  # Soft constraints have not been extensively tested.
  constraint_type: hard | soft

  # Set `true` to use contact-forced based constraints, or `false` to use constraints based on the zero-moment point and limit surface.
  # The latter constraints are less flexible and are no longer fully supported.
  use_force_constraints: bool

  # Weight on the contact forces in the objective function.
  force_weight: float, non-negative

  # Whether to do frictionless constraints
  frictionless: bool

# Settings for the inertial alignment method, an alternative to the balancing constraints.
# Inertial alignment tries to tilt the tray so that its normal is always aligned opposite to the gravito-inertial acceleration.
inertial_alignment:
  # Set `true` to add inertial alignment as a cost.
  cost_enabled: bool

  # Set `true` to add inertial alignment as a constraint.
  # Should not be used in conjunction with `cost_enabled`.
  constraint_enabled: bool

  # Take the angular acceleration of the specified `com` into account when computing alignment.
  # The `com` parameter has no effect if this is `false`.
  use_angular_acceleration: bool

  # The point around which to compute the acceleration, with respect to the tray's origin (only has an effect when `use_angular_acceleration` is `true`).
  com: list of float, length 3

  # Instead of aligning with the acceleration vector, set to `true` to align with `contact_plane_normal`, expressed in the world frame.
  # This is useful for keeping the tray flat.
  align_with_fixed_vector: bool

  # The world-frame normal vector to align with if `align_with_fixed_vector` is true.
  contact_plane_normal: list of float, length 3

# For safety, the end effector can be restricted to lie inside of a box.
end_effector_box_constraint:
  enabled: bool                       # Set `true` to enable this constraint in the controller.
  xyz_lower: list of float, length 3  # Task-space lower bound.
  xyz_upper: list of float, length 3  # Task-space upper bound.

# Settings for constraining the robot to avoid the path of a projectile.
projectile_path_constraint:
  enabled: bool  # Set `true` to enable this constraint in the controller.

  # The names of the links to constrain to avoid collision with the projectile.
  collision_links: list of str

  # The minimum distance that should be maintained between the projectile and each link in `collision_links`.
  distances: list of float

  # Scale the constraints by this value.
  scale: float, non-negative

# List of waypoints defining the trajectory relative to the EE's initial pose.
waypoints:
  -
     # Time the EE should be at this waypoint.
     time: float, non-negative

     # EE position (x, y, z).
     position: list of float, length 3

     # EE orientation quaternion (x, y, z, w).
     orientation: list of float, length 4

# Settings for general obstacle avoidance.
obstacles:
  enabled: bool         # Set `true` to enable this constraint in the controller.
  constraint_type: hard | soft  # `Soft` not fully supported.

  # Minimum distance to enforce between objects.
  minimum_distance: float, positive

  # List of pairs of str.
  # Defines the pairs of links which should be constrained not to collide.
  collision_pairs:
    - [link1, link2]  # For example.

  # List of dynamic obstacles.
  dynamic:
    -
      name: str                # Name of the dynamic obstacle.
      radius: float, positive  # Radius of the dynamic obstacle.

      # Modes of the obstacle.
      modes:
        -
          # This mode is active from this time until mode with the next time.
          time: float, non-negative

          # State of the obstacle at the start of the mode.
          position: list of float, length 3
          velocity: list of float, length 3
          acceleration: list of float, length 3

# The controller can be initialized around some operating points.
# May not fully supported.
operating_points:
  enabled: bool  # Leave `false`.

# List of the balanced object parameters from the perspective of the controller.
objects:
  object_name: # Name of the object.
    # Type of shape.
    shape: cuboid | cylinder | wedge

    # Parameters for different shape types.
    side_lengths: list of float, length 3  # Side lengths of a cuboid or wedge.
    radius: float, non-negative            # Radius of a cylinder.
    height: float, non-negative            # Height of a cylinder.

    # Offset of the center of mass from the shape's centroid.
    com_offset: list of float, length 3

    # That mass of the object.
    mass: float, non-negative

    # The diagonal of the inertia matrix about the CoM.
    # This is optional; if not specified, shape is assumed to have uniform density.
    inertia_diag: list of float, length 3, non-negative

# Arrangements of the objects.
arrangements:
  arrangement_name:
    objects:  # List of objects composing the arrangement.
      -
        name: str  # The name of this particular object.
        type: str  # The type of the object; must correspond to an entry in `objects`.
        parent: str  # Name of (one of) the object(s) this one is placed upon.

        # Optional offset of this object's CoM in the horizontal plane w.r.t. the parent's CoM.
        # All the entries under offset are also optional and default to zero if not provided.
        offset:
          x: float  # X-component.
          y: float  # Y-component.

          # Polar coordinates can also be used.
          # If both Cartesian and polar coordinates are used, they are added together.
          r: float  # Distance.
          θ: float  # Angle.

    # Define the contact planes in the arrangement.
    contacts:
      -
        first: str   # Name of the first object in the contact pair.
        second: str  # Name of the second object.

        mu: float, non-negative  # Friction coefficient.

        # Safety margin for the controller.
        # The balancing constraints are formulated as if the friction coefficient is `mu - mu_margin`.
        mu_margin: float, non-negative

        # Safety margin for the controller.
        # Contact points are pulled in by this amount so that the balancing constraints are more conservative.
        support_area_inset: float, non-negative

# Parameters of the robot.
robot:
  x0: list of float  # Initial state.

  # System dimensions.
  dims:
    q: int, positive  # Generalized position dimension.
    v: int, positive  # Generalized velocity dimension.
    x: int, positive  # State dimension.
    u: int, positive  # Input dimension.

  # URDF model defining the robot.
  urdf:
    package: str  # Package name.
    path: str     # Where to write the URDF relative to the package.

    # The xacro (which is a superset of plain URDF) files to include together to compile the URDF.
    includes: list of str

    # Specify any xacro argument values here.
    args:
      arg: value

  # The name of the link representing the tool (i.e., the tray).
  tool_link_name: str

  # Type of mobile base.
  base_type: omnidirectional | fixed | nonholonomic | floating

  # Optional map of joint names to the value at which they should remain constant.
  # This is useful if one only wants to work with a subset of the whole robot.
  locked_joints:
    joint_name: float

# Weights on state, input, and end effector pose.
weights:
  input:  # Input weight.
    scale: float, non-negative         # Scale coefficient for the whole weight matrix.
    diag: list of float, non-negative  # Weight matrix diagonal.
  state:  # State weight.
    scale: float, non-negative
    diag: list of float, non-negative
  end_effector:  # EE pose weight.
    scale: float, non-negative
    diag: list of float, non-negative, length 6  # 3 position DOFs, 3 orientation DOFs.

# State and input limits.
limits:
  constraint_type: hard | soft
  input:  # Input limits.
    lower: list of float
    upper: list of float
  state:  # State limits.
    lower: list of float
    upper: list of float

# Settings for performing the forward rollout of the trajectory.
# See <ocs2_oc/rollout/RolloutSettings.h>.
rollout:

# TODO
arrangements:

### Simulation
Nested under `simulation` are the following keys:
```yaml
timestep: float, non-negative     # Simulation timestep [s].
duration: float, non-negative     # Duration of the simuation [s].
gravity: list of float, length 3  # Gravity vector.

# Name of the arrangement of objects being balanced.
arrangement: str

# Set `true` to show the contact points between objects in their initial position useful for debugging.
show_contact_points: bool

# Set `true` to show reference frames including the initial EE pose and all desired waypoints.
# Useful for debugging.
show_debug_frame: bool

# Define virtual cameras to capture static shots of the scene or as a viewpoint for a video.
# Cameras can be defined in multiple ways.
cameras:
  # Using absolute target and camera position.
  camera_name:  # Name of the camera.
    target: list of float, length 3
    position: list of float, length 3

  # Using target and camera position relative to EE initial position.
  camera_name:
    relative_target: list of float, length 3
    relative_position: list of float, length 3

  # Using target and distance and orientation of the camera.
  # This one is convenient because these parameters can be read off of the PyBullet GUI.
  camera_name:
    target: list of float, length 3
    distance: float, non-negative
    roll: float
    pitch: float
    yaw: float

# Define videos to be captured during the simulation.
# This needs to be enabled by the command line argument --video.
video:
  # Set `true` to also save an image for each of the video frames (in addition to the video itself).
  # This is useful for post-processing the video, or taking stills of particular frames.
  safe_frames: bool

  # Absolute path to the directory in which to save the video.
  dir: str

  # Take a new frame every `timestep` seconds.
  timestep: float, non-negative [s]

  # Multiple videos (i.e., from different viewpoints) can be taken simultaneously.
  # Each one is specifed here.
  views:
    -
      camera: str  # The name of the camera to use to record the video.
      name: str    # The name of the video corresponding to this view.

# Define photos to be taken during the simulation.
# Currently only photos at the start or end of the simulation are supported.
photos:
  start:  # Viewpoints to take photos at the start.
    -
      camera: str
      name: str
  end:  # Viewpoints to take photos at the end.
    -
      camera: str
      name: str

static_obstacles:
  enabled: bool  # Add static obstacles to the simulation.

  # Obstacles are defined using a URDF.
  urdf:
    package: str  # Name of the package.
    path: str     # Where to write the URDF relative to the package.

    # The xacro (which is a superset of plain URDF) files to include together to compile the URDF.
    includes: list of str

dynamic_obstacles:
  enabled: bool  # Add dynamic obstacles to the simulation.

  # List of dynamic obstacles.
  obstacles:
    -
      # Set `true` if the obstacle's trajectory should be actively be controlled/tracked, or `false` if it should be left subject to the simulation's dynamics.
      controlled: bool

      # Radius of the obstacle.
      radius: float, positive

      # `Position` of each mode is relative to the initial EE if `true`, otherwise relative to world coordinates.
      relative: bool

      # Same as in the `controller.obstacles.dynamic` section.
      modes:

# Parameters of the robot.
# This is the same as in the `controller` section with the following exceptions:
robot:
  ...
  # Instead of `x0`, the simulation only requires the home joint configuration.
  home: list of floats

  # Measurement and process noise.
  noise:
    measurement:
      q_std_dev: float, non-negative  # Standard deviation of measured joint positions.
      v_std_dev: float, non-negative  # Standard deviation of measured joint velocities.
    process:
      v_std_dev: float, non-negative  # Standard deviation of velocity inputs.

  # List of the names of the joints being controlled.
  joint_names: list of str

# The objects are defined in the same way as in the `controller` section, except that there is color parameter as well:
objects:
  object_name:
    ...
    # [r, g, b, a] color, where each value is between 0 and 1, a is the alpha (transparency).
    color: list of float

# This has the same structure as in the `controller` section.
# We typically make a seperate arrangements.yaml file that is included in both the controller and simulation settings, where the parameters of the objects themselves are changed if we want to have differences between the two.
arrangements:
```
