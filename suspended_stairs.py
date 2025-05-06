import mujoco as mj
import random
import numpy as np


def tread_mill():
  # https://github.com/google-deepmind/mujoco/issues/547
  pass

def floating_platform(spec, gird_loc=[0, 0, 0], name='platform'):
  PLATFORM_LENGTH = 0.1
  WIDTH = 0.02
  INWARD_OFFSET = 0.008
  THICKNESS = 0.001
  SIZE = [PLATFORM_LENGTH, WIDTH, THICKNESS]

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Platform with sites
  platform = spec.worldbody.add_body(pos=[0, 0, 0], name=name)
  platform.add_geom(size= SIZE)
  platform.add_freejoint()

  for x in [-1, 1]:
    for y in [-1, 1]:
      # Add site to world
      spec.worldbody.add_site(name=f'{name}_hook_{x}_{y}',
                              pos=[ gird_loc[0] + x * PLATFORM_LENGTH,
                                    gird_loc[1] + y * (WIDTH - INWARD_OFFSET),
                                    gird_loc[2] + 0.2],
                              size=[0.002, 0, 0])
      # Add site to platform
      platform.add_site(name=f'{name}_anchor_{x}_{y}',
                        pos=[ x * PLATFORM_LENGTH, y * (WIDTH - INWARD_OFFSET), THICKNESS * 2],
                        size=[0.002, 0, 0])

      # Connect tendon to sites
      thread = spec.add_tendon(name = f'{name}_thread_{x}_{y}', limited=True, range=[0, 0.1], width=0.001 )
      thread.wrap_site(f'{name}_hook_{x}_{y}')
      thread.wrap_site(f'{name}_anchor_{x}_{y}')

def simple_suspended_stair(spec, num_stair=4):
  V_STEP = 0.01
  H_STEP = 0.04

  for i in range(num_stair):
    floating_platform(spec,[0, i * H_STEP, i * V_STEP], name =f'p_{i}')

def sin_suspended_stair(spec, num_stair=80):
  V_STEP = 0.01
  H_STEP = 0.04
  AMPLITUDE = 0.2
  FREQUENCY = 1

  for i in range(num_stair):
    x_step = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY * (i * H_STEP))
    floating_platform(spec,[x_step, i * H_STEP, i * V_STEP], name =f'p_{i}')

def floating_cube(spec):
  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Add site to world
  spec.worldbody.add_site(name='hook', pos=[0, 0, 0.2], size=[0.002, 0, 0])

  # Cube with site
  cube = spec.worldbody.add_body(pos=[0, 0, 0])
  cube.add_geom(size=[ 0.015]*3)

  cube.add_freejoint()
  cube.add_site(name='anchor',pos = [0.015]*3, size=[0.002, 0, 0])

  # Connect tendon to sites
  thread = spec.add_tendon(name = 'thread', limited=True,
                           range=[0, 0.02], width=0.001 )
  thread.wrap_site('hook')
  thread.wrap_site('anchor')


def circular_stairs(spec, grid_loc=[0, 0], num_stair=40, radius=1, v_step=0.01, h_step=0.05):
    """
    Generates circular stairs around a central point.

    Args:
        spec: The specification object (assuming this is defined elsewhere and has a
              'floating_platform' method).
        grid_loc (list, optional): The center of the circle in the grid. Defaults to [0, 0].
        num_stair (int, optional): The number of stairs. Defaults to 80.
        radius (float, optional): The radius of the circle. Defaults to 1.
        v_step (float, optional): The vertical step between stairs. Defaults to 0.01.
        h_step (float, optional): The horizontal step, influencing the width of each stair.
                        It's more accurately an angular step in this case. Defaults to 0.05.
    """
    # TODO make stairs tangent to circle

    # Use the number of stairs to determine the angle increment.
    theta_step = 2 * np.pi / num_stair  # Angle step for each stair to cover a full circle.

    for i in range(num_stair):
        # Calculate the angle for this stair.
        theta = i * theta_step

        # Convert polar coordinates (r, theta) to Cartesian coordinates (x, y).
        x = grid_loc[0] + radius * np.cos(theta)
        y = grid_loc[1] + radius * np.sin(theta)

        # Calculate the z-coordinate based on the stair number.
        z = i * v_step
        # Calculate the tangent vector (perpendicular to the vector from the center).
        tangent = np.array([-y + grid_loc[1], x - grid_loc[0]])  # Tangent relative to the center

        # Normalize the tangent vector.
        norm = np.linalg.norm(tangent)
        if norm != 0:  # Avoid division by zero
            tangent /= norm
        else:
            tangent = np.array([0, 0 ]) # handle the zero case

        print(f"tangent::{tangent}")

        # Angle between (1,0) and tangent
        theta = np.arccos(np.clip(np.dot(np.array([1,0]), tangent), -1.0, 1.0))
        print(f"theta::{theta}")

        # Create the floating platform.
        floating_platform(spec,[x, y, z], name=f'p_{i}')

if __name__ == "__main__":

  arena_xml = """
  <mujoco model="2f85 scene">

    <!-- Add some fluid viscosity to prevent the hanging box from jiggling forever -->
    <option viscosity="0.1"/>

    <statistic center="0 0 0.05" extent="0.3"/>

    <visual>
      <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
      <rgba haze="0.15 0.25 0.35 1"/>
      <global azimuth="60" elevation="-20"/>
    </visual>

    <asset>
      <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
        markrgb="0.8 0.8 0.8" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    </asset>

    <worldbody>
      <light pos="0 0 1"/>
      <light pos="0 -0.2 1" dir="0 0.2 -0.8" directional="true"/>
      <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    </worldbody>
  </mujoco>

  """

  spec = mj.MjSpec.from_string(arena_xml)


  # floating_cube(spec)
  # simple_suspended_stair(spec)
  # sin_suspended_stair(spec)
  circular_stairs(spec)


  model = spec.compile()
  data = mj.MjData(model)

  with open("./suspension.xml","w") as f:
    f.write(spec.to_xml())

  # visualization
  with mj.viewer.launch_passive(
      model = model, data = data,
      show_left_ui = False, show_right_ui = False
  ) as viewer:
    mj.mjv_defaultFreeCamera(model, viewer.cam)
    while viewer.is_running():
      mj.mj_step(model, data)
      viewer.sync()

