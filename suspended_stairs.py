import mujoco as mj
import random
import numpy as np


# https://c7.alamy.com/comp/2AHRGR7/high-angle-view-of-the-medieval-semi-circular-stone-staircase-outside-the-entrance-to-the-dominican-monastery-in-the-old-town-of-dubrovnik-croatia-2AHRGR7.jpg

def tread_mill():
  # https://github.com/google-deepmind/mujoco/issues/547
  pass

def floating_platform_for_circular_stair():
  # TODO
  # make the platform out of three rectangles.
  # the tendon sites need to be place on outer and inner circle
  # outer and inner circles are at the edges of the platforms
  # https://www.google.com/search?sca_esv=bf2c799dee4adccd&sxsrf=AHTn8zogVNx3s3lRtL5dfb_zMDRJUt0nqg:1746614086661&q=circular+stairs&udm=2&fbs=ABzOT_CWdhQLP1FcmU5B0fn3xuWpA-dk4wpBWOGsoR7DG5zJBsxayPSIAqObp_AgjkUGqengxVrJ7hrmYmz7X2OZp_NIYfhIAjPnSJLO3GH6L0gKvuUU9jOm91_NGkSK1WJYVWjjHf1cMeOfIs5S2VkHB51zdvs5rEFgawK72NTjOMMeP0ZkzDJsLwBUA55RuQQ6_IgnTa_0Sg-ZSjc8BLe0lulqWQaGgA&sa=X&ved=2ahUKEwi5ysfBlJGNAxW6zQIHHW7kAjsQtKgLegQIERAB&biw=1708&bih=958&dpr=2#vhid=JrepwICK0wfd-M&vssid=mosaic
  # https://www.archdaily.com/642612/living-staircase-paul-cocksedge/557f8523e58ece7103000058-living-staircase-paul-cocksedge-photo
  # https://www.archdaily.com/642612/living-staircase-paul-cocksedge/557f8592e58ece710300005a-living-staircase-paul-cocksedge-photo
  pass

def floating_platform(spec, gird_loc=[0, 0, 0], theta = 0, name='platform'):
  # https://dutchess.com/standard-stair-dimensions/
  # https://architectureideas.info/wp-content/uploads/2013/10/staircase.jpg
  PLATFORM_LENGTH = 0.5
  WIDTH = 0.12
  INWARD_OFFSET = 0.008
  THICKNESS = 0.005
  SIZE = [PLATFORM_LENGTH, WIDTH, THICKNESS]
  TENDON_LENGTH = 0.5
  Z_OFFSET = 0.1

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Platform with sites
  gird_loc[2] += Z_OFFSET
  platform = spec.worldbody.add_body(pos=gird_loc, name=name,euler =[0,0,theta])
  platform.add_geom(size= SIZE)
  platform.add_freejoint()

  for x_dir in [-1, 1]:
    for y_dir in [-1, 1]:

      # Add site to world
      rotation_matrix = np.array([[np.cos(-theta), -np.sin(-theta)],
                                  [np.sin(-theta), np.cos(-theta)]])
      vector = np.array([x_dir * PLATFORM_LENGTH, y_dir * ( WIDTH - INWARD_OFFSET) ])
      vector = np.dot(vector , rotation_matrix)

      x_w = gird_loc[0] + vector[0]
      y_w = gird_loc[1] + vector[1]
      z_w = gird_loc[2] + TENDON_LENGTH
      # Rotate sites by theta
      spec.worldbody.add_site(name=f'{name}_hook_{x_dir}_{y_dir}',
                              pos=[ x_w, y_w, z_w],
                              size=[0.01, 0, 0])
      # Add site to platform
      x_p = x_dir * PLATFORM_LENGTH
      y_p = y_dir * (WIDTH - INWARD_OFFSET)
      platform.add_site(name=f'{name}_anchor_{x_dir}_{y_dir}',
                        pos=[ x_p, y_p, THICKNESS * 2],
                        size=[0.01, 0, 0])

      # Connect tendon to sites
      thread = spec.add_tendon(name = f'{name}_thread_{x_dir}_{y_dir}', limited=True,
                               range=[0, TENDON_LENGTH], width = 0.01 )
      thread.wrap_site(f'{name}_hook_{x_dir}_{y_dir}')
      thread.wrap_site(f'{name}_anchor_{x_dir}_{y_dir}')

def simple_suspended_stair(spec, num_stair=40):
  V_STEP = 0.076
  H_STEP = 0.12

  for i in range(num_stair):
    floating_platform(spec,[0, i * H_STEP, i * V_STEP], name =f'p_{i}')

def sin_suspended_stair(spec, num_stair=40):
  V_STEP = 0.076
  H_STEP = 0.12
  AMPLITUDE = 0.2
  FREQUENCY = 0.5

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


def circular_stairs(spec, grid_loc=[0, 0], num_stair=40,
                    radius=1.5, v_step=0.076, h_step=0.05):
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

        # Create the floating platform.
        floating_platform(spec,[x, y, z],theta=theta , name=f'p_{i}')

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
  spec.compiler.degree  = False

  spec.option.enableflags |= mj.mjtEnableBit.mjENBL_OVERRIDE
  spec.option.enableflags |= mj.mjtEnableBit.mjENBL_MULTICCD
  spec.option.timestep = 0.0001

  main = spec.default
  main.geom.solref = [0.001, 1]


  # floating_cube(spec)
  # simple_suspended_stair(spec)
  sin_suspended_stair(spec)
  # circular_stairs(spec)


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

