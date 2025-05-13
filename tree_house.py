import mujoco as mj
import random
import numpy as np


from enum import Enum

class Connection(Enum):
    BRIDGE = 1
    LADDER = 2

def platform(spec, gird_loc=[0, 0, 0],theta = 0, name='platform'):
  PLATFORM_LENGTH = 0.1
  WIDTH = 0.02
  INWARD_OFFSET = 0.008
  THICKNESS = 0.001
  SIZE = [PLATFORM_LENGTH, WIDTH, THICKNESS]

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Platform with sites
  platform = spec.worldbody.add_body(pos=[0, 0, 0.1], name=name, euler = [0,0,1.57])
  platform.add_geom(size= SIZE)
  # platform.add_freejoint()

def rope_ladder(spec,world_sites,num_stairs=10 ,name="stair"):

  PLATFORM_LENGTH = 0.1
  WIDTH = 0.02
  THICKNESS = 0.001
  SIZE = [PLATFORM_LENGTH, WIDTH, THICKNESS]
  TENDON_LENGTH = 0.1

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Create first stair
  platform_bellow = spec.worldbody.add_body(pos=[0, 0, 0.1], name=f"{name}_{0}")
  platform_bellow.add_geom(size= SIZE)
  platform_bellow.add_freejoint()

  tip_of_lader = []
  for i in range(num_stairs-1):
    print(f"i::{i}")
    # Create stair above
    platform_above = spec.worldbody.add_body(pos=[0, 0, 0.1], name=f"{name}_{i+1}")
    platform_above.add_geom(size= SIZE)
    platform_above.add_freejoint()

    tip =[]
    for dir in [-1,1]:
      x_p = dir * PLATFORM_LENGTH
      y_p = 0
      s_b = platform_bellow.add_site( name=f'{name}_{i}_down_anchor_{dir}',
                         pos=[ x_p, y_p, THICKNESS * 2],
                         size=[0.005, 0, 0])
      s_a = platform_above.add_site( name=f'{name}_{i}_up_anchor_{dir}',
                         pos=[ x_p, y_p, THICKNESS * 2],
                         size=[0.005, 0, 0])

      # Connect using a tendon
      thread = spec.add_tendon(name = f'{name}_thread_{i}_{i+1}_{dir}', limited=True,
                               range=[0, TENDON_LENGTH], width = 0.001 )
      thread.wrap_site(s_b.name)
      thread.wrap_site(s_a.name)

      tip.append(s_a)

    tip_of_lader = tip

    platform_bellow = platform_above

  # Attach ladder
  for j in range(2):
    s_w = world_sites[j]
    s_l = tip_of_lader[j]
    thread = spec.add_tendon(name = f'attachment_{j}', limited=True,
                               range=[0, TENDON_LENGTH], width = 0.001 )
    thread.wrap_site(s_w.name)
    thread.wrap_site(s_l.name)

def balustrade(parent_body, loc=[0,0,0], open=False, connection=None, theta=0, name="balustrade"):
  # RAILS
  STEP = 0.1
  RADIUS = 0.01
  HEIGHT = 0.1
  SIZE = [RADIUS, HEIGHT, 0]

  # Top Panel
  LENGTH = 0.1
  OVERHANG = 0.02
  WIDTH = 0.02
  THICKNESS = 0.01

  sites = None
  body = parent_body.add_body(pos=loc, name=name, euler=[0,0 , theta])

  for i in range(10):
    if open and i>2 and i<7:
      continue
    body.add_geom(type=mj.mjtGeom.mjGEOM_CYLINDER,
                  pos=[0,STEP * i,HEIGHT],
                  size=SIZE)
  if open:
    SIZE = [WIDTH, LENGTH+OVERHANG, THICKNESS]
    body.add_geom(type=mj.mjtGeom.mjGEOM_BOX,
                  pos=[0, LENGTH, 2 * HEIGHT],
                  size=SIZE)
    body.add_geom(type=mj.mjtGeom.mjGEOM_BOX,
                  pos=[0, STEP * 10 - (2 * LENGTH), 2 * HEIGHT],
                  size=SIZE)
    if connection == Connection.BRIDGE:
      pass
    elif connection == Connection.LADDER:
      pass
  else:
    SIZE = [WIDTH, 4.5 * LENGTH + OVERHANG, THICKNESS]
    body.add_geom(type=mj.mjtGeom.mjGEOM_BOX,
                  pos=[0, 4.5 * STEP, 2 * HEIGHT],
                  size=SIZE)

  return sites

def look_at_platform(spec, loc=[0, 0,0], openings=[False, False, False, True], name="lookout"):
  LENGTH = 0.5
  THICKNESS = 0.01
  SIZE = [LENGTH, LENGTH, THICKNESS]
  HIGHT = 0.5
  RADIUS = 0.1

  platform = spec.worldbody.add_body(pos=loc, name=name)
  platform.add_geom(
    type=mj.mjtGeom.mjGEOM_BOX,
    size=SIZE,
    pos = [0,0, HIGHT * 2]
  )

  SIZE = [RADIUS, LENGTH, 0]
  platform.add_geom(
    type=mj.mjtGeom.mjGEOM_CYLINDER,
    size=SIZE,
    pos = [0,0,HIGHT]
  )

  balustrade(parent_body=platform, open=openings[0], loc=[0.5, 0.45, HIGHT * 2 ], theta=3.14, name=f"{name}_v" )
  balustrade(parent_body=platform, open=openings[1], loc=[-0.5,0.49, HIGHT * 2 ], theta=3.14, name=f"{name}_v2" )
  balustrade(parent_body=platform, open=openings[2], loc=[0.5, 0.49, HIGHT * 2 ], theta=1.57, name=f"{name}_h" )
  balustrade(parent_body=platform, open=openings[3], loc=[0.5, -0.5, HIGHT * 2 ], theta=1.57, name=f"{name}_h2" )

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

  PLATFORM_LENGTH = 0.1
  THICKNESS = 0.001
  TENDON_LENGTH = 0.1
  num_stairs=10

  # Add site to the world
  # world_sites = []
  # for dir in [-1,1]:
  #     x_p = dir * PLATFORM_LENGTH
  #     y_p = 0
  #     s_w = spec.worldbody.add_site(  name=f'w_site_{dir}',
  #                        pos=[ x_p, y_p, TENDON_LENGTH * (num_stairs + 1)],
  #                        size=[0.005, 0, 0])
  #     world_sites.append(s_w)

  # rope_ladder(spec, world_sites, num_stairs)

  # balustrade(spec,open=False, theta = 3.14, name="v" )
  # balustrade(spec,open=False,loc=[-0.9, 0, 0 ], theta = 3.14, name="v2" )
  # balustrade(spec,open=False, theta = 1.57,name="h" )
  # balustrade(spec,open=True, loc=[0, -0.9, 0 ], theta = 1.57,name="h2" )


  look_at_platform(spec,openings=[True, False, False, True], name= "lk1")
  look_at_platform(spec, openings=[False, True, False, False], loc=[2,0,0], name="lk2")

  model = spec.compile()
  data = mj.MjData(model)


  with open("./platform.xml","w") as f:
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