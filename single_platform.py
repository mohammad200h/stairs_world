import mujoco as mj
import random
import numpy as np

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

  platform(spec)

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