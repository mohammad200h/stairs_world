import mujoco as mj
import random
import numpy as np

from scipy.signal import convolve2d


def stairs(spec, num_stairs=4, direction=1, name='base_tile'):
  SQUARE_LENGTH = 1
  THICKNESS = 0.1
  STEP = 0.1

  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]

  base_tile = spec.body(name)
  for i in range(num_stairs):
    # Side left
    pos = [SQUARE_LENGTH * 1 + STEP * (1 + 2 * i), 0,
           direction * (2 * i) * THICKNESS]
    size = [SQUARE_LENGTH * 0.1 , (2 - STEP * ( 2 * i)) * SQUARE_LENGTH,
            THICKNESS]
    base_tile.add_geom(pos=pos, size=size, rgba=BROWN_RGBA)
    # Side right
    pos = [SQUARE_LENGTH * 5 - STEP * (1 + 2 * i), 0,
           direction * (2 * i) * THICKNESS]
    base_tile.add_geom(pos=pos, size=size, rgba=BROWN_RGBA)
    # Bottom
    pos =[3 * SQUARE_LENGTH , -(2 - STEP * (1 + 2 * i)) * SQUARE_LENGTH,
          direction * (2 * i) * THICKNESS]
    size =  [(2 - (2* STEP * (1+ i))) * SQUARE_LENGTH, SQUARE_LENGTH * 0.1,
             THICKNESS]
    base_tile.add_geom(pos=pos, size=size, rgba=BROWN_RGBA)
    # Top
    pos =[3 * SQUARE_LENGTH , (2 - STEP * (1 + 2 * i)) * SQUARE_LENGTH,
          direction * (2 * i) * THICKNESS]
    base_tile.add_geom(pos=pos, size=size , rgba=BROWN_RGBA)
  # Closing
  l = (2 - STEP * ( 2 * num_stairs)) * SQUARE_LENGTH
  pos = [ SQUARE_LENGTH * 1 + STEP * (2 + 2 * i) + l, 0,
          direction * (2 + 2 * i) * THICKNESS]
  size = [l , l, THICKNESS]
  base_tile.add_geom(pos=pos, size=size, rgba=BROWN_RGBA)

def debris(spec,name='base_tile'):
    SQUARE_LENGTH = 1
    x_offset = SQUARE_LENGTH * 3
    y_offset = SQUARE_LENGTH * 0
    BROWN_RGBA = BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]

    # Plane
    base_tile = spec.body(name)
    base_tile.add_geom(pos=[x_offset, y_offset, 0],
                       size = [2 * SQUARE_LENGTH, 2 * SQUARE_LENGTH, 0.1],
                       rgba = BROWN_RGBA)

    # Debris
    extrusion = 0.05
    step = 0.5
    for i in range(5):
      for j in range(5):
        # x y
        drawing = np.random.normal(size = (4, 2))
        z = np.zeros((drawing.shape[0], 1))
        # Add z value
        base = np.concatenate((drawing, z), axis=1)
        # Extrude triangle
        z_extrusion = np.full((drawing.shape[0], 1), extrusion)
        top = np.concatenate((drawing, z_extrusion), axis=1)
        mesh = np.vstack((base, top))
        # Normalize
        mesh /= np.linalg.norm(mesh, axis=1, keepdims=True)

        # Create Body and add mesh to the Geom of the Body
        spec.add_mesh(name=f'd{i}_{j}_{name}', uservert=mesh.flatten())
        pos = [-1 * SQUARE_LENGTH + step * i,
               -1 * SQUARE_LENGTH + step * j,
               extrusion * 2]

        body = spec.worldbody.add_body(pos=pos, name=f'd{i}_{j}_{name}', mass=1)
        body.add_geom(type=mj.mjtGeom.mjGEOM_MESH ,meshname=f'd{i}_{j}_{name}')
        body.add_freejoint()

def box_extrusions(spec, grid_size=20, complex=False, name='base_tile'):
  # Warning! complex sometimes leads to creation of holes

  SQUARE_LENGTH = 0.1
  x_offset = SQUARE_LENGTH * 11
  y_offset = SQUARE_LENGTH * 19
  x_step = SQUARE_LENGTH * 2
  y_step = -SQUARE_LENGTH * 2

  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]

  base_tile = spec.body(name)
  # Create initial grid and store geoms ref
  grid = [[ 0 for _ in range(grid_size)] for _ in range(grid_size)]
  for i in range(grid_size):
    for j in range(grid_size):
      rgba = np.random.rand(4)
      rgba[3] = 1
      ref = base_tile.add_geom(
        pos = [x_offset + i * x_step ,
               y_offset + j * y_step ,
               0],
        size = [0.1, 0.1, 0.1],
        rgba = BROWN_RGBA
      )
      grid[i][j] = ref

  # Extrude or Cut operation using the boxes
  for _ in range(random.randint(4, 50)):
    box = None
    while box == None:
      # Create a box
      start = (random.randint(0, 18), random.randint(0, 18))
      dim = (random.randint(0, 18), random.randint(0, 18))
      # Make suer box is valid
      if start[0] + dim [0] < len(grid) and start[1] + dim [1] < len(grid):
        box = {"start":start , "dim":dim}
    # Use the box to Cut or Extrude
    operation = random.choice([1, -1])
    start = box["start"]
    dim = box["dim"]
    for i in range(start[0], dim[0]):
      for j in range(start[1], dim[1]):
        tile = grid[i][j]
        if complex:
          tile.pos[2] += operation * SQUARE_LENGTH
        else:
          tile.pos[2] = operation * SQUARE_LENGTH

def boxy_terrain(spec, grid_size=20, name='base_tile'):
  SQUARE_LENGTH = 0.1
  x_step = SQUARE_LENGTH * 2
  y_step = -SQUARE_LENGTH * 2
  x_offset = SQUARE_LENGTH * 11
  y_offset = SQUARE_LENGTH * 19

  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]

  base_tile = spec.body(name)
  for i in range(grid_size):
    for j in range(grid_size):
      rgba = np.random.rand(4)
      rgba[3] = 1
      base_tile.add_geom(
        pos = [ x_offset + i * x_step ,
                y_offset + j * y_step ,
               random.randint(-1, 1) * SQUARE_LENGTH ],
        size = [0.1, 0.1, 0.1],
        rgba = BROWN_RGBA
      )


def add_tile(spec=None, grid_loc=(0,0),name='base_tile'):
  if spec is None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  SQUARE_LENGTH = 1 # singe square length

  x = grid_loc[0] * SQUARE_LENGTH * 4
  y = grid_loc[1] * SQUARE_LENGTH * 4
  body = spec.worldbody.add_body(pos=[x, y, 0], name=name)
  center = spec.worldbody.add_body(pos=[x, y, 0])
  center.add_geom(type = mj.mjtGeom.mjGEOM_SPHERE,size=[0.1,0.1,0])

  tile_type = random.randint(0,3)
  if tile_type == 0:
    stairs(spec=spec, num_stairs=random.randint(2, 8),
           direction=1, name=name)
  elif tile_type == 1:
    stairs(spec=spec, num_stairs=random.randint(2, 8),
           direction=-1, name=name)
  elif tile_type == 2:
    boxy_terrain(spec=spec, name=name)
  elif tile_type == 3:
    box_extrusions(spec=spec, name=name)
  elif tile_type == 4:
    debris(spec=spec, name=name)

  return spec

if __name__ == "__main__":

  arena_xml = """
  <mujoco>
    <visual>
      <headlight diffuse=".5 .5 .5" specular="1 1 1"/>
      <global elevation="-10" offwidth="2048" offheight="1536"/>
      <quality shadowsize="8192"/>
    </visual>

    <asset>
      <texture type="skybox" builtin="gradient" rgb1=".5 .5 .5" rgb2="0 0 0" width="10" height="10"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="1 1 1" rgb2="1 1 1" markrgb="0 0 0" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.3"/>
    </asset>

    <worldbody>
    </worldbody>
  </mujoco>
  """

  spec = mj.MjSpec.from_string(arena_xml)


  # Add lights
  for x in [-5, 5]:
    for y in [-5, 5]:
      spec.worldbody.add_light(pos = [x, y, 20], dir = [-x, -y, -15])

  for i in range(-2,2):
    for j in range(-2,2):
      add_tile(spec=spec, grid_loc=(i, j),name=f'base_tile_{i}_{j}')

  model = spec.compile()
  data = mj.MjData(model)

  # visualization
  with mj.viewer.launch_passive(
      model = model, data = data,
      show_left_ui = False, show_right_ui = False
  ) as viewer:
    mj.mjv_defaultFreeCamera(model, viewer.cam)
    while viewer.is_running():
      mj.mj_step(model, data)
      viewer.sync()

