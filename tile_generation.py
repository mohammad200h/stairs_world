import mujoco as mj
import random
import numpy as np


def plane(spec, grid_loc=[0, 0], name='plane'):
  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]
  SQUARE_LENGTH = 1 # singe square length
  THICKNESS = 0.05

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  rgba = np.random.rand(4); rgba[3]=1
  body.add_geom(size = [SQUARE_LENGTH,SQUARE_LENGTH, THICKNESS], rgba = rgba )

  # center = spec.worldbody.add_body(pos=grid_loc + [0.2])
  # center.add_geom(type = mj.mjtGeom.mjGEOM_SPHERE,size=[0.1,0,0],rgba=BROWN_RGBA)

def plane_with_simple_geoms(spec=None, grid_loc=[0, 0], name='plane'):
  SQUARE_LENGTH = 1
  THICKNESS = 0.05
  STEP = THICKNESS * 8
  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]
  RED_RGBA = [0.6, 0.12, 0.15, 1.0]

  if spec == None:
    spec = spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  x_beginning, y_end = [-SQUARE_LENGTH + THICKNESS] * 2
  x_end, y_beginning = [SQUARE_LENGTH - THICKNESS] * 2

  # Plane
  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  body.add_geom(size = [SQUARE_LENGTH,SQUARE_LENGTH, THICKNESS], rgba = BROWN_RGBA )

  # Simple Geoms
  x_grid = np.linspace(x_beginning, x_end, 10)
  y_grid = np.linspace(y_beginning, y_end, 10)

  history = []

  for i in range(10):
    x = np.random.choice(x_grid)
    y = np.random.choice(y_grid)

    history.append([x, y])

    pos=[grid_loc[0] + x ,
           grid_loc[1] + y ,
           0.2]

    type = None
    size = None
    if random.randint(0, 1):
      type = mj.mjtGeom.mjGEOM_BOX
      size = [0.1, 0.1, 0.02]
    else:
      type = mj.mjtGeom.mjGEOM_CYLINDER
      size = [0.1, 0.02, 0]

    body = spec.worldbody.add_body(pos=pos, name=f'g{i}_{j}_{name}', mass=1)
    body.add_geom(type=type ,size=size, rgba=RED_RGBA)
    body.add_freejoint()


  # TODO: Scatter geoms over the surface

def stairs(spec=None, grid_loc=[0, 0] , num_stairs=4, direction=1, name='stair'):
  SQUARE_LENGTH = 1
  THICKNESS = 0.05
  STEP = THICKNESS * 2
  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]

  if spec == None:
    spec = spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  body = spec.worldbody.add_body(pos= grid_loc + [0], name=name)
  # Offset
  x_beginning, y_end = [-SQUARE_LENGTH + THICKNESS] * 2
  x_end, y_beginning = [SQUARE_LENGTH - THICKNESS] * 2
  # Dimension
  size_one = [THICKNESS , SQUARE_LENGTH, THICKNESS]
  size_two =  [SQUARE_LENGTH , THICKNESS, THICKNESS]
  # Geoms positions
  x_pos_l = [x_beginning, 0, THICKNESS]
  x_pos_r = [x_end, 0, THICKNESS]
  y_pos_up = [0, y_beginning, THICKNESS]
  y_pos_down = [0, y_end, THICKNESS]

  for i in range(num_stairs):
    size_one[1] = SQUARE_LENGTH - STEP * i
    size_two[0] = SQUARE_LENGTH - STEP * i

    x_pos_l[2], x_pos_r[2], y_pos_up[2], y_pos_down[2]  = [
      direction * ( THICKNESS + STEP * i)] * 4

    # Left side
    x_pos_l[0] = x_beginning + STEP * i
    body.add_geom(pos=x_pos_l, size=size_one, rgba=BROWN_RGBA)
    # Right side
    x_pos_r[0] = x_end - STEP * i
    body.add_geom(pos=x_pos_r, size=size_one, rgba=BROWN_RGBA)
    # Top
    y_pos_up[1] = y_beginning - STEP * i
    body.add_geom(pos=y_pos_up, size=size_two, rgba=BROWN_RGBA)
    # Bottom
    y_pos_down[1] = y_end + STEP * i
    body.add_geom(pos=y_pos_down, size=size_two, rgba=BROWN_RGBA)

  # Closing
  size = [SQUARE_LENGTH - STEP * num_stairs,
          SQUARE_LENGTH - STEP * num_stairs,
          THICKNESS]
  pos = [0, 0,
         direction * (THICKNESS + STEP * num_stairs)]
  body.add_geom(pos=pos, size=size, rgba=BROWN_RGBA)

def debris(spec=None, grid_loc=[0, 0] , name='debris'):
  SQUARE_LENGTH = 1
  THICKNESS = 0.05
  STEP = THICKNESS * 8
  SCALE = 0.1
  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]
  RED_RGBA = [0.6, 0.12, 0.15, 1.0]

  if spec == None:
    spec = spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX
  main.mesh.scale = np.array([SCALE]*3 , dtype = np.float64)

  x_beginning = -SQUARE_LENGTH + THICKNESS
  y_beginning = SQUARE_LENGTH - THICKNESS

  # Plane
  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  body.add_geom(size=[SQUARE_LENGTH,SQUARE_LENGTH, THICKNESS], rgba=BROWN_RGBA)

  # Debris
  for i in range(5):
    for j in range(5):
      # x y
      drawing = np.random.normal(size=(4, 2))
      # Normalize
      drawing /= np.linalg.norm(drawing, axis=1, keepdims=True)
      z = np.zeros((drawing.shape[0], 1))
      # Add z value
      base = np.concatenate((drawing, z), axis=1)
      # Extrude triangle
      z_extrusion = np.full((drawing.shape[0], 1), THICKNESS * 4)
      top = np.concatenate((drawing, z_extrusion), axis=1)
      # Combine to get mesh
      mesh = np.vstack((base, top))

      # Create body and add mesh to the geom of the body
      spec.add_mesh(name=f'd{i}_{j}_{name}', uservert=mesh.flatten())
      pos=[grid_loc[0] + x_beginning + i * STEP ,
           grid_loc[1] + y_beginning - j * STEP ,
           0.2]

      body = spec.worldbody.add_body(pos=pos, name=f'd{i}_{j}_{name}', mass=1)
      body.add_geom(type=mj.mjtGeom.mjGEOM_MESH ,meshname=f'd{i}_{j}_{name}', rgba=RED_RGBA)
      body.add_freejoint()

def box_extrusions(spec=None, grid_loc=[0, 0], complex=False, name='box_extrusions'):
  # Warning! complex sometimes leads to creation of holes
  SQUARE_LENGTH = 1
  THICKNESS = 0.05
  GRID_SIZE = int(SQUARE_LENGTH / THICKNESS)
  STEP = THICKNESS * 2
  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]


  if spec == None:
    spec = spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  x_beginning = -SQUARE_LENGTH + THICKNESS
  y_beginning = SQUARE_LENGTH - THICKNESS

  body = spec.worldbody.add_body(pos= grid_loc + [0], name=name)
  # Create initial grid and store geoms ref
  grid = [[ 0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
  for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
      ref = body.add_geom(
        pos=[x_beginning + i * STEP , y_beginning - j * STEP , 0],
        size=[THICKNESS] * 3,
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
          tile.pos[2] += operation * THICKNESS
        else:
          tile.pos[2] = operation * THICKNESS

def boxy_terrain(spec=None, grid_loc=[0, 0], name='boxy_terrain'):
  SQUARE_LENGTH = 1
  THICKNESS = 0.05
  GRID_SIZE = int(SQUARE_LENGTH / THICKNESS)
  STEP = THICKNESS * 2
  BROWN_RGBA = [0.647, 0.165, 0.165, 1.0]

  if spec == None:
    spec = spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  x_beginning = -SQUARE_LENGTH + THICKNESS
  y_beginning = SQUARE_LENGTH - THICKNESS

  body = spec.worldbody.add_body(pos= grid_loc + [0], name=name)
  for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
      body.add_geom(
        pos=[x_beginning + i * STEP , y_beginning - j * STEP ,
             random.randint(-1, 1) * THICKNESS ],
        size=[THICKNESS] * 3,
        rgba = BROWN_RGBA
      )

def add_tile(spec=None, grid_loc=[0, 0]):
  if spec is None:
    spec = mj.MjSpec()

  tile_type = random.randint(0,5)
  # tile_type = 3
  if tile_type == 0:
    plane_with_simple_geoms(spec,grid_loc, name = f"plane_{grid_loc[0]}_{grid_loc[1]}")
  elif tile_type == 1:
    stairs(spec,grid_loc, name = f"stairs_up_{grid_loc[0]}_{grid_loc[1]}",direction=1)
  elif tile_type == 2:
    stairs(spec,grid_loc, name = f"stairs_down_{grid_loc[0]}_{grid_loc[1]}",direction=-1)
  elif tile_type == 3:
    debris(spec,grid_loc, name = f"debris_{grid_loc[0]}_{grid_loc[1]}")
  elif tile_type == 4:
    box_extrusions(spec,grid_loc, name = f"box_extrusions_{grid_loc[0]}_{grid_loc[1]}")
  elif tile_type == 5:
    boxy_terrain(spec,grid_loc, name = f"boxy_terrain_{grid_loc[0]}_{grid_loc[1]}")

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
      <texture type="skybox" builtin="gradient" rgb1=".5 .5 .5" rgb2="0 0 0" width="10" height="20"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="1 1 1" rgb2="1 1 1" markrgb="0 0 0" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.3"/>
    </asset>

    <worldbody>
    </worldbody>
  </mujoco>
  """

  spec = mj.MjSpec.from_string(arena_xml)

  spec.option.enableflags |= mj.mjtEnableBit.mjENBL_OVERRIDE
  spec.option.enableflags |= mj.mjtEnableBit.mjENBL_MULTICCD
  spec.option.timestep = 0.0001

  main = spec.default
  main.geom.solref = [0.001, 1]

  # Add lights
  for x in [-1, 1]:
    for y in [-1, 1]:
      spec.worldbody.add_light(pos = [x, y, 40], dir = [-x, -y, -15])

  SQUARE_LENGTH = 1
  for i in range(-4,4):
    for j in range(-4,4):
      add_tile(spec=spec, grid_loc=[i * 2 * SQUARE_LENGTH,j * 2 * SQUARE_LENGTH])

  model = spec.compile()
  data = mj.MjData(model)

  with open("./terrain.xml","w") as f:
    f.write(spec.to_xml())

  # visualization
  with mj.viewer.launch_passive(
      model = model, data = data,
      show_left_ui = False, show_right_ui = False
  ) as viewer:
    mj.mjv_defaultFreeCamera(model, viewer.cam)
    while viewer.is_running():
      mj.mj_step(model, data)
      stability = np.isnan(data.qacc).any() or np.isinf(data.qacc).any()
      time = data.time
      viewer.sync()

