import mujoco as mj
import random
import numpy as np

####### Utility #######
def interpolant(t):
    return t*t*t*(t*(t*6 - 15) + 10)

def perlin(shape, res, tileable=(False, False), interpolant=interpolant):
  """Generate a 2D numpy array of perlin noise.

  Args:
      shape: The shape of the generated array (tuple of two ints).
          This must be a multple of res.
      res: The number of periods of noise to generate along each
          axis (tuple of two ints). Note shape must be a multiple of
          res.
      tileable: If the noise should be tileable along each axis
          (tuple of two bools). Defaults to (False, False).
      interpolant: The interpolation function, defaults to
          t*t*t*(t*(t*6 - 15) + 10).

  Returns:
      A numpy array of shape shape with the generated noise.

  Raises:
      ValueError: If shape is not a multiple of res.
  """
  delta = (res[0] / shape[0], res[1] / shape[1])
  d = (shape[0] // res[0], shape[1] // res[1])
  grid = np.mgrid[0:res[0]:delta[0], 0:res[1]:delta[1]]\
            .transpose(1, 2, 0) % 1
  # Gradients
  angles = 2*np.pi*np.random.rand(res[0]+1, res[1]+1)
  gradients = np.dstack((np.cos(angles), np.sin(angles)))
  if tileable[0]:
    gradients[-1,:] = gradients[0,:]
  if tileable[1]:
    gradients[:,-1] = gradients[:,0]
  gradients = gradients.repeat(d[0], 0).repeat(d[1], 1)
  g00 = gradients[    :-d[0],    :-d[1]]
  g10 = gradients[d[0]:     ,    :-d[1]]
  g01 = gradients[    :-d[0],d[1]:     ]
  g11 = gradients[d[0]:     ,d[1]:     ]
  # Ramps
  n00 = np.sum(np.dstack((grid[:,:,0]  , grid[:,:,1]  )) * g00, 2)
  n10 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1]  )) * g10, 2)
  n01 = np.sum(np.dstack((grid[:,:,0]  , grid[:,:,1]-1)) * g01, 2)
  n11 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1]-1)) * g11, 2)
  # Interpolation
  t = interpolant(grid)
  n0 = n00*(1-t[:,:,0]) + t[:,:,0]*n10
  n1 = n01*(1-t[:,:,0]) + t[:,:,0]*n11
  return np.sqrt(2)*((1-t[:,:,1])*n0 + t[:,:,1]*n1)

######################
def floating_platform_for_circular_stair(spec=None, gird_loc=[0, 0, 0] ,theta=0, name='platform'):
  PLATFORM_LENGTH = 0.5
  TENDON_LENGTH = 0.5
  WIDTH = 0.12/4
  THICKNESS = 0.005
  INWARD_OFFSET = 0.0
  SIZE = [PLATFORM_LENGTH, WIDTH, THICKNESS]
  Z_OFFSET = 0.1

  RGBA_GOLD = [0.850, 0.838, 0.119, 1]

  if spec == None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Platform with sites
  gird_loc[2] += Z_OFFSET
  platform = spec.worldbody.add_body(pos=gird_loc, name=name,euler=[0, 0, theta])
  platform.add_geom(pos=[0,0,0] , size= SIZE, euler=[0,0,0],rgba=RGBA_GOLD)
  platform.add_geom(pos=[0,0.02,0], size= SIZE, euler=[0,0,0.05],rgba=RGBA_GOLD)
  platform.add_geom(pos=[0,0.05,0], size= SIZE, euler=[0,0,0.1],rgba=RGBA_GOLD)
  platform.add_geom(pos=[0,0.08,0], size= SIZE, euler=[0,0,0.15],rgba=RGBA_GOLD)
  platform.add_freejoint()

  for i, x_dir in enumerate([-1, 1]):
    for j, y_dir in enumerate([-1, 1]):
      # Add site to world
      rotation_matrix = np.array([[np.cos(-theta), -np.sin(-theta)],
                                  [np.sin(-theta), np.cos(-theta)]])
      vector = np.array([x_dir *  PLATFORM_LENGTH, y_dir * (WIDTH - INWARD_OFFSET) ])
      if i + j == 2:
        vector = np.array([x_dir * PLATFORM_LENGTH, y_dir * ( 6 * WIDTH - INWARD_OFFSET) ])
      vector = np.dot(vector , rotation_matrix)
      x_w = gird_loc[0] + vector[0]
      y_w = gird_loc[1] + vector[1]
      z_w = gird_loc[2] + TENDON_LENGTH
      # Rotate sites by theta
      spec.worldbody.add_site(name=f'{name}_hook_{x_dir}_{y_dir}',
                              pos=[ x_w, y_w, z_w],
                              size=[0.01, 0, 0])
      # Add site to platform
      x_p = x_dir *  PLATFORM_LENGTH
      y_p = y_dir * (WIDTH - INWARD_OFFSET)
      if i + j == 2:
        y_p = y_dir * (6 * WIDTH - INWARD_OFFSET)
      platform.add_site(name=f'{name}_anchor_{x_dir}_{y_dir}',
                        pos=[ x_p, y_p, THICKNESS * 2],
                        size=[0.01, 0, 0])

      # Connect tendon to sites
      thread = spec.add_tendon(name = f'{name}_thread_{x_dir}_{y_dir}', limited=True,
                               range=[0, TENDON_LENGTH], width = 0.01 )
      thread.wrap_site(f'{name}_hook_{x_dir}_{y_dir}')
      thread.wrap_site(f'{name}_anchor_{x_dir}_{y_dir}')

def floating_platform(spec=None, gird_loc=[0, 0, 0] ,theta=0, name='platform'):
  PLATFORM_LENGTH = 0.5
  WIDTH = 0.12
  INWARD_OFFSET = 0.008
  THICKNESS = 0.005
  SIZE = [ PLATFORM_LENGTH, WIDTH, THICKNESS]
  TENDON_LENGTH = 0.5
  Z_OFFSET = 0.1

  RGBA_GOLD = [0.850, 0.838, 0.119, 1]

  if spec == None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Platform with sites
  gird_loc[2] += Z_OFFSET
  platform = spec.worldbody.add_body(pos=gird_loc, name=name,euler =[0,0,theta])
  platform.add_geom(size=SIZE, rgba=RGBA_GOLD )
  platform.add_freejoint()

  for x_dir in [-1, 1]:
    for y_dir in [-1, 1]:
      # Add site to world
      rotation_matrix = np.array([[np.cos(-theta), -np.sin(-theta)],
                                  [np.sin(-theta), np.cos(-theta)]])
      vector = np.array([x_dir *  PLATFORM_LENGTH, y_dir * (WIDTH - INWARD_OFFSET) ])
      vector = np.dot(vector , rotation_matrix)

      x_w = gird_loc[0] + vector[0]
      y_w = gird_loc[1] + vector[1]
      z_w = gird_loc[2] + TENDON_LENGTH
      # Rotate sites by theta
      spec.worldbody.add_site(name=f'{name}_hook_{x_dir}_{y_dir}',
                              pos=[ x_w, y_w, z_w],
                              size=[0.01, 0, 0])
      # Add site to platform
      x_p = x_dir *  PLATFORM_LENGTH
      y_p = y_dir * (WIDTH - INWARD_OFFSET)
      platform.add_site(name=f'{name}_anchor_{x_dir}_{y_dir}',
                        pos=[ x_p, y_p, THICKNESS * 2],
                        size=[0.01, 0, 0])

      # Connect tendon to sites
      thread = spec.add_tendon(name = f'{name}_thread_{x_dir}_{y_dir}', limited=True,
                               range=[0, TENDON_LENGTH], width = 0.01 )
      thread.wrap_site(f'{name}_hook_{x_dir}_{y_dir}')
      thread.wrap_site(f'{name}_anchor_{x_dir}_{y_dir}')

def simple_suspended_stair(spec=None, grid_loc=[0, 0], num_stair=20,
                           name="simple_suspended_stair"):
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]
  SQUARE_LENGTH = 2
  THICKNESS = 0.05
  OFFSET_Y = -4/5 * SQUARE_LENGTH

  V_STEP = 0.076
  H_STEP = 0.12


  if spec == None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Plane
  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  body.add_geom(size = [SQUARE_LENGTH, SQUARE_LENGTH, THICKNESS], rgba = BROWN_RGBA )

  # Stairs
  for i in range(num_stair):
    floating_platform(spec,[grid_loc[0],
                            OFFSET_Y + grid_loc[1] + i * 2 * H_STEP,
                            i * V_STEP],
                       name =f'{name}_p_{i}')

def sin_suspended_stair(spec, grid_loc=[0, 0], num_stair=40,
                        name="sin_suspended_stair"):
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]
  SQUARE_LENGTH = 2
  THICKNESS = 0.05
  OFFSET_Y = -4/5 * SQUARE_LENGTH

  V_STEP = 0.076
  H_STEP = 0.12
  AMPLITUDE = 0.2
  FREQUENCY = 0.5


  if spec == None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Plane
  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  body.add_geom(size = [SQUARE_LENGTH,SQUARE_LENGTH, THICKNESS], rgba = BROWN_RGBA )

  for i in range(num_stair):
    x_step = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY * (i * H_STEP))
    floating_platform(spec,[grid_loc[0] + x_step,
                            OFFSET_Y + grid_loc[1] + i * 2 * H_STEP,
                            i * V_STEP],
                           name =f'{name}_p_{i}')

def circular_stairs(spec, grid_loc=[0, 0], num_stair=60, name="circular_stairs"):
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]
  SQUARE_LENGTH = 2
  THICKNESS = 0.05

  RADIUS = 1.5
  V_STEP = 0.076

  if spec == None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Plane
  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  body.add_geom(size = [SQUARE_LENGTH,SQUARE_LENGTH, THICKNESS], rgba = BROWN_RGBA )

  theta_step = 2 * np.pi / num_stair
  for i in range(num_stair):
    theta = i * theta_step
    x = grid_loc[0] + RADIUS * np.cos(theta)
    y = grid_loc[1] + RADIUS * np.sin(theta)
    z = i * V_STEP

    floating_platform_for_circular_stair(spec,[x, y, z], theta=theta , name=f'{name}_p_{i}')

def plane(spec, grid_loc=[0, 0], name='plane'):
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]
  SQUARE_LENGTH = 1
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
  SQUARE_LENGTH = 2
  THICKNESS = 0.05
  STEP = THICKNESS * 8
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]
  RED_RGBA = [0.6, 0.12, 0.15, 1.0]

  if spec == None:
    spec = mj.MjSpec()

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

    body = spec.worldbody.add_body(pos=pos, name=f'g{i}_{name}', mass=1)
    body.add_geom(type=type ,size=size, rgba=RED_RGBA)
    body.add_freejoint()


  # TODO: Scatter geoms over the surface

def stairs(spec=None, grid_loc=[0, 0] , num_stairs=4, direction=1, name='stair'):
  SQUARE_LENGTH = 2
  THICKNESS = 0.05
  STEP = THICKNESS * 2
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]

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
  SQUARE_LENGTH = 2
  THICKNESS = 0.05
  STEP = THICKNESS * 8
  SCALE = 0.1
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]
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
  for i in range(10):
    for j in range(10):
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
  SQUARE_LENGTH = 2
  THICKNESS = 0.05
  GRID_SIZE = int(SQUARE_LENGTH / THICKNESS)
  STEP = THICKNESS * 2
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]


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
      start = (random.randint(0, GRID_SIZE-2), random.randint(0, GRID_SIZE-2))
      dim = (random.randint(0, GRID_SIZE-2), random.randint(0, GRID_SIZE-2))
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
  SQUARE_LENGTH = 2
  THICKNESS = 0.05
  GRID_SIZE = int(SQUARE_LENGTH / THICKNESS)
  STEP = THICKNESS * 2
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]

  if spec == None:
    spec=mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  x_beginning = -SQUARE_LENGTH + THICKNESS
  y_beginning = SQUARE_LENGTH - THICKNESS

  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
      body.add_geom(
        pos=[x_beginning + i * STEP , y_beginning - j * STEP ,
             random.randint(-1, 1) * THICKNESS ],
        size=[THICKNESS] * 3,
        rgba = BROWN_RGBA
      )

def h_field(spec=None, grid_loc=[0, 0], name='h_field'):
  SQUARE_LENGTH = 2
  THICKNESS = 0.1
  BROWN_RGBA = [0.460, 0.362, 0.216, 1.0]

  if spec is None:
    spec = mj.MjSpec()

  size = 128
  noise = perlin((size, size), (8, 8))

  # Remap noise to 0 to 1
  noise = (noise + 1)/2
  noise -= np.min(noise)
  noise /= np.max(noise)

  # Create height field
  hfield = spec.add_hfield(name=name,
                           size=[SQUARE_LENGTH, SQUARE_LENGTH, THICKNESS, THICKNESS/10],
                           nrow=noise.shape[0],
                           ncol=noise.shape[1],
                           userdata=noise.flatten())

  body = spec.worldbody.add_body(pos=grid_loc + [0], name=name)
  body.add_geom(type=mj.mjtGeom.mjGEOM_HFIELD, hfieldname=name,
                rgba=BROWN_RGBA)

def add_tile(spec=None, grid_loc=[0, 0]):
  if spec is None:
    spec = mj.MjSpec()

  tile_type = random.randint(0,9)

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
  elif tile_type == 6:
    h_field(spec,grid_loc, name = f"h_field_{grid_loc[0]}_{grid_loc[1]}")
  elif tile_type == 7:
    simple_suspended_stair(spec,grid_loc, name = f"sss_{grid_loc[0]}_{grid_loc[1]}")
  elif tile_type == 8:
    sin_suspended_stair(spec,grid_loc, name = f"sinss_{grid_loc[0]}_{grid_loc[1]}")
  elif tile_type == 9:
    circular_stairs(spec,grid_loc, name = f"circular_s_{grid_loc[0]}_{grid_loc[1]}")
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
  spec.compiler.degree  = False

  spec.option.enableflags |= mj.mjtEnableBit.mjENBL_OVERRIDE
  spec.option.enableflags |= mj.mjtEnableBit.mjENBL_MULTICCD
  spec.option.timestep = 0.0001

  main = spec.default
  main.geom.solref = [0.001, 1]

  # Add lights
  for x in [-1, 1]:
    for y in [-1, 1]:
      spec.worldbody.add_light(pos = [x, y, 40], dir = [-x, -y, -15])

  # simple_suspended_stair(spec)
  # sin_suspended_stair(spec)
  # circular_stairs(spec)

  # plane_with_simple_geoms(spec)
  # debris(spec)

  # stairs(spec)
  # box_extrusions(spec)
  # boxy_terrain(spec)

  # h_field(spec)

  SQUARE_LENGTH = 1
  for i in range(-2,2):
    for j in range(-2,2):
      add_tile(spec=spec, grid_loc=[i * 4 * SQUARE_LENGTH, j * 4 * SQUARE_LENGTH])

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

