import mujoco as mj
import random
import numpy as np

from scipy.signal import convolve2d

SAND_RGBA = [0.96, 0.96, 0.86, 1.0]

box = {
  "start":(0, 0),
  "dim":(2, 2)
}

def reset_grid(grid,grid_size):
  for i in range(grid_size[0]):
    for j in range(grid_size[1]):
      grid[i][j].pos[2] = 0

def there_is_a_hole(grid,grid_size):
  # Maybe replace this for loop with numpy
  # Fill gaps caused by operation if any
  # TODO:Go through each 4 neighbor if the neighbor
  # abs(dz) is bigger than 2 * 0.1 move it up or down
  for i in range(grid_size[0]):
    for j in range(grid_size[1]):
      # Looking at neighbors height
      current_tile_z = grid[i][j].pos[2]
      for k in [1,-1]:
        for l in [1,-1]:
          if i + k < grid_size[0]:
            if j + l < grid_size[1]:
              neighbors_tile_z = grid[i+k][j+l].pos[2]
              dz = get_dz(current_tile_z , neighbors_tile_z, 0.1)
              if abs(dz) > 0.25:
                return True

def get_dz(current_tile_z, neighbor_tile_z, thickness):
  threshold = 5e-3
  current  = {"top":current_tile_z + thickness, "bottom":current_tile_z - thickness}
  neighbor = {"top":neighbor_tile_z + thickness, "bottom":neighbor_tile_z - thickness}

  dist = 0
  # neighbor is underneath
  if neighbor["top"] < current["bottom"] + threshold:
    dist = abs(current["bottom"] - neighbor["top"])

  # neighbor is above
  if neighbor["bottom"] > current["top"] + threshold:
    dist = abs(neighbor["bottom"] - current["top"])

  return dist

def stairs(base_tile, num_stairs = 4, direction = 1):
  SQUARE_LENGTH = 1 # singe square length
  THICKNESS = 0.1
  STEP = 0.1

  for i in range(num_stairs):
    # Side left
    pos = [ SQUARE_LENGTH * 1 + STEP * (1 + 2 * i),
           0,
          direction * (0 + 2 * i) * THICKNESS]
    size = [SQUARE_LENGTH * 0.1 , (2 - STEP * ( 2 * i)) * SQUARE_LENGTH, THICKNESS]
    base_tile.add_geom(pos=pos, size=size, rgba = SAND_RGBA)

    # Side right
    pos = [SQUARE_LENGTH * 5 - STEP * (1 + 2 * i),
           0,
          direction * (0 + 2 * i) * THICKNESS]
    base_tile.add_geom(pos=pos, size=size, rgba = SAND_RGBA)

    # Bottom
    pos =[3 * SQUARE_LENGTH ,
          -(2 - STEP * (1 + 2 * i)) * SQUARE_LENGTH,
          direction * (0 + 2 * i) * THICKNESS]
    size =  [(2 - (2* STEP * (1+ i))) * SQUARE_LENGTH,
                             SQUARE_LENGTH * 0.1,
                             THICKNESS
                            ]
    base_tile.add_geom(pos=pos, size=size, rgba = SAND_RGBA)
    # Top
    pos =[3 * SQUARE_LENGTH ,
         (2 - STEP * (1 + 2 * i)) * SQUARE_LENGTH,
         direction * (0 + 2 * i) * THICKNESS]
    base_tile.add_geom(pos = pos, size = size , rgba = SAND_RGBA)
  # TODO: Closing
  l = (2 - STEP * ( 2 * num_stairs)) * SQUARE_LENGTH
  pos = [ SQUARE_LENGTH * 1 + STEP * (2 + 2 * i) + l,
           0,
          direction * (2 + 2 * i) * THICKNESS]
  size = [l , l, THICKNESS]
  base_tile.add_geom(pos=pos, size=size, rgba = SAND_RGBA)

def box_extrusions(base_tile, grid_size=(20, 20),complex = False):
  SQUARE_LENGTH = 0.1
  x_offset = SQUARE_LENGTH * 11
  y_offset = SQUARE_LENGTH * 19
  x_step = SQUARE_LENGTH * 2
  y_step = -SQUARE_LENGTH * 2

  # Create initial grid and store geoms ref
  grid = [[ 0 for _ in range(grid_size[0])] for _ in range(grid_size[0])]
  for i in range(grid_size[0]):
    for j in range(grid_size[1]):
      rgba = np.random.rand(4)
      rgba[3] = 1
      ref = base_tile.add_geom(
        pos = [x_offset + i * x_step ,
               y_offset + j * y_step ,
               0],
        size = [0.1, 0.1, 0.1],
        rgba = SAND_RGBA
      )
      grid[i][j] = ref

  # Create valid boxes
  num_boxes = random.randint(4, 50)
  boxes = []
  while len(boxes) < num_boxes:
    start = (random.randint(0, 18), random.randint(0, 18))
    dim = (random.randint(0, 18), random.randint(0, 18))
    # Add valid boxes
    if start[0] + dim [0] < len(grid) and start[1] + dim [1] < len(grid):
      box = {"start":start , "dim":dim}
      boxes.append(box)

  # Extrude or Cut operation using the boxes
  for b in boxes:
    operation = random.choice([1, -1])
    start = b["start"]
    dim = b["dim"]
    for i in range(start[0], dim[0]):
      for j in range(start[1], dim[1]):
        tile = grid[i][j]
        if complex:
          tile.pos[2] += operation * SQUARE_LENGTH
        else:
          tile.pos[2] = operation * SQUARE_LENGTH

def boxy_terrain(base_tile, grid_size=(20, 20)):
  SQUARE_LENGTH = 0.1
  x_step = SQUARE_LENGTH * 2
  y_step = -SQUARE_LENGTH * 2

  x_offset = SQUARE_LENGTH * 11
  y_offset = SQUARE_LENGTH * 19


  for i in range(grid_size[0]):
    for j in range(grid_size[1]):
      rgba = np.random.rand(4)
      rgba[3] = 1
      base_tile.add_geom(
        pos = [ x_offset + i * x_step ,
                y_offset + j * y_step ,
               random.randint(-1, 1) * SQUARE_LENGTH  ],
        size = [0.1, 0.1, 0.1],
        rgba = SAND_RGBA
      )

def tile(spec=None, grid_loc=(0,0), tile_num = 0):
  if spec is None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  SQUARE_LENGTH = 1 # singe square length

  x = grid_loc[0] * SQUARE_LENGTH * 4
  y = grid_loc[1] * SQUARE_LENGTH * 4
  body = spec.worldbody.add_body(pos =[x, y, 0])

  tile_type = random.randint(0,3)
  # tile_type = 4
  if tile_type == 0:
    stairs(base_tile = body, num_stairs = random.randint(2, 8), direction = 1)
  elif tile_type == 1:
    stairs(base_tile = body, num_stairs = random.randint(2, 8), direction = -1)
  elif tile_type == 2:
    boxy_terrain(base_tile = body)
  elif tile_type == 3:
    box_extrusions(base_tile = body)

  return spec

if __name__ == "__main__":

  spec =  mj.MjSpec()
  # tile(spec)
  tile_num = 0
  for i in range(-2,2):
    for j in range(-2,2):
      tile_num +=1
      tile(spec = spec, grid_loc=(i,j),tile_num = tile_num)

  pos = [-40, -20, 20 ,40]
  dir = [-0.8,-0.8, 0.8,0.8]

  # Add lights
  for x, x_dir in zip(pos, dir):
    for y, y_dir in zip(pos, dir):
      spec.worldbody.add_light(pos = [x, y, 5], dir = [-x_dir, -y_dir, -1])

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

