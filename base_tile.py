import mujoco as mj
import random


def stack(base_tile, num_stairs=4, direction = 1):
  SQUARE_LENGTH = 1 # singe square length
  THICKNESS = 0.1
  STEP = 0.1

  for i in range(num_stairs):
    # Side left
    pos = [ SQUARE_LENGTH * 1 + STEP * (1 + 2 * i),
           0,
          direction * (2 + 2 * i) * THICKNESS]
    size = [SQUARE_LENGTH * 0.1 , (2 - STEP * ( 2 * i)) * SQUARE_LENGTH, THICKNESS]
    base_tile.add_geom(pos=pos, size=size)

    # Side right
    pos = [SQUARE_LENGTH * 5 - STEP * (1 + 2 * i),
           0,
          direction * (2 + 2 * i) * THICKNESS]
    base_tile.add_geom(pos=pos, size=size)

    # Bottom
    pos =[3 * SQUARE_LENGTH ,
          -(2 - STEP * (1 + 2 * i)) * SQUARE_LENGTH,
          direction * (2 + 2 * i) * THICKNESS]
    size =  [(2 - (2* STEP * (1+ i))) * SQUARE_LENGTH,
                             SQUARE_LENGTH * 0.1,
                             THICKNESS
                            ]
    base_tile.add_geom(pos=pos, size=size)
    # Top
    pos =[3 * SQUARE_LENGTH ,
         (2 - STEP * (1 + 2 * i)) * SQUARE_LENGTH,
         direction * (2 + 2 * i) * THICKNESS]
    base_tile.add_geom(pos = pos,
                       size = size
                      )
  # TODO: Closing
  l = (2 - STEP * ( 2 * num_stairs)) * SQUARE_LENGTH
  pos = [ SQUARE_LENGTH * 1 + STEP * (2 + 2 * i) + l,
           0,
          direction * (2 + 2 * i) * THICKNESS]
  size = [l , l, THICKNESS]
  base_tile.add_geom(pos=pos, size=size)


def tile(spec=None, grid_loc=(0,0), num_stairs = 5, direction = 1):
  if spec is None:
    spec = mj.MjSpec()

  # Defaults
  main = spec.default
  main.geom.type = mj.mjtGeom.mjGEOM_BOX

  # Hollow Square
  SQUARE_LENGTH = 1 # singe square length
  THICKNESS = 0.1

  x = grid_loc[0] * SQUARE_LENGTH * 8
  y = grid_loc[1] * SQUARE_LENGTH * 8
  hallow_body = spec.worldbody.add_body(pos =[x, y, 0])

  # Left Panel
  hallow_body.add_geom(size=[SQUARE_LENGTH, 4 * SQUARE_LENGTH, THICKNESS])
  # Right Panel
  hallow_body.add_geom(pos=[6 * SQUARE_LENGTH, 0 ,0],
                       size =[SQUARE_LENGTH, 4 * SQUARE_LENGTH, THICKNESS])
  # Bottom Panel
  hallow_body.add_geom(pos=[3 * SQUARE_LENGTH, -3 * SQUARE_LENGTH ,0],
                       size =[ 2 * SQUARE_LENGTH, SQUARE_LENGTH, THICKNESS])
  # Top Panel
  hallow_body.add_geom(pos=[3 * SQUARE_LENGTH, 3 * SQUARE_LENGTH ,0],
                       size = [ 2 * SQUARE_LENGTH, SQUARE_LENGTH, THICKNESS])

  # Stairs up or down
  stack(base_tile = hallow_body,
        num_stairs = num_stairs,
        direction = direction)

  return spec


if __name__ == "__main__":


  spec =  mj.MjSpec()
  for i in range(-5,5):
    for j in range(-5,5):
      tile(spec, grid_loc=(i,j),
           num_stairs=random.randint(2, 8),
           direction=random.choice([1, -1]))


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

