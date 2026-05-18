from maze import generate_decision_heavy_maze

TILE_SIZE = 1
WALL_HEIGHT = 1

WORLD_TEMPLATE = """#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/adept/pioneer3/protos/Pioneer3at.proto"

WorldInfo {{
  basicTimeStep 32
}}

Viewpoint {{
  orientation 0.4 0.4 -1 1.7
  position {view_x} {view_y} {view_z}
}}

TexturedBackground {{
}}

TexturedBackgroundLight {{
  texture "noon_cloudy_countryside"
  castShadows FALSE
}}

RectangleArena {{
  floorSize {arena_x} {arena_z}
  wallHeight 0.1
}}

Group {{
  children [
    DEF Walls Group {{
      children [
{walls}
      ]
    }}

    DEF DangerBlocks Group {{
      children [
{dangers}
      ]
    }}

    DEF GoalBlocks Group {{
      children [
{goals}
      ]
    }}
  ]

}}
{robots}
"""


def block(
    name: str,
    x: float,
    y: float,
    size: float,
    height: float,
    color: str,
    collidable: bool = False,
) -> str:
    bounding_object = ""

    if collidable:
        bounding_object = f"""
  boundingObject Box {{
    size {size} {size} {height}
  }}"""

    return f"""
Solid {{
  name "{name}"
  translation {x} {y} {height / 2}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor {color}
          emissiveColor 0 0 0
          ambientIntensity 1
          shininess 0
          specularColor 0 0 0
        }}
      }}
      geometry Box {{
        size {size} {size} {height}
      }}
    }}
  ]{bounding_object}
}}
"""


def wall_block(x: float, y: float, size: float) -> str:
    return block(
        name=f"wall_{x}_{y}",
        x=x,
        y=y,
        size=size,
        height=WALL_HEIGHT,
        color="0.5 0.5 0.5",
        collidable=True,
    )


def danger_block(x: float, y: float, size: float) -> str:
    return block(
        name=f"danger_{x}_{y}",
        x=x,
        y=y,
        size=size,
        height=WALL_HEIGHT,
        color="1 0 0",
        collidable=False,
    )


def goal_block(x: float, y: float, size: float) -> str:
    return block(
        name=f"goal_{x}_{y}",
        x=x,
        y=y,
        size=size,
        height=WALL_HEIGHT,
        color="0 1 0",
        collidable=False,
    )


def robot_spawn(x: float, y: float) -> str:
    return f"""
Pioneer3at {{
  name "Scout"
  translation {x} {y} 0.12
  rotation 0 0 1 0
  controller "pioneer3at"
  extensionSlot [
    Camera {{
      translation 0.15 0 0.35
      rotation 0 1 0 0.25
      width 320
      height 240
    }}
  ]
}}
"""


def maze_to_world(
    maze: list[list[str]],
    tile_size: float = TILE_SIZE,
) -> str:
    height = len(maze)
    width = len(maze[0])

    walls: list[str] = []
    dangers: list[str] = []
    goals: list[str] = []
    robots: list[str] = []

    offset_x = -((width - 1) * tile_size) / 2
    offset_y = -((height - 1) * tile_size) / 2

    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            world_x = x * tile_size + offset_x
            world_y = y * tile_size + offset_y

            if cell == "#":
                walls.append(wall_block(world_x, world_y, tile_size))

            elif cell == "R":
                dangers.append(danger_block(world_x, world_y, tile_size))

            elif cell == "G":
                goals.append(goal_block(world_x, world_y, tile_size))

            elif cell == "S":
                robots.append(robot_spawn(world_x, world_y))

    arena_x = width * tile_size
    arena_z = height * tile_size

    view_x = 0
    view_y = 50
    view_z = 50

    return WORLD_TEMPLATE.format(
        arena_x=arena_x,
        arena_z=arena_z,
        walls="\n".join(walls),
        dangers="\n".join(dangers),
        goals="\n".join(goals),
        robots="\n".join(robots),
        view_x=view_x,
        view_y=view_y,
        view_z=view_z,
    )


def save_world(
    filename: str,
    maze: list[list[str]],
    tile_size: float = TILE_SIZE,
) -> None:
    world_data = maze_to_world(
        maze=maze,
        tile_size=tile_size,
    )

    with open(filename, "w", encoding="utf-8") as file:
        file.write(world_data)
def save_maze_as_txt(filename: str, maze: list[list[str]]) -> None:
    with open(filename, "w", encoding="utf-8") as file:
        for row in maze:
            file.write("".join(row))
            file.write("\n")

if __name__ == "__main__":
    maze = generate_decision_heavy_maze(height=10, width=10)

    save_world(
        filename="Scout, Search And Rescue.wbt",
        maze=maze,
        tile_size=TILE_SIZE,
    )
    save_maze_as_txt(
        filename="current_maze.txt",
        maze=maze,
    )

    print(f"Generated Scout, Search And Rescue.wbt with tile size {TILE_SIZE:.2f}m")
