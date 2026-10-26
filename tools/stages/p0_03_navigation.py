"P0-03: attach to an existing TB3 Gazebo world, localize, then start Nav2.\n\nSource the simulation's session.bash and tools/stages/p0_03_env.bash, then supply its verified world pose:\n  python3 tools/stages/p0_03_navigation.py --initial-pose X Y YAW\nNo world or goal is created. Ctrl-C shuts down this launcher's processes only."
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parents[1] / '__init__.py'))['bootstrap']()
from tools.navigation.manual_navigation import main

if __name__ == "__main__":
    raise SystemExit(main())
