''
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parents[1] / '__init__.py'))['bootstrap']()
from tools.navigation.baselines import main

if __name__ == "__main__":
    raise SystemExit(main())
