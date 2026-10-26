'Thin serial dispatcher: only starts the existing single-trial parent.'
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parent / '__init__.py'))['bootstrap']()
from tools.batch.matrix import main

if __name__ == "__main__":
    raise SystemExit(main())
