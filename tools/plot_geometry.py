'Render one immutable offline report. PNG creation is exclusive.'
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parent / '__init__.py'))['bootstrap']()
from tools.geometry.plotting import main

if __name__ == "__main__":
    raise SystemExit(main())
