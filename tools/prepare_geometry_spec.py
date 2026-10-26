'Prepare one concrete spec using archived inputs and the existing input checks.\n\nstdout remains the output path; stderr contains a JSON preparation review.\nThis does not run the distance evaluator or create an evaluation report.'
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parent / '__init__.py'))['bootstrap']()
from tools.geometry.spec_preparation import main

if __name__ == "__main__":
    raise SystemExit(main())
