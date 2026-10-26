'Optional GUI entry into the ORIGINAL run_trial.main; frozen runner stays byte-identical.\n\nOnly scoped adapter classes are substituted while that same main owns the trial.\nNo launch, recorder, action, pause, result or cleanup lifecycle is duplicated here.'
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parent / '__init__.py'))['bootstrap']()
from tools.runner.observe import main

if __name__ == "__main__":
    raise SystemExit(main())
