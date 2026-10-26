'Bounded host-only load observation around ONE existing Ubuntu runner call.'
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parents[1] / '__init__.py'))['bootstrap']()
from tools.dynamic.host_trial import main

if __name__ == "__main__":
    raise SystemExit(main())
