"""CMR tools package and one shared bootstrap for direct file entry points."""
from pathlib import Path
import importlib.util
import sys

PROJECT_ROOT = Path(__file__).resolve().parent.parent

def bootstrap():
    """Load this package by its source path, without changing sys.path."""
    path = Path(__file__).resolve()
    existing = sys.modules.get('tools')
    if existing is not None:
        if Path(existing.__file__).resolve() != path:
            raise ImportError('Another tools package is already loaded')
        return existing
    spec = importlib.util.spec_from_file_location(
        'tools', path, submodule_search_locations=[str(path.parent)])
    package = importlib.util.module_from_spec(spec)
    sys.modules['tools'] = package
    spec.loader.exec_module(package)
    return package
