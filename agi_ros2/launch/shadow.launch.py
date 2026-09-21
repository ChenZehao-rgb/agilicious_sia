"""The hardware profile with an immutable prohibition on serial control frames."""
import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location('agi_runtime_launch', Path(__file__).with_name('runtime_launch.py'))
_runtime = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_runtime)


def generate_launch_description():
    return _runtime.description(shadow=True)
