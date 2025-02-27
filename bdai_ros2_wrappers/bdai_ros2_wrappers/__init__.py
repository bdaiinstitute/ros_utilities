# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

import importlib
import pkgutil
import sys
import warnings

warnings.simplefilter("default")
warnings.warn(
    "bdai_ros2_wrappers has been renamed to synchros2. Please use the new name synchros2 instead",
    DeprecationWarning,
    stacklevel=2,
)


def aliased_import(name, alias):
    """Import a module or a package using an alias for it.

    For packages, this function will recursively import all its subpackages and modules.
    """
    sys.modules[alias] = module = importlib.import_module(name)
    if hasattr(module, "__path__"):
        for info in pkgutil.iter_modules(module.__path__):
            aliased_import(f"{name}.{info.name}", f"{alias}.{info.name}")


aliased_import("synchros2", alias=__name__)
