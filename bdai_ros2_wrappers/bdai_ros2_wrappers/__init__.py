# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

import importlib
import pkgutil
import sys


def aliased_import(name, alias):
    """Import a module or a package using an alias for it.

    For packages, this function will recursively import all its subpackages and modules.
    """
    sys.modules[alias] = module = importlib.import_module(name)
    if hasattr(module, "__path__"):
        for info in pkgutil.iter_modules(module.__path__):
            aliased_import(f"{name}.{info.name}", f"{alias}.{info.name}")


aliased_import("synchros2", alias=__name__)
