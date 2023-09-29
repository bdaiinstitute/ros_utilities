# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import typing


def either_or(obj: typing.Any, name: str, default: typing.Any) -> typing.Any:
    """
    Gets either an object attribute's value or a default value.

    Unlike `getattr`, callable attributes are applied as getters on `obj`.
    """
    if not hasattr(obj, name):
        return default
    value_or_getter = getattr(obj, name)
    if callable(value_or_getter):
        return value_or_getter(obj)
    return value_or_getter
