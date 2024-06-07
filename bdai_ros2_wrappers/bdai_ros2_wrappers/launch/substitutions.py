# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.
from typing import List, Optional

from launch import Condition, SomeSubstitutionsType
from launch.conditions import UnlessCondition
from launch.substitutions import OrSubstitution


# TODO: when/if we move to rolling we should use the `AnySubstitution`
def not_any_substitution(conditions: List[SomeSubstitutionsType]) -> Optional[Condition]:
    """A substitution that is True if none of the conditions are substituted with True

    Args:
        conditions (list[SomeSubstitutionsType]): A list of substitutions
    Returns:
        Optional[Condition]: A substitution that is True if none of the conditions are substituted with True, if less
            than 2 conditions are provided returns none
    """
    if len(conditions) < 2:
        print("Bad number of conditions in not_any_substitution.")
        return None

    substitution = OrSubstitution(conditions[0], conditions[1])
    for cond in conditions[2:]:
        substitution = OrSubstitution(substitution, cond)

    return UnlessCondition(substitution)
