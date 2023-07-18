from typing import Dict, List, Callable, Hashable, TypeVar


T = TypeVar("T")


def find_match(
    source: List[Hashable], check_object: T, match_fn: Callable[[Hashable, T], int]
) -> List[Hashable]:
    num_matches: Dict[Hashable, int] = {}
    for obj in source:
        if obj not in num_matches:
            num_matches[obj] = 0
        property_match_count = match_fn(obj, check_object)
        num_matches[obj] += property_match_count
    selected_modes = []
    max_count = max(num_matches.values())
    for obj, count in num_matches.items():
        if count == max_count and count != 0:
            selected_modes.append(obj)
    selected_modes.sort()
    return selected_modes
