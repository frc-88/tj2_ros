from typing import Dict, List, Callable, Hashable, TypeVar


T = TypeVar("T")


def find_match(
    source: List[Hashable], check_object: T, match_fn: Callable[[Hashable, T], bool]
) -> List[Hashable]:
    num_matches: Dict[Hashable, int] = {}
    for obj in source:
        if obj not in num_matches:
            num_matches[obj] = 0
        is_match = match_fn(obj, check_object)
        num_matches[obj] += 1 if is_match else 0
    selected_modes = []
    max_count = max(num_matches.values())
    for obj, count in num_matches.items():
        if count == max_count:
            selected_modes.append(obj)
    selected_modes.sort()
    return selected_modes
