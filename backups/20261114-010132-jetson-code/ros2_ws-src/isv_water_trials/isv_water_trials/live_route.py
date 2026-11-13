"""Map the active GPS target through edits without restarting the mission."""
from difflib import SequenceMatcher


def live_goal(previous, current, index):
    old = [tuple(p) for p in previous['points']]
    new = [tuple(p) for p in current['points']]
    if type(index) is not int or not 0 <= index < len(old):
        raise ValueError('Current waypoint index is invalid')
    for operation, a, b, c, d in SequenceMatcher(a=old, b=new, autojunk=False).get_opcodes():
        if not a <= index < b:
            continue
        target = c+(index-a) if operation == 'equal' else c+min(index-a, d-c)
        return target if target < len(new) else None
    return None


def leg_signature(route, index):
    return (route['points'][index], route.get('waypoint_types', ['path']*len(route['points']))[index],
            route.get('waypoint_orbits', [None]*len(route['points']))[index],
            route['points'][index-1] if index else None)
