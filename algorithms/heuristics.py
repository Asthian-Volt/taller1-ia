from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem
import math


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    # TODO: Add your code here
    position, hasKit, pendingSystems = state

    if not hasKit:
        target = problem.kitPosition

    elif pendingSystems:
        dist_min = float("inf")

        for i in pendingSystems:
            dist = abs(position[0] - i[0]) + abs(position[1] - i[1])
            if dist < dist_min:
                dist_min = dist
        return dist_min
    else:
        target = problem.controlPosition

    return abs(position[0] - target[0]) + abs(position[1] - target[1])

def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    # TODO: Add your code here
    position, hasKit, pendingSystems = state

    if not hasKit:
        target = problem.kitPosition
    elif pendingSystems:
        dist_min = float("inf")
        for i in pendingSystems:
            dist = math.hypot(position[0] - i[0], position[1] - i[1])
            if dist < dist_min:
                dist_min = dist
        return dist_min
    else:
        target = problem.controlPosition

    return math.hypot(position[0] - target[0], position[1] - target[1])



def systemRepairHeuristic(
    state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem
):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    # TODO: Add your code here
    position, hasKit, pendingSystems = state

    if hasKit and not pendingSystems:
        return manhattanHeuristic(state, problem)
    
    cache = problem.heuristicInfo
    cache_key = (hasKit, pendingSystems)

    if cache_key not in cache:
        if not hasKit:
            nodos = (problem.kitPosition,) + pendingSystems + (problem.controlPosition,)
        else:
            nodos = pendingSystems + (problem.controlPosition,)

        if len(nodos) <= 1:
            cost = 0
        else:
            no_visited = set(nodos[1:])
            visited = {nodos[0]}
            cost = 0

            while no_visited:
                min_dist = float("inf")
                nodo_close = None
                for nodo in visited:
                    for nodo2 in no_visited:
                        dist = abs(nodo[0] - nodo2[0]) + abs(nodo[1] - nodo2[1])
                        if dist < min_dist:
                            min_dist = dist
                            nodo_close = nodo2

                cost += min_dist
                visited.add(nodo_close)
                no_visited.remove(nodo_close)

        cache[cache_key] = cost
    cost = cache[cache_key]

    if not hasKit:
        return cost + manhattanHeuristic(state, problem)
    else:
        target = pendingSystems + (problem.controlPosition,)
        min_dist_to_target = float("inf")
        for target_node in target:
            dist = abs(position[0] - target_node[0]) + abs(position[1] - target_node[1])
            if dist < min_dist_to_target:
                min_dist_to_target = dist

        return cost + min_dist_to_target
