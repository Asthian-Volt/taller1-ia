from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyDiagnosticSearch(problem: SearchProblem):
    """
    Returns a hard-coded sequence of moves for the tinyDiagnostic layout.
    For any other station layout, the sequence of moves will be incorrect.
    """
    s = Directions.SOUTH
    e = Directions.EAST
    return [s, e, s, e, e, e, e, s, e, e, s, s, e, s, s, e, s, e, e, e, e, e, e, e]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    """
    Search the deepest nodes in the search tree first.
    """
    frontera = utils.Stack()
    frontera.push((problem.getStartState(), []))
    alcanzados = set()

    while not frontera.isEmpty():
        estado, camino = frontera.pop()

        if estado in alcanzados:
            continue
        alcanzados.add(estado)

        if problem.isGoalState(estado):
            return camino

        for sucesor, accion, _ in problem.getSuccessors(estado):
            if sucesor not in alcanzados:
                frontera.push((sucesor, camino + [accion]))

    return []

def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    inicio = problem.getStartState()
    frontera = utils.Queue()
    frontera.push((inicio, []))
    alcanzados = {inicio}

    while not frontera.isEmpty():
        estado, camino = frontera.pop()

        if problem.isGoalState(estado):
            return camino

        for sucesor, accion, _ in problem.getSuccessors(estado):
            if sucesor not in alcanzados:
                alcanzados.add(sucesor)
                frontera.push((sucesor, camino + [accion]))

    return []


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """
    inicio = problem.getStartState()
    frontera = utils.PriorityQueue()
    frontera.push((inicio, [], 0), 0)
    mejor_g = {inicio: 0}

    while not frontera.isEmpty():
        estado, camino, g = frontera.pop()

        if g > mejor_g.get(estado, float("inf")):
            continue

        if problem.isGoalState(estado):
            return camino

        for sucesor, accion, costo in problem.getSuccessors(estado):
            nuevo_g = g + costo
            if nuevo_g < mejor_g.get(sucesor, float("inf")):
                mejor_g[sucesor] = nuevo_g
                frontera.push((sucesor, camino + [accion], nuevo_g), nuevo_g)

    return []


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    inicio = problem.getStartState()
    frontera = utils.PriorityQueue()
    frontera.push((inicio, [], 0), heuristic(inicio, problem))
    mejor_g = {inicio: 0}

    while not frontera.isEmpty():
        estado, camino, g = frontera.pop()

        if g > mejor_g.get(estado, float("inf")):
            continue

        if problem.isGoalState(estado):
            return camino

        for sucesor, accion, costo in problem.getSuccessors(estado):
            nuevo_g = g + costo
            if nuevo_g < mejor_g.get(sucesor, float("inf")):
                mejor_g[sucesor] = nuevo_g
                frontera.push(
                    (sucesor, camino + [accion], nuevo_g),
                    nuevo_g + heuristic(sucesor, problem),
                )

    return []


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
