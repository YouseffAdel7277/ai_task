"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    def __init__(self, initial_state, goal_state):
        self.initial_state = initial_state
        self.goal_state = goal_state

    def getStartState(self):
        return self.initial_state

    def isGoalState(self, state):
        return state == self.goal_state

    def getSuccessors(self, state):
        successors = []
        # Your implementation to get successors here
        return successors

    def getCostOfActions(self, actions):
        total_cost = 0
        # Your implementation to calculate total cost here
        return total_cost


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

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
    "*** YOUR CODE HERE ***"
        
    start_state = problem.getStartState()
    if problem.isGoalState(start_state):
        return []

    visited = set()
    stack = util.Stack()
    stack.push((start_state, []))

    while not stack.isEmpty():
        state, path = stack.pop()
        if problem.isGoalState(state):
            return path
        if state not in visited:
            visited.add(state)
            for successor, action, _ in problem.getSuccessors(state):
                stack.push((successor, path + [action]))

    return []


def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()
    if problem.isGoalState(start_state):
        return []

    visited = set()
    queue = util.Queue()
    queue.push((start_state, []))

    while not queue.isEmpty():
        state, path = queue.pop()
        if problem.isGoalState(state):
            return path
        if state not in visited:
            visited.add(state)
            for successor, action, _ in problem.getSuccessors(state):
                queue.push((successor, path + [action]))

    return []

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    visited = set()
    fringe = util.PriorityQueue()  # Priority queue for storing fringe nodes
    start_state = (problem.getStartState(), [], 0)  # Initial state with path cost 0
    fringe.push(start_state, 0)  # Push the initial state into the fringe with priority 0

    while not fringe.isEmpty():
        state, actions, total_cost = fringe.pop()  # Get the state, actions, and total cost of the node
        if problem.isGoalState(state):  # Check if the current state is a goal state
            return actions  # Return the actions to reach the goal state
        if state not in visited:  # Check if the state has not been visited before
            visited.add(state)  # Mark the state as visited
            for next_state, action, step_cost in problem.getSuccessors(state):  # Iterate over successor states
                new_actions = actions + [action]  # Append the current action to the list of actions
                new_total_cost = total_cost + step_cost  # Calculate the new total cost
                fringe.push((next_state, new_actions, new_total_cost), new_total_cost)  # Push the successor into the fringe

    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()
    if problem.isGoalState(start_state):
        return []

    visited = set()
    priority_queue = util.PriorityQueue()
    priority_queue.push((start_state, [], 0), heuristic(start_state, problem))

    while not priority_queue.isEmpty():
        state, path, cost_so_far = priority_queue.pop()
        if problem.isGoalState(state):
            return path
        if state not in visited:
            visited.add(state)
            for successor, action, step_cost in problem.getSuccessors(state):
                new_cost = cost_so_far + step_cost
                priority_queue.push((successor, path + [action], new_cost), new_cost + heuristic(successor, problem))

    return []



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
