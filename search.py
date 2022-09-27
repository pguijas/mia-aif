"""
Search (Chapters 3-4)

The way to use this code is to subclass Problem to create a class of problems,
then create problem instances and solve them with calls to the various search
functions.
"""

from collections import deque
import logging
from utils import *


class Problem:
    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError


# ______________________________________________________________________________


class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state. Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node. Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return f"({self.depth}, {self.path_cost}, {self.action}, {self.state})"

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        logging.debug(f"Expanding {self}")
        children = [self.child_node(problem, action)
                    for action in problem.actions(self.state)]
        logging.debug(f"Successor nodes: {children}")
        return children

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def print_path(self):
        """Print the nodes forming the path from the root to this node."""
        path = self.path()

        logging.info(path[0])
        for node in path[1:]:
            logging.info(node.action)
            logging.info(node)
            # print("Steps:\n\t" + "\n\t".join([str(step.action) + " -> " + str(step.state) for step in nodo.path()[1:]]))


    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)

# ______________________________________________________________________________

class State(tuple):

    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation

    def __new__(cls, x, y, orientation):
        return super(State, cls).__new__(cls, tuple((x, y, orientation)))

    def __repr__(self):
        return f"({self.x}, {self.y}, {self.format_orientation(self.orientation)})"

    def format_orientation(self, orientation):
        if      orientation == 0: return 'North'
        elif    orientation == 1: return 'East'
        elif    orientation == 2: return 'South'
        elif    orientation == 3: return 'West'

# ______________________________________________________________________________
# Uninformed Search algorithms
# ______________________________________________________________________________

def breadth_first_tree_search(problem):
    """
    [Figure 3.7]
    Search the shallowest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Repeats infinitely in case of loops.
    """

    frontier = deque([Node(problem.initial)])  # FIFO queue

    while frontier:
        node = frontier.popleft()
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))
    return None


def depth_first_tree_search(problem):
    """
    [Figure 3.7]
    Search the deepest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Repeats infinitely in case of loops.
    """

    frontier = [Node(problem.initial)]  # Stack

    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))
    return None


class GraphSearch():

    def __init__(self, problem):
        self.problem = problem
        self.frontier = []
        self.explored = set()
        self.node = None
        self.finished = False
        self.generated_nodes = 0

    def initialize(self):
        """Prepare data structures (frontier) and execute the corresponding checks."""
        raise NotImplementedError

    def execute(self):
        """Main loop of the search process."""

        # First computation of the algorithm
        self.initialize()
        if self.finished:  # the goal node is the initial node
            return self.node, [], set()

        # Loop
        iteration = 0
        while self.frontier:
            logging.debug(f"# Iteration {iteration}")
            iteration += 1

            if self.search():
                return self.node, self.frontier, self.explored

            # logging.debug(f"Frontier (size={len(self.frontier)}): {self.frontier}")
            # logging.debug(f"Explored list (size={len(self.explored)}): {self.explored}")

        logging.debug(f"Cannot find a solution.")
        return None, self.frontier, self.explored

    def search(self):
        """Execute the actions and checks of the specific search algorithm."""
        raise NotImplementedError

    def check(self, node):
        """Check if the node is a goal node."""
        if self.problem.goal_test(node.state):
            logging.debug(f"Solution found: {node}")
            self.finished = True
            return True
        return False


class DepthFirstGraphSearch(GraphSearch):

    def initialize(self):
        self.frontier = [(Node(self.problem.initial))]  # Stack

    def search(self):

        self.node = self.frontier.pop()
        if self.check(self.node):
            return True

        self.explored.add(self.node.state)
        self.frontier.extend(child for child in self.node.expand(self.problem)
                        if child.state not in self.explored and child not in self.frontier)

        return False


class BreadthFirstGraphSearch(GraphSearch):

    def initialize(self):

        self.node = Node(self.problem.initial)
        self.check(self.node)
        self.frontier = deque([self.node])

    def search(self):

        self.node = self.frontier.popleft()
        self.explored.add(self.node.state)

        for child in self.node.expand(self.problem):
            if child.state not in self.explored and child not in self.frontier:
                if self.check(child):
                    self.node = child
                    return True
                self.frontier.append(child)

        return False


class BestFirstGraphSearch(GraphSearch):

    def __init__(self, f):
        super().__init__()
        self.f = f

    def initialize(self):
        self.f = memoize(f, 'f')

        self.node = Node(problem.initial)
        self.frontier = PriorityQueue('min', f)
        self.frontier.append(node)

    def search(self):

        self.node = self.frontier.pop()
        if self.check(self.node):
            return True

        self.explored.add(self.node.state)

        for child in self.node.expand(self.problem):

            if child.state not in self.explored and child not in self.frontier:
                self.frontier.append(child)

            elif child in self.frontier and self.f(child) < self.frontier[child]:
                del self.frontier[child]
                self.frontier.append(child)

        return False


def depth_first_graph_search(problem):
    """
    [Figure 3.7]
    Search the deepest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Does not get trapped by loops.
    If two paths reach a state, only use the first one.
    """

    frontier = [(Node(problem.initial))]  # Stack
    explored = set()
    iteration = 0

    while frontier:
        logging.debug(f"# Iteration {iteration}")
        iteration += 1

        node = frontier.pop()

        if problem.goal_test(node.state):
            logging.debug(f"Solution found: {node}")
            return node, frontier, explored

        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and child not in frontier)

        logging.debug(f"Frontier: {frontier}")
        logging.debug(f"Explored list: {explored}")

    logging.debug(f"Cannot find a solution.")
    return None, frontier, explored


def breadth_first_graph_search(problem):
    """[Figure 3.11]
    Note that this function can be implemented in a
    single line as below:
    return graph_search(problem, FIFOQueue())
    """

    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node, [], set()

    frontier = deque([node])
    explored = set()
    iteration = 0

    while frontier:
        logging.debug(f"# Iteration {iteration}")
        iteration += 1

        node = frontier.popleft()
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:

                if problem.goal_test(child.state):
                    logging.debug(f"Solution found: {child}")
                    return child, frontier, explored

                frontier.append(child)
                logging.debug(f"Frontier: {frontier}")
                logging.debug(f"Explored list: {explored}")

    logging.debug(f"Cannot find a solution.")
    return None, frontier, explored


def best_first_graph_search(problem, f, display=False):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""

    f = memoize(f, 'f')
    node = Node(problem.initial)

    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    iteration = 0

    while frontier:
        logging.debug(f"# Iteration {iteration}")
        iteration += 1

        node = frontier.pop()

        if problem.goal_test(node.state):
            # if display:
            #     print(len(explored), "paths have been expanded and", len(frontier), "paths remain in the frontier")
            logging.debug(f"Solution found: {node}")
            return node, frontier, explored

        explored.add(node.state)
        for child in node.expand(problem):

            if child.state not in explored and child not in frontier:
                frontier.append(child)

            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)

            logging.debug(f"Frontier: {frontier}")
            logging.debug(f"Explored list: {explored}")

    logging.debug(f"Cannot find a solution.")
    return None, frontier, explored

# ______________________________________________________________________________
# Informed (Heuristic) Search

greedy_best_first_graph_search = best_first_graph_search

# Greedy best-first search is accomplished by specifying f(n) = h(n).

def astar_search(problem, h=None, display=False):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)


