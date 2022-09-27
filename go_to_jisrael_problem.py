import argparse
import logging
import sys

from search import Problem, depth_first_graph_search, breadth_first_graph_search, State, BreadthFirstGraphSearch, DepthFirstGraphSearch
from map import Map

class GoToJisraelProblem(Problem):

    def __init__(self, problem_file):

        self.map = Map(problem_file)
        self.table = self.map.table

        super().__init__(State(*self.map.initial), State(*self.map.goal))

        # print("Table: {} \nInit state: {} \nGoal state: {}".format(  # f-strings?
        #     "".join(["\n\t"+str(x) for x in self.table]),
        #     str(self.initial),
        #     str(self.goal)
        # ))

    def actions(self, state):

        #
        # DUDA, LA QUE PASABA CON LA ROTACION DEL OBJETIVO¿?¿?¿?
        #

        # Check if can move
        can_move = True
        if state[2] == 0 and state[0] == 0:                     # /\
            logging.debug(f"Cannot go up.")
            can_move = False
        
        elif state[2] == 1 and state[1] == self.map.size_x-1:   # >
            logging.debug("Cannot go right.")
            can_move = False

        elif state[2] == 2 and state[0] == self.map.size_y-1:   # \/
            logging.debug("Cannot go down.")
            can_move = False

        elif state[2] == 3 and state[1] == 0:                   # <
            logging.debug("Cannot go left.")
            can_move = False

        # Return possible actions
        if can_move:
            return ["ROTATE_L", "ROTATE_R", "MOVE"]
        else:
            return ["ROTATE_L", "ROTATE_R"]

    def result(self, state, action):
        y,x,rot = state
        if action=="ROTATE_L":
            # logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y,x,(rot-1)%4)))
            return State(y,x,(rot-1)%4)

        elif action=="ROTATE_R":
            # logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y,x,(rot+1)%4)))
            return State(y,x,(rot+1)%4)

        elif action=="MOVE":
            if rot == 0:    return State(y-1,x,0)
            elif rot == 1:  return State(y,x+1,1)
            elif rot == 2:  return State(y+1,x,2)
            elif rot == 3:  return State(y,x-1,3)

    def goal_test(self, state):
        y1,x1,_ = state
        y2,x2,_ = self.goal
        return y1 == y2 and x1 == x2

    def path_cost(self, c, state1, action, state2):
        # Revisar !!! -> NO REVISE EL ENUNCIADO BIEN
        if action=="MOVE":
            y,x,_ = state2
            return c + self.table[y][x]
        else:
            return c + 1

    def value(self, state):
        pass

    def h(self, node):
        pass


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Solve the Jisrael problem.')
    parser.add_argument('file', type=str, help='file with the terrain, the initial and the goal positions')
    parser.add_argument('--debug', default=False, action='store_true', help='true to print the search process')
    args = parser.parse_args()

    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG if args.debug else logging.INFO, format='%(message)s')

    #
    # REVIEW RESULTS, NON OPTIMAL SOLUTIONS
    #

    problem = GoToJisraelProblem(args.file)
    search = DepthFirstGraphSearch(problem)
    node, frontier, explored = search.execute()

    logging.info("#########################################")
    logging.info("PATH")
    logging.info("#########################################")
    node.print_path()

    logging.info("#########################################")
    logging.info("# ANALYTICS")
    logging.info("#########################################")
    logging.info(f"Path depth: {node.depth}")
    logging.info(f"Path cost: {node.path_cost}")
    logging.info(f"Frontier size: {len(frontier)}")
    logging.info(f"Explored list size: {len(explored)}")
