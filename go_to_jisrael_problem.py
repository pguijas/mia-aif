from search import Problem, breadth_first_tree_search, depth_first_tree_search, depth_first_graph_search, breadth_first_graph_search
import logging
import sys

class GoToJisraelProblem(Problem):

    def __init__(self, problem_file):
        with open(problem_file, 'r', encoding="utf-8-sig") as file:
            # Read Data
            lines = " ".join(file.readlines())
            elements = [int(x) for x in lines.split()]
            

            # Load Table
            self.table_size_y, self.table_size_x  = elements[0:2]
            self.table = []
            for y in range(self.table_size_y):
                r1=(2+self.table_size_x*y)
                r2=(2+self.table_size_x*(y+1))
                self.table.append(elements[r1:r2])    

            # Load goal and initial states
            initial = tuple(elements[2+self.table_size_y*self.table_size_x:5+self.table_size_y*self.table_size_x])
            goal = tuple(elements[5+self.table_size_y*self.table_size_x:8+self.table_size_y*self.table_size_x])
            super().__init__(initial, goal)

            # Print Problem Data
            print("Table: {} \nInit state: {} \nGoal state: {}".format(
                "".join(["\n\t"+str(x) for x in self.table]),
                str(initial),
                str(goal)
            )) 

    def actions(self, state):

        #
        # DUDA, LA QUE PASABA CON LA ROTACION DEL OBJETIVO¿?¿?¿?
        #

        # Check if can move
        can_move = True
        if state[2] == 0 and state[0] == 0:                     # /\
            logging.debug("Cannot up " + str(state))
            can_move = False
        
        elif state[2] == 1 and state[1] == self.table_size_x-1:   # >
            logging.debug("Cannot right " + str(state))
            can_move = False

        elif state[2] == 2 and state[0] == self.table_size_y-1:   # \/
            logging.debug("Cannot down " + str(state))
            can_move = False

        elif state[2] == 3 and state[1] == 0:                   # <
            logging.debug("Cannot left " + str(state))
            can_move = False

        # Return possible actions
        if can_move:
            return ["ROTATE_L", "ROTATE_R", "MOVE"]
        else:
            return ["ROTATE_L", "ROTATE_R"]

    def result(self, state, action):

        y,x,rot = state
        if action=="ROTATE_L":
            logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y,x,(rot-1)%4)))
            return (y,x,(rot-1)%4)

        elif action=="ROTATE_R":
            logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y,x,(rot+1)%4)))
            return (y,x,(rot+1)%4)

        elif action=="MOVE":
            if rot == 0:
                logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y+1,x,0)))
                return (y-1,x,0)

            elif rot == 1:
                logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y,x+1,1)))
                return (y,x+1,1)

            elif rot == 2:
                logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y+1,x,2)))
                return (y+1,x,2)

            elif rot == 3:
                logging.debug("State: " + str(state) + " Action: " + action + " Result: " + str((y,x-1,3)))
                return (y,x-1,3)

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
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)

    #
    # REVIEW RESULTS, NON OPTIMAL SOLUTIONS
    #

    nodo = breadth_first_graph_search(GoToJisraelProblem("exampleMap1.txt"))
    print("Steps:\n\t" + "\n\t".join([str(step.action) + " -> " + str(step.state) for step in nodo.path()[1:]]))
    print("Cost: {}".format(nodo.path_cost))
    
