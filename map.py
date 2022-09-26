import argparse
import logging
import os
from random import randint


class Map:

    def __init__(self, file=None):

        if file is not None:
            self.load(file)


    def __repr__(self):
        return "".join(["\n\t" + str(x) for x in self.table])


    def load(self, file):
        '''Loads the map from a file.'''

        # Check that the file exists
        if not os.path.isfile(file):
            logging.error(f"Cannot load file from '{file}'")
            return 

        # Load the data
        with open(file, 'r', encoding="utf-8-sig") as f:
            lines = " ".join(f.readlines())
            elements = [int(x) for x in lines.split()]

            # Load the size
            self.size_y, self.size_x  = elements[0:2]

            # Load the table as a list of lists
            self.table = []
            for y in range(self.size_y):
                r1 = 2 + self.size_x * y
                r2 = 2 + self.size_x * (y + 1)
                self.table.append(elements[r1:r2])

            # Load the initial and goal positions
            self.initial = tuple(elements[2 + self.size_y * self.size_x:5 + self.size_y * self.size_x])
            self.goal = tuple(elements[5 + self.size_y * self.size_x:8 + self.size_y * self.size_x])


    def save(self, file):
        '''Saves the map in a file.'''

        with open(file, 'w', encoding="utf-8-sig") as f:

            # Write dimensions
            f.write(f"{self.size_y} {self.size_x}")

            # Write table
            table_str = self.__repr__()
            table_str = table_str.replace('\t', "").replace('[', "").replace(']', "").replace(',', "")
            f.write(table_str)

            # Write initial and goal
            f.write(f"\n{self.initial[0]} {self.initial[1]} -1")
            f.write(f"\n{self.goal[0]} {self.goal[1]} -1")


    def generate(self, size_x, size_y, initial=None, goal=None, min_cost=1, max_cost=5, output=None):
        '''Generates a new map randomized with the parameters given.'''

        # Set the size of the table
        self.size_x, self.size_y = size_x, size_y
        
        # Establish the initial and goal positions
        self.initial = initial
        if self.initial is None:
            self.initial = (randint(0, size_y-1), randint(0, size_x-1))

        self.goal = goal
        if self.goal is None:
            self.goal = (randint(0, size_y-1), randint(0, size_x-1))

        # Generate the terrain
        self.table = [[randint(min_cost, max_cost) for x in range(size_x)] for y in range(size_y)]

        # Save the map in a file
        if output is not None:
            self.save(output)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Generate a new map.')
    parser.add_argument('--dims', type=int, default=3, help='width and height of the map')
    parser.add_argument('--initial', type=int, nargs='+', default=[0,0], help='initial position')
    parser.add_argument('--goal', type=int, nargs='+', default=[0,0], help='goal position')   
    parser.add_argument('--min-cost', type=int, default=0, help='minimum cost of the terrain') 
    parser.add_argument('--max-cost', type=int, default=5, help='maximum cost of the terrain') 
    parser.add_argument('--output', type=str, default='./output.txt', help='file to save the map') 
    args = parser.parse_args()

    map = Map()
    map.generate(args.dims, args.dims, args.initial, args.goal, args.min_cost, args.max_cost, args.output)