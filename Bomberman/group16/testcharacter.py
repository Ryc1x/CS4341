# This is necessary to find the main code
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import heapq
import math


class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here
        # 1. go through the map and find current condition
        # 2. determine state(Ex. safe, monster, bomb, monster&bomb)
        # 3. move or bomb according to state

        # Part 1
        # current position of character in a tuple: Where is the character?
        c_position = (self.x, self.y)
        a_star_move = self.a_star_search(c_position, wrld.exitcell, wrld)

        # Part 2
        # finds the next position on the A star path to exit
        state = "Default"
        if a_star_move is None:
            state = "Stuck"
        else:
            state = a_star_move[1]

        # Part 3
        if state == "Stuck":
            # we will be stuck and need bomb
            self.place_bomb()
            # will need to escape from bomb
        elif state == "has path to exit":
            # move the character by one step
            move_x = a_star_move[0][0] - c_position[0]
            move_y = a_star_move[0][1] - c_position[1]
            self.move(move_x, move_y)
        elif state == "no path to exit":
            move_x = a_star_move[0][0] - c_position[0]
            move_y = a_star_move[0][1] - c_position[1]
            self.move(move_x, move_y)
        elif state == "default":
            print("This should not happen!!!!!")
        pass

    def empty_cell_neighbors(self, node, wrld):
        # List of empty cells
        cells = []
        # Go through neighboring cells
        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if (node[0] + dx >= 0) and (node[0] + dx < wrld.width()):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if (node[1] + dy >= 0) and (node[1] + dy < wrld.height()):
                        # Is this cell safe?
                        if wrld.exit_at(node[0] + dx, node[1] + dy) or wrld.empty_at(node[0] + dx, node[1] + dy):
                            # Yes
                            if not (dx is 0 and dy is 0):
                                cells.append((node[0] + dx, node[1] + dy))
        # All done
        return cells

    # heuristic from one location to another
    # node is just a tuple with (x, y)
    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    # param: start is a start node as tuple (x, y)
    #        goal is a goal node as tuple (x, y)
    #        wrld is the current world
    # return: the best next node towards exit
    def a_star_search(self, start, goal, wrld):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next in self.empty_cell_neighbors(current, wrld):
                new_cost = cost_so_far[current] + 1  # cost from one node to its neighbor is 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        min = math.inf
        # search if we can reach goal
        for next in came_from:
            dis_to_goal = self.heuristic(next, goal)
            if dis_to_goal < min:
                min = dis_to_goal
                min_to_exit = next
        if next == goal:
            # we can reach goal
            while came_from[next] is not None:
                self.set_cell_color(next[0], next[1], Fore.RED + Back.RED)
                if came_from[next] == start:
                    return next, "has path to exit"  # next move from start to in the A* path
                next = came_from[next]

        while came_from[min_to_exit] is not None:
            self.set_cell_color(min_to_exit[0], min_to_exit[1], Fore.RED + Back.RED)
            if came_from[min_to_exit] == start:
                return min_to_exit, "no path to exit"  # next move from start to in the A* path
            min_to_exit = came_from[min_to_exit]
        # goal can not be reached
        # return a move that get close to the goal

        return None


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]
