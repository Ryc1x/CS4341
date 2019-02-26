# This is necessary to find the main code
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import heapq
import math

infinity = float('inf')
max_depth = 5  # number of depth for expectimax search


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
        (threaten, escape_x, escape_y) = self.threatens(c_position, wrld)

        # Part 2
        # finds the next position on the A star path to exit
        state = "Default"
        if threaten:
            state = "Escape"
        elif a_star_move is None:
            state = "Stuck"
        else:
            state = a_star_move[1]  # can be "has path to exit" or "no path to exit"

        # Part 3
        if state == "Escape":
            print("move to:", escape_x - c_position[0], escape_y - c_position[1])
            self.move(escape_x - c_position[0], escape_y - c_position[1])
        elif state == "Stuck":
            # we will be stuck and need bomb
<<<<<<< HEAD
            if not self.any_explosion(wrld):
                self.place_bomb()
            # will need to escape from bomb
=======
            self.place_bomb()  # will need to escape from bomb
>>>>>>> 9809b95f81ac933f39bce9e6bcf4d4f51f9ad1af
        elif state == "has path to exit":
            # move the character by one step
            move_x = a_star_move[0][0] - c_position[0]
            move_y = a_star_move[0][1] - c_position[1]
            self.move(move_x, move_y)
        elif state == "no path to exit":
            move_x = a_star_move[0][0] - c_position[0]
            move_y = a_star_move[0][1] - c_position[1]
            self.move(move_x, move_y)
        elif state == "Default":
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

    def any_explosion(self, wrld):
        for x in range(0, wrld.width()):
            for y in range (0, wrld.height()):
                if (wrld.explosion_at(x, y)):
                    return True
        return False

    def threatens(self, node, wrld):
        # Go through neighboring cells
<<<<<<< HEAD
        for dx in range(-3,4):
            # Avoid out-of-bounds access
            x = node[0] + dx
            if (x >= 0) and (x < wrld.width()):
                for dy in range(-3,4):
=======
        for dx in range(-2, 3):
            # Avoid out-of-bounds access
            x = node[0] + dx
            if (x >= 0) and (x < wrld.width()):
                for dy in range(-2, 3):
>>>>>>> 9809b95f81ac933f39bce9e6bcf4d4f51f9ad1af
                    y = node[1] + dy
                    # Avoid out-of-bounds access
                    if (y >= 0) and (y < wrld.height()):
                        # If the cell is not safe, rate it really low
                        self.set_cell_color(x, y, Fore.GREEN + Back.GREEN)
                        if wrld.monsters_at(x, y) or wrld.bomb_at(x,y):
                            print("Threatened")
<<<<<<< HEAD
                            (esc_x, esc_y) = max(self.empty_cell_neighbors(node, wrld), key= lambda n: self.heuristic(n,(x,y)))
=======
                            (esc_x, esc_y) = max(self.empty_cell_neighbors(node, wrld),
                                                 key=lambda n: self.heuristic(n, (x, y)))
                            print(
                                sorted(self.empty_cell_neighbors(node, wrld), key=lambda n: self.heuristic(n, (x, y))))
>>>>>>> 9809b95f81ac933f39bce9e6bcf4d4f51f9ad1af

                            print(esc_x, esc_y)
                            return (True, esc_x, esc_y)
        # All done
        return (False, 0, 0)

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

    # def value(s)
    #   if s is a max node return maxValue(s)
    #   if s is an exp node return expValue(s)
    #   if s is a terminal node return evaluation(s)

    # max pseudocode:
    # def maxValue(s)
    #     values = [value(s’) for s’ in successors(s)] return max(values)
    def expectimax_c(self, wrld, events, depth):
        # go through the event list to see if the wrld is terminated
        # Event.tpe: the type of the event. It is one of Event.BOMB_HIT_WALL,
        # Event.BOMB_HIT_MONSTER, Event.BOMB_HIT_CHARACTER,
        # Event.CHARACTER_KILLED_BY_MONSTER, Event.CHARACTER_FOUND_EXIT.
        for event in events:
            if event.tpe == event.BOMB_HIT_CHARACTER or event.tpe == event.CHARACTER_KILLED_BY_MONSTER:
                # character is dead so worst evaluation
                return -infinity
            elif event.tpe == event.CHARACTER_FOUND_EXIT:
                # character is winning so best evaluation
                return infinity
            elif depth >= max_depth:
                # reached searching depth, evaluate the wrld
                # TODO: evaluation function used here, evaluation function can include events if needed
                return self.evaluation(wrld)

        v = -infinity
        c = next(iter(wrld.characters().values()))  # get the character in the wrld

        # Go through the possible 9-moves of the character
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x + dx >= 0) and (c.x + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bound indexing
                    if (c.y + dy >= 0) and (c.y + dy < wrld.height()):
                        # No need to check impossible moves
                        if not wrld.wall_at(c.x + dx, c.y + dy):
                            # Set move in wrld
                            c.move(dx, dy)
                            # Get new world
                            (new_wrld, new_events) = wrld.next()
                            # TODO: do something with newworld and events
                            v = max(v, self.expectimax_m(new_wrld, new_events, depth + 1))
        return v

    # expect pseudocode:
    # def expValue(s)
    #     values = [value(s’) for s’ in successors(s)]
    #     weights = [probability(s, s’) for s’ in successors(s)]
    #     return expectation(values, weights)
    # param: wlrd is a senseworld object which contains events
    def expectimax_m(self, wrld, events, depth):
        for event in events:
            if event.tpe == event.BOMB_HIT_CHARACTER or event.tpe == event.CHARACTER_KILLED_BY_MONSTER:
                # character is dead so worst evaluation
                return -infinity
            elif event.tpe == event.CHARACTER_FOUND_EXIT:
                # character is winning so best evaluation
                return infinity
            elif depth >= max_depth:
                # reached searching depth, evaluate the wrld
                # TODO: evaluation function used here
                return self.evaluation(wrld)

        v = infinity
        sum_v = 0
        m = next(iter(wrld.monsters().values()))

        # record all possible number of moves
        n = 0

        # Go through the possible 8-moves of the monster
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (m.x + dx >= 0) and (m.x + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (m.y + dy >= 0) and (m.y + dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(m.x + dx, m.y + dy):
                                # Set move in wrld
                                m.move(dx, dy)
                                # Get new world
                                (new_wrld, new_events) = wrld.next()
                                # TODO: do something with newworld and events
                                n += 1
                                sum_v += self.expectimax_c(new_wrld, new_events, depth + 1)
        return sum_v / n


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]
