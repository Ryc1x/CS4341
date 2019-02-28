# This is necessary to find the main code
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import heapq
import math

infinity = math.inf
max_depth = 2  # number of depth for expectimax search


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

        # (threaten, escape_x, escape_y) = self.threatens(c_position, wrld) # currently not using threaten function

        # if a_star_move returns none, there is no way to go closer to exit
        if a_star_move is not None:
            self.bestmove = a_star_move[0]
        print(self.bestmove)

        # Place the bomb as soon as I can
        self.place_bomb()
        print("character at:", c_position)
        # Take the move based on expectimax
        (dx,dy) = self.expectimax_action(wrld, 0)
        print("taking move:", dx, dy)
        self.move(dx,dy)

        return

        ########### NOTE ############
        # FOLLOWING LINES INGNORED! #
        #############################

        # Part 2
        # finds the next position on the A star path to exit
        state = "Default"

        # modified threaten to be bomb thread and monster thread
        # TODO: Escape bomb when bomb thread
        # expectimax when monster threaten
        mthreaten = self.monster_threaten(wrld, 3)[0]
        if mthreaten and self.bomb_threaten(wrld):
            state = "Monster and bomb"
        elif mthreaten:
            state = "Monster"
        elif self.bomb_threaten(wrld):
            state = "Bomb"
        elif a_star_move is None:
            state = "Stuck"
        else:
            state = a_star_move[1]  # can be "has path to exit" or "no path to exit"

        # Part 3
        if state == "Stuck":
            # we will be stuck and need bomb
            if not self.any_explosion(wrld):
                self.place_bomb()  # will need to escape from bomb
        else:
            (dx,dy) = self.expectimax_action(wrld, 0)
            print("taking move:(", dx, dy, ")")
            self.move(dx,dy)
        return
        ## ignored case for now

        if state == "Monster and bomb":
            print("Monster and bomb near me!!!!!!!!!!!!")
            self.move(0, 0)
        elif state == "Monster":
            print("Monster !!!!!!!!!!!!!!")
            (dx,dy) = self.expectimax_action(wrld, 0)
            self.move(dx,dy)
        elif state == "Bomb":
            print("Bomb!!!!!!!!!!!!!!!!!!!!")
            self.move(0, 0)
        elif state == "Stuck":
            # we will be stuck and need bomb
            if not self.any_explosion(wrld):
                self.place_bomb()  # will need to escape from bomb
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
            for y in range(0, wrld.height()):
                if (wrld.explosion_at(x, y)):
                    return True
        return False

    def threatens(self, node, wrld):
        # Go through neighboring cells

        for dx in range(-3, 4):
            # Avoid out-of-bounds access
            x = node[0] + dx
            if (x >= 0) and (x < wrld.width()):
                for dy in range(-3, 4):
                    y = node[1] + dy
                    # Avoid out-of-bounds access
                    if (y >= 0) and (y < wrld.height()):
                        # If the cell is not safe, rate it really low
                        # self.set_cell_color(x, y, Fore.GREEN + Back.GREEN)
                        if wrld.monsters_at(x, y) or wrld.bomb_at(x, y):
                            # print("Threatened")
                            (esc_x, esc_y) = max(self.empty_cell_neighbors(node, wrld),
                                                 key=lambda n: self.heuristic(n, (x, y)))

                            # print(esc_x, esc_y)
                            return (True, esc_x, esc_y)
        # All done
        return (False, 0, 0)

    # param:
    #
    # Checking monster in choosing size grid
    # return a tuple with (true/false, array)
    # returned array is contains tuple (x, y)
    def monster_threaten(self, wrld, size):
        c = next(iter(wrld.characters.values()))
        c = c[0]
        x = c.x
        y = c.y
        print("In this world character is at ", x, ", ", y)
        monster_list = []
        # Go through neighboring cells

        for dx in range(-size, size + 1):
            # Avoid out-of-bounds access
            if (x + dx >= 0) and (x + dx < wrld.width()):
                for dy in range(-size, size + 1):
                    # Avoid out-of-bounds access
                    if (y + dy >= 0) and (y + dy < wrld.height()):
                        if wrld.monsters_at(x + dx, y + dy):
                            monster_list.append((x + dx, y + dy))
        print(" Monster is at: ", monster_list)
        if len(monster_list) > 0:
            return True, monster_list
        else:
            return False, monster_list

    # Checking bomb in straight line
    def bomb_threaten(self, wrld):
        c = next(iter(wrld.characters.values()))
        c = c[0]
        x = c.x
        y = c.y
        for dx in range(-5, 6):
            # Avoid out-of-bounds access
            if (x + dx >= 0) and (x + dx < wrld.width()):
                if wrld.bomb_at(x + dx, y):
                    return True
        for dy in range(-5, 6):
            # Avoid out-of-bounds access
            if (y + dy >= 0) and (y + dy < wrld.height()):
                if wrld.bomb_at(x, y + dy):
                    return True
        return False

    def explosion_threaten(self, wrld):
        c = next(iter(wrld.characters.values()))
        if wrld.explosion_at(c.x, c.y):
            return True
        else:
            return False

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

    def expectimax_action(self, wrld, depth):
        # go through the event list to see if the wrld is terminated
        # Event.tpe: the type of the event. It is one of Event.BOMB_HIT_WALL,
        # Event.BOMB_HIT_MONSTER, Event.BOMB_HIT_CHARACTER,
        # Event.CHARACTER_KILLED_BY_MONSTER, Event.CHARACTER_FOUND_EXIT.
        action = (0, 0)
        max_v = -infinity

        c = next(iter(wrld.characters.values()))  # get the character position in the wrld
        c = c[0]
        # TODO here need fix bug when no monster, will be killed by bomb
        if len(wrld.monsters.values()) == 0:
            return (self.bestmove[0]-self.x, self.bestmove[1]-self.y)
        # TODO add support for 0/1/2 monsters
        m = next(iter(wrld.monsters.values()))  # get the monster position in the wrld
        m = m[0]

        # Go through the possible 9-moves of the character
        # Loop through delta x
        for dx_c in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x + dx_c >= 0) and (c.x + dx_c < wrld.width()):
                # Loop through delta y
                for dy_c in [-1, 0, 1]:
                    # Avoid out-of-bound indexing
                    if (c.y + dy_c >= 0) and (c.y + dy_c < wrld.height()):
                        # No need to check impossible moves
                        if not wrld.wall_at(c.x + dx_c, c.y + dy_c):
                            # Set move in wrld
                            c.move(dx_c, dy_c)

                            n = 0  # number of options for monster actions
                            sum_v = 0  # sum of all monster actions value

                            # Go through the possible 8-moves of the monster
                            # Loop through delta x
                            for dx_m in [-1, 0, 1]:
                                # Avoid out-of-bound indexing
                                if (m.x + dx_m >= 0) and (m.x + dx_m < wrld.width()):
                                    # Loop through delta y
                                    for dy_m in [-1, 0, 1]:
                                        # Make sure the monster is moving
                                        if (dx_m != 0) or (dy_m != 0):
                                            # Avoid out-of-bound indexing
                                            if (m.y + dy_m >= 0) and (m.y + dy_m < wrld.height()):
                                                # No need to check impossible moves
                                                if not wrld.wall_at(m.x + dx_m, m.y + dy_m):
                                                    # Set move in wrld
                                                    m.move(dx_m, dy_m)
                                                    # Get new world
                                                    (new_wrld, new_events) = wrld.next()
                                                    # do something with new world and events
                                                    n += 1  # number of options for monster movements
                                                    sum_v += self.expectimax(new_wrld, new_events, depth + 1)
                            dist_to_best = self.heuristic((c.x + dx_c, c.y + dy_c), self.bestmove)
                            expect = sum_v / n - dist_to_best  # TODO: adding a weight to the dist_to_best
                            print("action:", dx_c, dy_c)
                            print("value:", expect)
                            if expect > max_v:
                                action = (dx_c, dy_c)
                                max_v = expect
        print("max action:", action)
        print("max value:", max_v)
        return action

    def expectimax(self, wrld, events, depth):
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
        if depth >= max_depth:
            # reached searching depth, evaluate the wrld
            return self.evaluation(wrld)

        expect_values = []
        c = next(iter(wrld.characters.values()))  # get the character position in the wrld
        c = c[0]  # c was a list
        # TODO here need fix bug when no monster, will be killed by bomb
        if len(wrld.monsters.values()) == 0: 
            return infinity
        # TODO add support for 0/1/2 monsters
        m = next(iter(wrld.monsters.values()))  # get the monster position in the wrld
        m = m[0]  # m was a list

        # Go through the possible 9-moves of the character
        # Loop through delta x
        for dx_c in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x + dx_c >= 0) and (c.x + dx_c < wrld.width()):
                # Loop through delta y
                for dy_c in [-1, 0, 1]:
                    # Avoid out-of-bound indexing
                    if (c.y + dy_c >= 0) and (c.y + dy_c < wrld.height()):
                        # No need to check impossible moves
                        if not wrld.wall_at(c.x + dx_c, c.y + dy_c):
                            # Set move in wrld
                            c.move(dx_c, dy_c)

                            n = 0  # number of options for monster actions
                            sum_v = 0  # sum of all monster actions value

                            # Go through the possible 8-moves of the monster
                            # Loop through delta x
                            for dx_m in [-1, 0, 1]:
                                # Avoid out-of-bound indexing
                                if (m.x + dx_m >= 0) and (m.x + dx_m < wrld.width()):
                                    # Loop through delta y
                                    for dy_m in [-1, 0, 1]:
                                        # Make sure the monster is moving
                                        if (dx_m != 0) or (dy_m != 0):
                                            # Avoid out-of-bound indexing
                                            if (m.y + dy_m >= 0) and (m.y + dy_m < wrld.height()):
                                                # No need to check impossible moves
                                                if not wrld.wall_at(m.x + dx_m, m.y + dy_m):
                                                    # Set move in wrld
                                                    m.move(dx_m, dy_m)
                                                    # Get new world
                                                    (new_wrld, new_events) = wrld.next()
                                                    # do something with new world and events
                                                    n += 1  # number of options for monster movements
                                                    sum_v += self.expectimax(new_wrld, new_events, depth + 1)
                            expect_values.append(sum_v / n)
        v = max(expect_values)
        return v

    # param: wrld
    # def:
    #   wrld
    # return: evaluation value
    def evaluation(self, wrld):
        c = next(iter(wrld.characters.values()))
        c = c[0]
        return self.eval1(wrld, c)
        # NOTE NOT USED EVAL FUNCTIONS
        # return self.evaluation_bomb(wrld, c) + self.evaluation_monster_easy(wrld, c) #+ self.evaluation_explosion(wrld, c)
        # + self.evaluation_straight_distance(wrld, c)

    # Evaluate world based on:
    #   1. distance to monster
    #   2. 
    def eval1(self, wrld, c):
        if len(wrld.monsters.values()) == 0: return 0
        mlist = next(iter(wrld.monsters.values()))
        score = 0
        for m in mlist:
            distx = abs(c.x - m.x)
            disty = abs(c.y - m.y)
            if distx <= 2 and disty <= 2:
                return -10000
            # higher -> stay more distance away (usually: 100~1000)
            sensitivity = 500 
            score -= sensitivity / (distx+disty)**2
        return score

    # param: wrld, c
    # def:
    #   wrld,
    #   c,
    # return: evaluation value between character and bomb
    def evaluation_bomb(self, wrld, c):
        x = c.x
        y = c.y
        # Check bomb in 5 grids distance (because bomb has 4 grids explosion extension)
        bomb_score = 0

        # Check Vertical Position
        for dy in (-5, 6):
            # Avoid out-of-bound indexing
            if (y + dy >= 0) and (y + dy < wrld.height()):
                if wrld.bomb_at(x, y + dy):
                    bomb_score -= 100

        # Check Horizontal Position
        for dx in (-5, 6):
            # Avoid out-of-bound indexing
            if (x + dx >= 0) and (x + dx < wrld.width()):
                if wrld.bomb_at(x + dx, y):
                    bomb_score -= 100

        return bomb_score

    # param: wrld, c
    # def:
    #   wrld,
    #   c,
    # return: evaluation value between character and explosion area
    def evaluation_explosion(self, wrld, c):
        x = c.x
        y = c.y

        if wrld.explosion_at(x, y):
            return -2000
        return 0

    # param: wrld, c
    # def:
    #   wrld,
    #   c,
    # return: evaluation value between character and monster
    def evaluation_monster_hard(self, wrld, c):
        x = c.x
        y = c.y
        # Check monster 2 grids around character
        monster_score = 0
        for dx in (-2, 2):
            # Avoid out-of-bound indexing
            if (x + dx >= 0) and (x + dx < wrld.width()):
                # Loop through delta y
                for dy in (-2, 2):
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (y + dy >= 0) and (y + dy < wrld.height()):
                            if wrld.monsters_at(x + dx, y + dy):
                                # Monster at 2 grids away
                                if abs(max(dx, dy)) == 2:
                                    # 1.
                                    # Monster is 2 grids away at a diagonal direction:
                                    # m 0 0
                                    # 0 0 0
                                    # 0 0 c
                                    # 2 moves till death
                                    if (abs(dx) == 2) and (abs(dy) == 2):
                                        m_score_1 = 0

                                        # m 0 0
                                        # 0 w 0
                                        # 0 0 c
                                        # 3 moves till death
                                        if wrld.wall_at(x + int(dx / 2), y + int(dy / 2)):

                                            # m w 0
                                            # 0 w 0
                                            # 0 w c
                                            #
                                            # m 0 0
                                            # w w w
                                            # 0 0 c
                                            # 4 moves till death
                                            if ((wrld.wall_at(x + int(dx / 2), c.y) and wrld.wall_at(x + int(dx / 2),
                                                                                                     y + dy)) or (
                                                    wrld.wall_at(x, y + int(dy / 2) and wrld.wall_at(x + dx, y + int(
                                                        dy / 2))))):
                                                m_score_1 -= 100
                                            else:
                                                m_score_1 -= 100

                                        if m_score_1 == 0:
                                            m_score_1 = -1000

                                        monster_score += m_score_1

                                    # 2.
                                    # Monster is 2 grids away in a straight line:
                                    # m 0 c
                                    #
                                    # m
                                    # 0
                                    # c
                                    # 2 moves till death
                                    elif dx == 0 or dy == 0:
                                        m_score_2 = 0

                                        # 0 w 0
                                        # m w c
                                        # 0 w 0
                                        # 4 moves till death
                                        if abs(dx) == 2:
                                            # check boundary
                                            if y + 1 <= wrld.height() and y - 1 >= 0:
                                                if wrld.wall_at(x + int(dx / 2), y + 1) and wrld.wall_at(
                                                        x + int(dx / 2), y) and wrld.wall_at(x + int(dx / 2), y - 1):
                                                    m_score_2 -= 100
                                            elif y + 1 > wrld.height():
                                                if wrld.wall_at(x + int(dx / 2), y) and wrld.wall_at(x + int(dx / 2),
                                                                                                     y - 1):
                                                    m_score_2 -= 100
                                            else:
                                                if wrld.wall_at(x + int(dx / 2), y + 1) and wrld.wall_at(
                                                        x + int(dx / 2), y):
                                                    m_score_2 -= 100

                                        # 0 m 0
                                        # w w w
                                        # 0 c 0
                                        # 4 moves till death
                                        elif abs(dy) == 2:
                                            # check boundary
                                            if x + 1 <= wrld.width() and x - 1 >= 0:
                                                if wrld.wall_at(x - 1, y + int(dy / 2)) and wrld.wall_at(x, y + int(
                                                        dy / 2)) and wrld.wall_at(x + 1, int(dy / 2)):
                                                    m_score_2 -= 100
                                            elif x + 1 > wrld.width():
                                                if wrld.wall_at(x - 1, y + int(dy / 2)) and wrld.wall_at(x, y + int(
                                                        dy / 2)):
                                                    m_score_2 -= 100
                                            else:
                                                if wrld.wall_at(x, y + int(dy / 2)) and wrld.wall_at(x + 1,
                                                                                                     int(dy / 2)):
                                                    m_score_2 -= 100

                                        if m_score_2 == 0:
                                            m_score_2 = -1000

                                        monster_score += m_score_2

                                    # 3.
                                    # Monster is 2 grids away in one line difference in one of x,y direction:
                                    # m 0 0
                                    # 0 0 c
                                    else:
                                        m_score_3 = 0

                                        # m w 0
                                        # 0 w c
                                        # 3 moves till death
                                        if abs(dx) == 2:
                                            if wrld.wall_at(x + int(dx / 2), y) and wrld.wall_at(x + int(dx / 2),
                                                                                                 y + dy):
                                                monster_score -= 200

                                        # m 0
                                        # w w
                                        # 0 c
                                        elif abs(dy) == 2:
                                            if wrld.wall_at(x, y + int(dy / 2)) and wrld.wall_at(x + dx, int(dy / 2)):
                                                monster_score -= 200

                                    if m_score_3 == 0:
                                        m_score_3 = -1000
                                    monster_score += m_score_3
                                # Monster at 1 grid away
                                else:
                                    monster_score -= 2000

        return monster_score

    # param: wrld, c
    # def:
    #   wrld,
    #   c,
    # return: evaluation value between character and monster (simplified)
    def evaluation_monster_easy(self, wrld, c):
        x = c.x
        y = c.y
        # Check monster 2 grids around character
        monster_score = 0
        for dx in (-2, 2):
            # Avoid out-of-bound indexing
            if (x + dx >= 0) and (x + dx < wrld.width()):
                # Loop through delta y
                for dy in (-2, 2):
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (y + dy >= 0) and (y + dy < wrld.height()):
                            if wrld.monsters_at(x + dx, y + dy):
                                # Monster at 2 grids away
                                if abs(max(dx, dy)) == 2:
                                    monster_score -= 200
                                else:
                                    monster_score -= 1000

        return monster_score

    # param: wrld, c
    # def:
    #   wrld,
    #   c,
    # return evaluation value between character and exit cell
    def evaluation_straight_distance(self, wrld, c):
        x = c.x
        y = c.y
        score = 0
        distance = self.heuristic((x, y), wrld.exitcell)

        # closer character with the exit cell, higher the score
        if distance > 10:
            score += 5
        else:
            score += (10 - distance) * 20
        return score


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]
