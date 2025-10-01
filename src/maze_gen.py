import copy
import random
import time
from operator import index
from unittest import case

import pygame
from pygame.locals import *
from sympy import ceiling
from sympy.codegen.fnodes import merge

class TreeNode:
    def __init__(self, data, parent=None, children=None):
        self.data = data
        self.parent = parent
        self.children = []
        if children:
            self.children = children
    def add_child(self, child):
        self.children.append(child)

def move(pos, dx=0, dy=0):
    return pos[0] + dx, pos[1] + dy

def get_common_node(node_fam_1, node_fam_2):
    for node_1 in node_fam_1:
        for node_2 in node_fam_2:
            if node_1.data == node_2.data:
                return node_1.data


class Tile:
    def __init__(self, top, left, bot, right, role):
        self.top = top
        self.left = left
        self.bot = bot
        self.right = right
        self.role = role
    
class Maze:
    def __init__(self, row, col, start, end, num_sols):
        self.row = row
        self.col = col
        self.start = start
        self.end = end
        self.num_sols = num_sols

        # initialize maze grid with walls
        self.maze = []
        coord_list = []
        for i in range(self.row):
            temp_row = []
            for j in range(self.col):
                temp_row.append(Tile(True, True, True, True, "normal"))
                coord_list.append((j,i))
            self.maze.append(temp_row)

        # initialize default starting pos for default config
        if self.start == "default":
            self.start = (0, 0)
        if self.end == "default":
            self.end = (self.row - 1, self.col - 1)

        # generate a random tile to start at and add to list of visited tiles
        visited_tiles = [self.end]
        coord_list.remove(self.end)

        solution_path = []
        branches = []
        is_sol = False

        print("Generating maze...")


        # loop until maze is complete
        complete = False
        while not complete:

            if not is_sol:

                loop_start_tile = self.start
            else:
                # generate a second random tile
                loop_start_tile = coord_list[random.randint(0, len(coord_list) - 1)]

            new_tile = loop_start_tile
            possible_moves = ["right", "down", "left", "up"]

            save_maze = copy.deepcopy(self.maze)
            curr_loop = []
            maze_history = []
            while not new_tile in visited_tiles:


                if new_tile in curr_loop:

                    while curr_loop[-1] != new_tile:
                        curr_loop.pop()
                        maze_history.pop()
                    curr_loop.pop()
                    self.maze = maze_history.pop()

                    if curr_loop:
                        prev_tile = curr_loop[-1]

                        if prev_tile[0] < new_tile[0]:
                            possible_moves = ["right", "down", "up"]
                        elif prev_tile[0] > new_tile[0]:
                            possible_moves = ["down", "left", "up"]
                        elif prev_tile[1] < new_tile[1]:
                            possible_moves = ["right", "down", "left"]
                        elif prev_tile[1] > new_tile[1]:
                            possible_moves = ["right", "left", "up"]
                    else:
                        possible_moves = ["right", "down", "left", "up"]



                curr_loop.append(new_tile)
                maze_history.append(copy.deepcopy(self.maze))

                # check for walls
                if new_tile[0] == 0:
                    possible_moves.remove("left")
                elif new_tile[0] == self.col-1:
                    possible_moves.remove("right")
                if new_tile[1] == 0:
                    possible_moves.remove("up")
                elif new_tile[1] == self.row-1:
                    possible_moves.remove("down")

                move_dir = possible_moves[random.randint(0,len(possible_moves)-1)]

                if move_dir == "right":
                    self.maze[new_tile[1]][new_tile[0]].right = False
                    new_tile = move(new_tile,dx=1)
                    self.maze[new_tile[1]][new_tile[0]].left = False
                    possible_moves = ["right", "down", "up"]
                elif move_dir == "down":
                    self.maze[new_tile[1]][new_tile[0]].bot = False
                    new_tile = move(new_tile,dy=1)
                    self.maze[new_tile[1]][new_tile[0]].top = False
                    possible_moves = ["right", "down", "left"]
                elif move_dir == "left":
                    self.maze[new_tile[1]][new_tile[0]].left = False
                    new_tile = move(new_tile,dx=-1)
                    self.maze[new_tile[1]][new_tile[0]].right = False
                    possible_moves = ["down", "left", "up"]
                else:
                    self.maze[new_tile[1]][new_tile[0]].top = False
                    new_tile = move(new_tile,dy=-1)
                    self.maze[new_tile[1]][new_tile[0]].bot = False
                    possible_moves = ["right", "left", "up"]



            if not is_sol:

                solution_path += curr_loop
                solution_path.append(self.end)

                is_sol = True

            visited_tiles += curr_loop
            for coord in curr_loop:
                coord_list.remove(coord)



            curr_loop.append(new_tile)
            # checks if this is the solution loop to skip adding to branches
            if len(visited_tiles) != len(curr_loop):
                branches.append(curr_loop)



            if len(visited_tiles) == self.row * self.col:
                complete = True


        merge_found = True
        while merge_found:
            merge_found = False
            for i, branch in enumerate(branches):
                for j in range(i+1, len(branches)):
                    if branch[0] == branches[j][-1]:
                        # merge first branch to end of second branch, remove first branch
                        branch.pop(0)
                        branches[j] += branch
                        branches.pop(i)
                        merge_found = True
                        break
                if merge_found:
                    break

        # map a pseudo tree of the maze branches
        solution_node = TreeNode(solution_path)

        maze_tree = [[solution_node]]
        layer_num = 0
        while branches:
            branches_to_remove = []
            branch_nodes = []
            for branch in branches:
                for prev_branch in maze_tree[layer_num]:
                    if branch[-1] in prev_branch.data:
                        branches_to_remove.append(branch)
                        new_node = TreeNode(branch, parent=prev_branch)
                        prev_branch.add_child(new_node)
                        branch_nodes.append(new_node)

                        break

            maze_tree.append(branch_nodes)
            for branch in branches_to_remove:
                branches.remove(branch)
            layer_num += 1


        edges = []
        for j, row in enumerate(self.maze):
            for i, tile in enumerate(row):
                if tile.right and i != self.col-1:
                    tile_1 = (i,j)
                    tile_2 = (i+1,j)
                    edges.append((tile_1,tile_2))

                if tile.bot and j != self.row-1:
                    tile_1 = (i,j)
                    tile_2 = (i,j+1)
                    edges.append((tile_1,tile_2))

        remove_candidates = []

        for tile_1, tile_2 in edges:
            tile_1_node = None
            tile_2_node = None

            for layer in maze_tree:
                for branch in layer:
                    if tile_1 in branch.data:
                        tile_1_node = branch
                    if tile_2 in branch.data:
                        tile_2_node = branch
                    if tile_1_node and tile_2_node:
                        break
                if tile_1_node and tile_2_node:
                    break


            tile_1_node_fam = [copy.copy(tile_1_node)]
            while tile_1_node.parent:
                tile_1_node = tile_1_node.parent
                tile_1_node_fam.append(copy.copy(tile_1_node))
            tile_2_node_fam = [copy.copy(tile_2_node)]
            while tile_2_node.parent:
                tile_2_node = tile_2_node.parent
                tile_2_node_fam.append(copy.copy(tile_2_node))




            common_node = get_common_node(tile_1_node_fam, tile_2_node_fam)

            if common_node != solution_path:
                continue

            distance_1 = 0
            distance_2 = 0

            if tile_1 in common_node:
                common_node_index_1 = common_node.index(tile_1)
            else:

                travel_tile = tile_1
                for node in tile_1_node_fam:
                    if node.data == common_node:
                        common_node_index_1 = common_node.index(travel_tile)
                    else:
                        distance_1 += len(node.data) - node.data.index(travel_tile) - 1
                        travel_tile = node.data[-1]
            if tile_2 in common_node:
                common_node_index_2 = common_node.index(tile_2)
            else:
                travel_tile = tile_2
                for node in tile_2_node_fam:
                    if node.data == common_node:
                        common_node_index_2 = common_node.index(travel_tile)
                    else:
                        distance_2 += len(node.data) - node.data.index(travel_tile) - 1
                        travel_tile = node.data[-1]

            distance = abs(common_node_index_1-common_node_index_2) + distance_1 + distance_2
            if distance > (self.row + self.col)/2:
                remove_candidates.append((tile_1, tile_2))


        remove_candidates.sort()

        if num_sols > len(remove_candidates):
            num_sols = len(remove_candidates)
        for i in range(num_sols):
            rand_index = ceiling(random.randrange(0, (len(remove_candidates)-1) ** 2) ** 0.5)
            edge = remove_candidates[rand_index]
            remove_candidates.remove(edge)

            tile_1_x = edge[0][0]
            tile_1_y = edge[0][1]
            tile_2_x = edge[1][0]
            tile_2_y = edge[1][1]

            if edge[0][1] == edge[1][1]:
                self.maze[tile_1_y][tile_1_x].right = False
                self.maze[tile_2_y][tile_2_x].left = False
            else:
                self.maze[tile_1_y][tile_1_x].bot = False
                self.maze[tile_2_y][tile_2_x].top = False
        print("Maze complete!")



    def disp_maze(self, sol=None, path=None):

        if path is None:
            path = []
        if sol is None:
            sol = []
        pygame.init()
        pygame.display.init()
        window = pygame.display.set_mode((600, 600))
        window.fill((0, 0, 0))
        white = (255, 255, 255)

        run = True
        # Creating a while loop
        while run:

            # Iterating over all the events received from
            # pygame.event.get()
            for event in pygame.event.get():

                # If the type of the event is quit
                # then setting the run variable to false
                if event.type == QUIT:
                    run = False

            startx = 100
            starty = 100
            side_length = 400/self.row

            y = starty
            for j, row in enumerate(self.maze):
                x = startx
                for i, tile in enumerate(row):
                    if tile.top:
                        pygame.draw.line(window, white, [x,y], [x+side_length,y], 2)
                    if tile.left and not (i==0 and j==0):
                        pygame.draw.line(window, white, [x,y], [x,y+side_length], 2)
                    if tile.bot:
                        pygame.draw.line(window, white, [x+side_length,y+side_length], [x,y+side_length], 2)
                    if tile.right and not (i==self.row-1 and j==self.col-1):
                        pygame.draw.line(window, white, [x+side_length,y+side_length], [x+side_length,y], 2)
                    x += side_length
                y += side_length

            sol_path = copy.copy(sol)
            lines_path = []
            if sol_path:
                if len(sol_path) != len(sol)+2:
                    sol_path.insert(0,(-1,0))
                    sol_path.append((sol[-1][0]+1,sol[-1][1]))
                for tile in sol_path:
                    lines_path.append((100+tile[0]*side_length+side_length/2,100+tile[1]*side_length+side_length/2))
            if path:
                for tile in path:
                    lines_path.append((100+tile[0]*side_length+side_length/2,100+tile[1]*side_length+side_length/2))

            if lines_path:
                pygame.draw.lines(window, (255,0,0), False, lines_path, 4)


            # Draws the surface object to the screen.
            pygame.display.update()

    def hug_left(self, maze):
        #Coordinates for the exit sequence
        end=maze.end

        #list map pairing
        path_list=list()
        path_list.append(maze.start)
        path_map=map(tuple,path_list)

        #current row, column pair
        cur_row, cur_col=maze.start
        next_row=0
        next_col=0

        #starting tile
        cur_Tile=maze.maze[cur_row][cur_col]

        #intializing random next tile
        next_Tile=Tile(True,True,True,True,True)

        #intializing left right up and down
        left=cur_Tile.left
        right=cur_Tile.right
        top=cur_Tile.top
        bottom=cur_Tile.bot

        #can update this to give initial orientation
        orientation=0

        while (cur_row,cur_col)!=end:
            print(cur_row,cur_col)
            print(not left, not top, not right, not bottom)
            print(cur_Tile)
            if not left:
                #go left
                #run into an issue when directios begin to change
                next_row,next_col=maze.move_direction("Left", orientation, cur_row, cur_col)
                orientation+=90
            elif not top:
                #go straight
                next_row, next_col=maze.move_direction("Up", orientation, cur_row, cur_col)
            elif not right:
                #go right
                next_row, next_col=maze.move_direction("Right", orientation, cur_row, cur_col)
                orientation+=270
            elif not bottom:
                next_row, next_col=maze.move_direction("Down", orientation, cur_row, cur_col)
                orientation+=180
                #turn around
            else:
                print("not working")
                break
            #Set next_tile to new row,col pair
            next_Tile = maze.maze[next_row][next_col]

            #update the direction the robot is facing
            left,top,right,bottom = maze.update_direction(orientation,next_Tile)

            if (next_row,next_col) in path_map:
                #if we are back tracking remove the paths that weve already stepped on
                path_list.remove((cur_row,cur_col))

            #add new tile to list and map and then move from cur to next
            path_list.append((next_row,next_col))
            path_map = map(tuple, path_list)

            cur_row = next_row
            cur_col = next_col
            cur_Tile=next_Tile

        print(path_list)
        print("Maze Finished!")
        return path_list

    def move_direction(self,direction, orientation, row, col):
        orientation=orientation%360
        print("orientation:", orientation)
        print("direction:", direction)
        match direction:
            case "Left":
                match orientation:
                    case 0:
                        #decrease column
                        col-=1
                    case 90:
                        #increase row
                        row+=1
                    case 180:
                        #increase col
                        col+=1
                    case 270:
                        #decrease row
                        row-=1
            case "Right":
                match orientation:
                    case 0:
                        # increase col
                        col += 1
                    case 90:
                        # decrease row
                        row -= 1
                    case 180:
                        # decrease col
                        col -= 1
                    case 270:
                        # increase row
                        row += 1
            case "Up":
                match orientation:
                    case 0:
                        # decrease row
                        row -= 1
                    case 90:
                        # decrease col
                        col -= 1
                    case 180:
                        # increase row
                        row += 1
                    case 270:
                        # increase col
                        col += 1

            case "Down":
                match orientation:
                    case 0:
                        # increase row
                        row += 1
                    case 90:
                        # increase col
                        col += 1
                    case 180:
                        # decrease row
                        row -= 1
                    case 270:
                        # decrease col
                        col -= 1
        return row,col

    def update_direction(self,direction, next_Tile):
        #mod by 360 to return back to 0-270
        direction=direction%360

        #base case is facing up
        left = next_Tile.left
        top = next_Tile.top
        right = next_Tile.right
        bottom = next_Tile.bot

        #switching the orientation of the robot

        match direction:
            #facing left
            case 90:
                left=next_Tile.bot
                top=next_Tile.left
                right=next_Tile.top
                bottom=next_Tile.right
            #facing down
            case 180:
                left = next_Tile.right
                top = next_Tile.bot
                right = next_Tile.left
                bottom = next_Tile.top
            #facing right
            case 270:
                left = next_Tile.top
                top = next_Tile.right
                right = next_Tile.bot
                bottom = next_Tile.left
        return left,top,right,bottom


if __name__ == "__main__":
    my_maze = Maze(10, 10, "default", "default", 0)

    path_list=my_maze.hug_left(my_maze)
    my_maze.disp_maze(path=path_list)

