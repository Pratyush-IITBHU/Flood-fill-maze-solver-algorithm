'''
an algorithm to make an array like this:
8 7 6 5 4 5 6 7 8
7 6 5 4 3 4 5 6 7
6 5 4 3 2 3 4 5 6
5 4 3 2 1 2 3 4 5
4 3 2 1 0 1 2 3 4
5 4 3 2 1 2 3 4 5
6 5 4 3 2 3 4 5 6
7 6 5 4 3 4 5 6 7
8 7 6 5 4 5 6 7 8
'''
import numpy as np
#array of -1's of size 5x5
# size = 10
# maze =  [[-1 for i in range(10)] for j in range(10)]
''' -left wall 1
    -down wall 10
    -right wall 100
    -up wall 1000
'''
# walls = [[0 for i in range(10)] for j in range(10)]
# # wall between 2,2 and 3,2
# walls[2][2] += 10
# walls[3][2] += 1000
# # wall between 3,2 and 3,3
# walls[3][3] += 1
# walls[3][2] += 100

# walls[2][2] += 1
# walls[2][1] += 100

# import queue
# from genpy import DeserializationError


def make_wall(wall_array, cell1, cell2):
    ''' 
    -left wall 1
    -down wall 10
    -right wall 100
    -up wall 1000
    '''
    x1 = cell1[0]
    y1 = cell1[1]
    x2 = cell2[0]
    y2 = cell2[1]
    
    #assume cell1 is the reference point
    #right wall y2-y1 = 1
    if y2-y1 == 1:
        if((wall_array[x2][y2]//1)%10) != 1:
            wall_array[x2][y2] += 1
        if((wall_array[x1][y1]//100)%10) != 1:
            wall_array[x1][y1] += 100

    #left wall y2-y1 = -1
    if y2-y1 == -1:
        if((wall_array[x2][y2]//100)%10) != 1:
            wall_array[x2][y2] += 100
        if((wall_array[x1][y1]//1)%10) != 1:
            wall_array[x1][y1] += 1
    
    #up wall x2-x1 = -1
    if x2-x1 == -1:
        if((wall_array[x2][y2]//10)%10) != 1:
            wall_array[x2][y2] += 10
        if((wall_array[x1][y1]//1000)%10) != 1:
            wall_array[x1][y1] += 1000
    
    #down wall x2-x1 = 1
    if x2-x1 == 1:
        if((wall_array[x2][y2]//1000)%10) != 1:
            wall_array[x2][y2] += 1000
        if((wall_array[x1][y1]//10)%10) != 1:
            wall_array[x1][y1] += 10
    else:
        print("Please enter adjacent cells")
    return wall_array

# make_wall(walls, (6,2), (7,2))
# make_wall(walls, (3,2), (3,3))

# print(maze)
# print()
# print(walls)
# print()

def mod_flood_fill(maze, walls, destination_array_maze_pos):
    '''
    mod_flood_fill function
    '''
    size = len(maze)
    # print(size)
    visited = [[0 for i in range(size)] for j in range(size)]
    
    value = 0
    queue = []
    for pos in destination_array_maze_pos:
        x = pos[0]
        y = pos[1]
        # if visited[x][y] == 0:
        #     value += 1
        #     visited = flood_fill(maze, walls, visited, x, y, value)
        maze[x][y] = value
        visited[x][y] = True
        # maze[x+1][y] = value
        # visited[x+1][y] = True
        
        queue.append((x, y,value))
        # queue.append((x+1, y,value))
    while len(queue) > 0:
        row, col, value = queue.pop(0)
        value += 1

        #up case
        i = row # - 1
        j = col + 1
        if 0 <= i < size and 0 <= j < size and ((walls[row][col]//1000)) != 1 and ((walls[i][j]//10)%10) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))
        #down case
        i = row# + 1
        j = col - 1
        
        if 0 <= i < size and 0 <= j < size and ((walls[row][col]//10)%10) != 1 and ((walls[i][j]//1000)) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))

        #left case
        j = col #- 1 
        i = row - 1
        if 0 <= i < size and 0 <= j < size and ((walls[row][col]//1)%10) != 1 and ((walls[i][j]//100)%10) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))
        #right case
        j = col #+ 1 
        i = row + 1
        if 0 <= i < size and 0 <= j < size and ((walls[row][col]//100)%10) != 1 and ((walls[i][j]//1)%10) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))
    return maze
    
#take in maze, current pos and determine next pos based of flood fill algorithm
def determine_next_maze_pos(maze, walls, current_maze_pos):
    '''
        0 Right(+ x axis)
        1 Left
        2 Down
        3 Up (+ y axis)

        w/ reference to +y axis
        -left wall 1
        -down wall 10
        -right wall 100
        -up wall 1000
    '''
    x = current_maze_pos[0]
    y = current_maze_pos[1]
   
    narray = []
    # print(x,y,maze[x][y],walls[x][y])
    
    # print(y,x,maze[y][x],walls[y][x])
    # print(walls[y][x],((walls[y][x]//10)%10))
    if x+1>-1 and x+1<16:
        if (((walls[x][y]//100)%10) == 1 or ((walls[x+1][y]//1)%10) == 1)!= True:
            # print("right")
            narray.append([maze[x+1][y], x+1, y])
    if x-1>-1 and x-1<16:
        if (((walls[x][y]//1)%10) == 1 or ((walls[x-1][y]//100)%10) == 1)!= True:
            # print("left")
            narray.append([maze[x-1][y], x-1, y])
    if y+1>-1 and y+1<16:
        if (((walls[x][y]//1000)) == 1 or ((walls[x][y+1]//10)%10) == 1)!= True:
            # print("up")
            narray.append([maze[x][y+1], x, y+1])
    if y-1>-1 and y-1<16:
        if (((walls[x][y]//10)%10) == 1 or ((walls[x][y-1]//1000)) == 1)!= True:
            # print("down")
            narray.append([maze[x][y-1], x, y-1])
    print("Length ",len(narray))
    if len(narray) > 0:
        next_maze_pos=narray[0]
        for x in narray:
            if x[0]<next_maze_pos[0]:
                next_maze_pos = x
        return next_maze_pos[1],next_maze_pos[2]
    else:
        print("No next pos (maa c*ud gayi)")
        
    

# print(mod_flood_fill(maze, walls, 3, 2))

def convert_to_path(maze, walls, current_maze_pos):
    path = []
    x = current_maze_pos[0]
    y = current_maze_pos[1]
    while(maze[x][y] != 0):
        path.append([x,y])
        next_maze_pos = determine_next_maze_pos(maze, walls, [x,y])
        if next_maze_pos == None:
            print("Oopsie",path)
            break
        x = next_maze_pos[0]
        y = next_maze_pos[1]
    path.append([x,y])
    return path

if __name__ == '__main__':
    import numpy as np
    maze = [[-1 for i in range(16)] for j in range(16)]
    walls = [[0 for i in range(16)] for j in range(16)]
    for i in range(1,16):
        walls[0][i] = 100
    print(np.array(walls))
    destination = [[8,8],[8,7],[7,8],[7,7]]
    temp = np.array(mod_flood_fill(maze, walls, destination))
    print(temp)
    # print(determine_next_maze_pos(maze, walls, [0,7]))
    # print(determine_next_maze_pos(maze, walls, [1,15]))
    # print(determine_next_maze_pos(maze, walls, [5,7]))
    # print(determine_next_maze_pos(maze, walls, [6,7]))
    # print(determine_next_maze_pos(maze, walls, [1,1]))
    print(convert_to_path(temp, walls, [0,15]))
    
