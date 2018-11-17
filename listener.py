#!/usr/bin/env python
import numpy as np
import rospy
import pygame
import time
from dijkstar import Graph, find_path

from pacman.msg import pacmanPos, bonusPos, cookiesPos, ghostsPos, game, Num
from pacman.srv import *
from std_msgs.msg import String

rx_dict =	{
  "coord0": 0,
  "ghosts": 0,
  "cookies": 0,
  "bonus": 0,
  "game": 0,
}

screen = []
mazeDrawn = []

pub = []
font = []
height = 0
width = 0
maze = []
maze_cost = []

red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
darkBlue = (0,0,128)
white = (255,255,255)
black = (0,0,0)
pink = (255,200,200)
orange = (180, 106, 2)
yellow = (255, 255, 0)
gray = (180, 180, 180)

OBSTACLE = 1000
GHOST = 500
GHOST_NB = 50000
EMPTY = 10
BONUS = 1
COOKIE = 1

size_win_x = 700
size_win_y = 480
block_size_x = 0
block_size_y = 0

world = []
pacmanpos = []
ghostspos = []
cookiespos = []
bonuspos = []
gameState = 0

goal_state = []

def worldToGrid(wx, wy):
    gx = wx-world.minX
    gy = -(wy+world.minY)
    return {'x':gx, 'y':gy}

def init_maze():
    global screen, myfont, mazeDrawn
    pygame.init()
    screen = pygame.display.set_mode((size_win_x,size_win_y))
    mazeDrawn = pygame.Surface((size_win_x,size_win_y))
    mazeDrawn.fill(black)
    pygame.font.init()
    myfont = pygame.font.SysFont('Comic Sans MS', 20)
    for y in range(height):
        for x in range(width):
            rect = pygame.Rect(x*block_size_x, y*block_size_y, block_size_x, block_size_y)
            if maze[y][x] != 1:
                pygame.draw.rect(mazeDrawn, black, rect, 0)
            else:
                pygame.draw.rect(mazeDrawn, gray, rect, 0)
    screen.blit(mazeDrawn,(0,0))
    pygame.display.update()

def posToPoint(x, y):
    px = x - world.minX
    py = -((y - world.minY) - height + 1)
    px = (px+1)*block_size_x - block_size_x/2
    py = (py+1)*block_size_y - block_size_y/2
    return (px, py)

def draw():
    global pacmanpos, ghostspos, cookiespos, bonuspos, maze_cost
    maze_cost = EMPTY*np.ones((height, width))
    screen.blit(mazeDrawn,(0,0))
    # COOKIES
    for cookie in cookiespos.cookiesPos:
        pygame.draw.circle(screen, orange,
            posToPoint(cookie.x, cookie.y),
            int(block_size_x*0.5*0.2), 0)
        cookiePos = worldToGrid(cookie.x, cookie.y)
        maze_cost[cookiePos['y']][cookiePos['x']] = COOKIE
    # BONUS
    for bonus in bonuspos.bonusPos:
        pygame.draw.circle(screen, orange,
            posToPoint(bonus.x, bonus.y),
            int(block_size_x*0.5*0.4), 0)
        bonusPos = worldToGrid(bonus.x, bonus.y)
        maze_cost[bonusPos['y']][bonusPos['x']] = BONUS
    # PACMAN
    pygame.draw.circle(screen, yellow,
        posToPoint(pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y),
        int(block_size_x*0.5*0.8), 0)
    # GHOSTS
    for i, ghost in enumerate(ghostspos.ghostsPos):
        pygame.draw.circle(screen, red,
            posToPoint(ghost.x, ghost.y),
            int(block_size_x*0.5*0.8), 0)
        pacman = worldToGrid(pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y)
        ghostPos = worldToGrid(ghost.x, ghost.y)
        if ghostspos.mode[i] == 0:
            maze_cost[ghostPos['y']][ghostPos['x']] =  GHOST
            ghost2pacman = np.abs(pacmanpos.pacmanPos.x-ghost.x) + np.abs(pacmanpos.pacmanPos.y-ghost.y)
            if ghost2pacman == 2:
                if pacmanpos.pacmanPos.y == ghost.y:
                    x = int(0.5*(pacman['x'] + ghostPos['x']))
                    maze_cost[ghostPos['y']][x] = GHOST_NB
                if pacmanpos.pacmanPos.x == ghost.x:
                    y = int(0.5*(pacman['y'] + ghostPos['y']))
                    maze_cost[y][ghostPos['x']] = GHOST_NB
                    print('manhattan')
                    print(pacman['y'])
                    print(ghostPos['y'])
                    print('')
                    print(y)
                    print(ghostPos['x'])
                    print('')
                else:
                    maze_cost[ghostPos['y']][pacman['x']] = GHOST_NB
                    maze_cost[pacman['y']][ghostPos['x']] = GHOST_NB

        else:
            maze_cost[ghostPos['y']][ghostPos['x']] =  COOKIE


    pygame.display.update()

def closest_cookie():
    global pacmanpos, cookiespos, world
    distance = {}
    pacman = (pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y)
    for cookie in cookiespos.cookiesPos:
        distance[(cookie.x, cookie.y)] = np.sqrt((pacman[0]-cookie.x)**2 + (pacman[1]-cookie.y)**2)
    next_cookie = min(distance, key=distance.get)
    #[-(cookie.y-world.minY)-1][cookie.x-world.minX]
    closest = worldToGrid(next_cookie[0], (next_cookie[1]))
    return (closest['y'], closest['x'])

def furthest_cookie():
    global pacmanpos, cookiespos, world
    distance = {}
    pacman = (pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y)
    for cookie in cookiespos.cookiesPos:
        distance[(cookie.x, cookie.y)] = np.sqrt((pacman[0]-cookie.x)**2 + (pacman[1]-cookie.y)**2)
    next_cookie = max(distance, key=distance.get)
    #[-(cookie.y-world.minY)-1][cookie.x-world.minX]
    closest = worldToGrid(next_cookie[0], (next_cookie[1]))
    return (closest['y'], closest['x'])

def closest_ghost():
    global pacmanpos, ghostspos, world
    distance = {}
    pacman = (pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y)
    for i, ghost in enumerate(ghostspos.ghostsPos):
        if ghostspos.mode[i]==1:
            distance[(ghost.x, ghost.y)] = np.sqrt((pacman[0]-ghost.x)**2 + (pacman[1]-ghost.y)**2)
    next_ghost = min(distance, key=distance.get)
    #[-(cookie.y-world.minY)-1][cookie.x-world.minX]
    closest = worldToGrid(next_ghost[0], (next_ghost[1]))
    return (closest['y'], closest['x'])

def closest_bonus():
    global pacmanpos, bonuspos, world
    distance = {}
    pacman = (pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y)
    for bonus in bonuspos.bonusPos:
        distance[(bonus.x, bonus.y)] = np.sqrt((pacman[0]-bonus.x)**2 + (pacman[1]-bonus.y)**2)
    next_bonus = min(distance, key=distance.get)
    #[-(cookie.y-world.minY)-1][cookie.x-world.minX]
    closest = worldToGrid(next_bonus[0], (next_bonus[1]))
    return (closest['y'], closest['x'])

# 0: arriba
# 1: abajo
# 2: derecha
# 3: izquierda

ghostFlag = False
ghostCounter = 0

def rx_frame():
    global pub, ghostspos, bonuspos, goal_state, pacmanpos, ghostFlag, gameState, ghostCounter

    pacman = worldToGrid(pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y)
    pacman = (pacman['y'], pacman['x'])

    ghostCounter += 1;

    if gameState==0:
        ghostCounter = 0

    if (pacman == goal_state) or (gameState == 0):
        goal_state = []
        ghostFlag = False
    if goal_state==[]:
        # if (1 in ghostspos.mode) and (ghostCounter>3):
        #     print('ghost')
        #     ghostFlag = True
        # elif len(bonuspos.bonusPos)>0:
        if len(bonuspos.bonusPos)>0:
            #print('bonus')
            goal_state = closest_bonus()
        else:
            #print('cookie')
            goal_state = closest_cookie()
    if ghostFlag:
        goal_state = closest_ghost()
    draw()
    action = solve_graph()
    pub.publish(action)


def solve_graph():
    global maze, maze_cost, pacmanpos, world, goal_state
    graph = Graph()
    for j in range(0, height):
        for i in range(0, width):
            if maze[j][i] != 1:
                #arriba
                if (maze[j-1][i] != 1) and (j-1 > 0):
                    xi = str((j,i))
                    xj = str((j-1,i))
                    c = maze_cost[j-1][i]
                    graph.add_edge(xi,xj,{'cost':c})
                    #print(xi+' - '+xj)
                #abajo
                if (maze[j+1][i] != 1) and (j+1 < height):
                    xi = str((j,i))
                    xj = str((j+1,i))
                    c = maze_cost[j+1][i]
                    graph.add_edge(xi,xj,{'cost':c})
                    #print(xi+' - '+xj)
                #derecha
                if (maze[j][i+1] != 1) and (i+1 < width):
                    xi = str((j,i))
                    xj = str((j,i+1))
                    c = maze_cost[j][i+1]
                    graph.add_edge(xi,xj,{'cost':c})
                    #print(xi+' - '+xj)
                #izquierda
                if (maze[j][i-1] != 1) and (i-1 > 0):
                    xi = str((j,i))
                    xj = str((j,i-1))
                    c = maze_cost[j][i-1]
                    graph.add_edge(xi,xj,{'cost':c})
                    #print(xi+' - '+xj)

    cost_func = lambda u, v, e, prev_e: e['cost']
    pacman = worldToGrid(pacmanpos.pacmanPos.x, pacmanpos.pacmanPos.y)
    start_state = (pacman['y'], pacman['x'])
    #start_state = (25,14)
    # goal_state = closest_bonus()
    # print(goal_state)
    #goal_state = (25,26)
    path = find_path(graph, str(start_state), str(goal_state), cost_func=cost_func)
    # print(path.nodes)
    if len(path.nodes)>1:
        next_state = eval(path.nodes[1])
        aux = (start_state[0]-next_state[0], start_state[1]-next_state[1])
    else:
        print('yuca')
        return 4
    #print(str(start_state)+'  '+str(next_state))
    if aux == (1,0):
        return 0
    if aux == (-1,0):
        return 1
    if aux == (0,-1):
        return 2
    if aux == (0,1):
        return 3

def pacmanCoord0Callback(data):
    global rx_dict, pacmanpos
    # rospy.loginfo('pacmanCoord0Callback')
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s %s", data.pacmanPos.x, data.pacmanPos.y)
    pacmanpos = data
    rx_dict['coord0'] += 1
    if len(set(list(rx_dict.values()))) == 1:
        rx_frame()

def ghostsCoordCallback(data):
    global rx_dict, ghostspos
    # rospy.loginfo('ghostsCoordCallback')
    ghostspos = data
    rx_dict['ghosts'] += 1
    if len(set(list(rx_dict.values()))) == 1:
        rx_frame()

def cookiesCoordCallback(data):
    global rx_dict, cookiespos
    # rospy.loginfo('cookiesCoordCallback')
    cookiespos = data
    rx_dict['cookies'] += 1
    if len(set(list(rx_dict.values()))) == 1:
        rx_frame()

def bonusCoordCallback(data):
    global rx_dict, bonuspos
    # rospy.loginfo('bonusCoordCallback')
    bonuspos = data
    rx_dict['bonus'] += 1
    if len(set(list(rx_dict.values()))) == 1:
        rx_frame()

def gameStateCallback(data):
    global rx_dict, gameState
    #rospy.loginfo('gameStateCallback')
    gameState = data.state
    rx_dict['game'] += 1
    if len(set(list(rx_dict.values()))) == 1:
        rx_frame()

#####################

rospy.wait_for_service('pacman_world')
try:
    serv = rospy.ServiceProxy('pacman_world', mapService)
    world = serv('Camilo')
    height = world.maxY - world.minY + 1
    width = world.maxX - world.minX + 1
    block_size_x = size_win_x / width
    block_size_y = size_win_y / height
    maze = np.zeros((height, width))
    for obs in world.obs:
        obsPos = worldToGrid(obs.x, obs.y)
        maze[obsPos['y']][obsPos['x']] = 1
except rospy.ServiceException, e:
    print "Service call failed: %s"%e

init_maze()

pub = rospy.Publisher('pacmanActions0', Num, queue_size=10)
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("pacmanCoord0", pacmanPos, pacmanCoord0Callback)
rospy.Subscriber("ghostsCoord", ghostsPos, ghostsCoordCallback)
rospy.Subscriber("cookiesCoord", cookiesPos, cookiesCoordCallback)
rospy.Subscriber("bonusCoord", bonusPos, bonusCoordCallback)
rospy.Subscriber("gameState", game, gameStateCallback)
rospy.spin()
