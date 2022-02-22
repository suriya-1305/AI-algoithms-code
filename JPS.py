from copy import copy, deepcopy;
import math
import time

USE_DIAGONALS = True;
DIAGONAL_COST = math.sqrt(2);

char_map = [];
path_map = [];
state_map = [];
dest_location = (0, 0);

def goal_function(state):
    return state.map_rep == 'd';

def h_manhattan_distance(loc):
    return (abs(loc[0] - dest_location[0])) + (abs(loc[1] - dest_location[1]));

def h_euclidean_distance(loc):
    return math.sqrt(abs(loc[0] - dest_location[0])**2) + (abs(loc[1] - dest_location[1])**2);

def refresh_state_map():from mapinfo import MapInfo
from scipy.spatial import cKDTree
import math

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def find_min_f(l, f_score):
    r = min(l, key=lambda item: f_score[item])
    i = l.index(r)
    return i

def neighbor_nodes(x, roadmap):
    plist = [(x[0] - 1, x[1] - 1), (x[0] - 1, x[1]), (x[0] - 1, x[1] + 1), (x[0], x[1] + 1), (x[0] + 1, x[1] + 1), (x[0] + 1, x[1]), (x[0] + 1, x[1] - 1), (x[0], x[1] - 1)]
    for p in plist:
        if 0 < p[0] < roadmap.width and 0 < p[1] < roadmap.height and p not in roadmap.obstacle:
            yield p

def reconstruct_path(camefrom, current_node):
    if current_node in camefrom:
        p = reconstruct_path(camefrom, camefrom[current_node])
        return p + [current_node]
    else:
        return [current_node]

def middle_point(p1, p2):
    x1 = float(p1[0])
    y1 = float(p1[1])
    x2 = float(p2[0])
    y2 = float(p2[1])
    return ((x1+x2)/2, (y1+y2)/2)

def line_of_sight(p1, p2, okdtree):
    points = [p1, p2]
    L = distance(p1, p2)
    # generate points in line p1-p2, make sure number of points more than distance p1 to p2
    while len(points) < L*1.3:
        i = 0
        j = 1
        while j < len(points):
            points.insert(j, middle_point(points[i], points[j]))
            i += 2
            j += 2
    # judge each point collide obstacle or not
    for p in points:
        d, _ = okdtree.query(p)
        if d < 1.0:
            return False
    return True

def a_star_planning(map_info, display=False):
    closedlist = []
    openlist = []
    camefrom = dict()
    g_score = {map_info.start: 0}
    h_score = {map_info.start: distance(map_info.start, map_info.end)}
    f_score = {map_info.start: h_score[map_info.start]}
    openlist.append(map_info.start)
    okdtree = cKDTree(map_info.obstacle)
    while openlist:
        min_i = find_min_f(openlist, f_score)
        x = openlist.pop(min_i)
        if x == map_info.end:
            return reconstruct_path(camefrom, map_info.end)
        closedlist.append(x)
        if display:
            map_info.close = x
        for y in neighbor_nodes(x, map_info):
            if y in closedlist:
                continue
            tentative_g_score = g_score[x] + distance(x, y)
            if y not in openlist:
                tentative_is_better = True
            elif tentative_g_score < g_score[y]:
                tentative_is_better = True
            else:
                tentative_is_better = False
            if tentative_is_better:
                if x != map_info.start and line_of_sight(y, camefrom[x], okdtree):
                    camefrom[y] = camefrom[x]
                    g_score[y] = g_score[camefrom[x]] + distance(camefrom[x], y)
                    h_score[y] = distance(y, map_info.end)
                    f_score[y] = g_score[y] + h_score[y]
                else:
                    camefrom[y] = x
                    g_score[y] = tentative_g_score
                    h_score[y] = distance(y, map_info.end)
                    f_score[y] = g_score[y] + h_score[y]
                openlist.append(y)
                if display:
                    map_info.open = y
    return []

if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    m.start = (10, 10)
    m.end = (50, 30)
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    raw_input('enter to start ...')
    m.path = a_star_planning(m, display=True)
    m.wait_close()
    for i in range(0, len(state_map)):
        for j in range(0, len(state_map[i])):
            state_map[i][j].reset_state();

class State:
    def __init__(self, map_rep, loc, map_size):
        self.map_rep = map_rep; 
        self.map_size = map_size;
        self.key = 0;
        self.path_cost = 0;
        self.time_expanded = 0; 
        self.loc = loc;
        self.successors = [];
        self.accessed_diagonally = False;
        self.JPS_pruned = False; 

    def __str__(self):
        return "( id:" + str(self.get_hash_id()) + ", key: " + str(self.key) + " )";
 
    def __repr__(self):
        return "( id:" + str(self.get_hash_id()) + ", key: " + str(self.key) + " )";

    def reset_state(self):
        self.successors = [];
        self.accessed_diagonally = False;
        self.key = 0;
        self.path_cost = 0;
        self.time_expanded = 0;
        self.JPS_pruned = False;
    def calculate_f(self, path_cost):
        g = 1;
        if self.accessed_diagonally:
            g = DIAGONAL_COST;
        if(self.map_rep == '@' or self.map_rep == 'T'):
            g = self.map_size[0] * self.map_size[1] * 1000; #simulate infinity for walls.
        if(self.map_rep == 'w'):
            g += 3;
        if(self.map_rep == 'm'):
            g += 5;

        g = g + path_cost;
        self.path_cost += g;
        h = heuristic_func(self.loc);
        self.key = g + h;

    def get_successors(self):
        if(len(self.successors) == 0):
            loc = self.loc;
            locations = [];
            locations.append((loc[0] - 1, loc[1], False));
            locations.append((loc[0], loc[1] - 1, False));
            locations.append((loc[0] + 1, loc[1], False));
            locations.append((loc[0], loc[1] + 1, False));
            if USE_DIAGONALS:
                locations.append((loc[0] - 1, loc[1] - 1, True));
                locations.append((loc[0] - 1, loc[1] + 1, True));
                locations.append((loc[0] + 1, loc[1] - 1, True));
                locations.append((loc[0] + 1, loc[1] + 1, True));
            for (x, y, diagonal) in locations:
                if(self._in_range(x, y, False)):
                    self.successors.append(state_map[x][y]);
                    state_map[x][y].accessed_diagonally = diagonal;
        return self.successors;

    def get_successors_JPS(self):
        if(len(self.successors) == 0):
            loc = self.loc;
            locations = [];
            if USE_DIAGONALS:
                locations.append((loc[0] - 1, loc[1] - 1, True));
                locations.append((loc[0] - 1, loc[1] + 1, True));
                locations.append((loc[0] + 1, loc[1] - 1, True));
                locations.append((loc[0] + 1, loc[1] + 1, True));
            locations.append((loc[0] - 1, loc[1], False));
            locations.append((loc[0], loc[1] - 1, False));
            locations.append((loc[0] + 1, loc[1], False));
            locations.append((loc[0], loc[1] + 1, False));
            for (x, y, diagonal) in locations:
                if(self._in_range(x, y, True)):
                    path_cost = 1;
                    if(diagonal):
                        path_cost = DIAGONAL_COST;
                    jump_state = self._jump_successor(self, x - self.loc[0], y - self.loc[1], diagonal, path_cost);
                    if(jump_state != None):
                        jump_state.JPS_pruned = True;
                        self.successors.append(jump_state);
                        state_map[x][y].accessed_diagonally = diagonal;
        return self.successors;
    def _jump_successor(self, direction_state, dx, dy, diagonal, curr_path_cost):
        currX = direction_state.loc[0];
        currY = direction_state.loc[1];

        nextX = currX + dx;
        nextY = currY + dy;
        new_path_cost = curr_path_cost;
        if(diagonal):
            new_path_cost += DIAGONAL_COST;
        else:
            new_path_cost += 1;
        if(not self._in_range(nextX, nextY, True)):
            if(diagonal):
                direction_state.path_cost = curr_path_cost;
                return direction_state;
            else:
                direction_state.JPS_pruned = True;
                return None;
        else:
            if(state_map[nextX][nextY].JPS_pruned):
                return None;
        if(goal_function(state_map[nextX][nextY])):
            state_map[nextX][nextY].path_cost = curr_path_cost;
            return state_map[nextX][nextY];

        forced_neighbour = False;
        if(diagonal):
            neighbour_1 = ((nextX - (dx)), nextY);
            neighbour_2 = (nextX, (nextY - (dy)));
            if(not self._in_range(neighbour_1[0], neighbour_1[1], True)):
                forced_neighbour = True;
            elif(not self._in_range(neighbour_2[0], neighbour_2[1], True)):
                forced_neighbour = True;
        else:
            forced_neighbour = self._has_forced_neighbours(currX, currY, dx, dy, nextX, nextY);
        if(diagonal):
            if(forced_neighbour):
                return state_map[nextX][nextY];
            horizontal_node = self._jump_successor(state_map[nextX][nextY], dx, 0, False, new_path_cost);
            vertical_node = self._jump_successor(state_map[nextX][nextY], 0, dy, False, new_path_cost);
            if(horizontal_node != None or vertical_node != None):
                state_map[nextX][nextY].path_cost = curr_path_cost;
                return state_map[nextX][nextY];
            else:
                direction_state.JPS_pruned = True;
        else:
            if(forced_neighbour):
                state_map[nextX][nextY].path_cost = curr_path_cost;
                return state_map[nextX][nextY];
        return self._jump_successor(state_map[nextX][nextY], dx, dy, diagonal, new_path_cost);
    def _has_forced_neighbours(self, cX, cY, dx, dy, nX, nY):        
        curr_neighbour_1_x = (cX + (dy));
        curr_neighbour_1_y = (cY + (dx));                              
        curr_neighbour_2_x = (cX - (dy));
        curr_neighbour_2_y = (cY - (dx));
        next_neighbour_1_x = (nX + (dy));
        next_neighbour_1_y = (nY + (dx));                              
        next_neighbour_2_x = (nX - (dy));
        next_neighbour_2_y = (nY - (dx));
        c_1 = self._in_range(curr_neighbour_1_x, curr_neighbour_1_y, True);
        n_1 = self._in_range(next_neighbour_1_x, next_neighbour_1_y, True);
        if(c_1 != n_1):
            return True;
        c_2 = self._in_range(curr_neighbour_2_x, curr_neighbour_2_y, True);
        n_2 = self._in_range(next_neighbour_2_x, next_neighbour_2_y, True);
        if(c_2 != n_2):
            return True;
        return False;
    def _in_range(self, x, y, use_wall_check):
        index_check = (x >= 0 and x <= self.map_size[0] - 1) and (y >= 0 and y <= self.map_size[1] - 1);
        wall_check = True;
        if(index_check):
            wall_check = state_map[x][y].map_rep != 'T' and state_map[x][y].map_rep != '@';
        if(not use_wall_check):
            wall_check = True;
        return  index_check and wall_check;
    def _is_wall(self, x, y):
        index_check = (x >= 0 and x <= self.map_size[0] - 1) and (y >= 0 and y <= self.map_size[1] - 1);
        wall_check = False;
        if(index_check):
            wall_check = state_map[x][y].map_rep == 'T' or state_map[x][y].map_rep == '@';
        else:
            wall_check = False;
        return wall_check;
    def get_hash_id(self):
        return self.map_rep + str(self.loc[0]) + "_" + str(self.loc[1]);  
    
class SearchEngine:
    def __init__(self, goal_fcn):
        self.goal_fcn = goal_fcn;
    def search(self, initState):
        Closed = dict();
        Open = [];
        Open.append(initState);
        expanded = 0;
        curr_expanded_path = [];
        while len(Open) != 0:
            min_key = Open[0].key;
            curr_index = 0;
            curr_state = Open[0];
            for i in range(0, len(Open)):
                if Open[i].key < min_key:
                    min_key = Open[i].key;
                    curr_index = i;
                    curr_state = Open[i];
            del Open[curr_index];
            if(curr_state.time_expanded > len(curr_expanded_path) - 1):
                curr_expanded_path.append(curr_state);
            else:
                curr_expanded_path[curr_state.time_expanded] = curr_state;
            
            if(self.goal_fcn(curr_state)):
                print("solution found with: " + str(expanded) + " nodes expanded.");
                print("with path length: " + str(curr_state.time_expanded));
                return curr_state;
            
            else:
                path_map[curr_state.loc[0]][curr_state.loc[1]] = "x";
                Closed[curr_state.get_hash_id()] = curr_state.path_cost;                
                expanded += 1;
                if JPS:
                    states = curr_state.get_successors_JPS();
                    curr_state.JPS_pruned = True;
                else:
                    states = curr_state.get_successors();
                for s in states:
                    s.time_expanded = curr_state.time_expanded + 1;
                    if not (s.get_hash_id() in Closed.keys()):
                        s.calculate_f(curr_state.path_cost);
                        Open.append(s);
                        Closed[s.get_hash_id()] = s.path_cost;
                    else:
                        path_cost = 1;
                        if(s.accessed_diagonally):
                            path_cost = DIAGONAL_COST;
                        if(Closed[s.get_hash_id()] > curr_state.path_cost + path_cost):
                            s.path_cost = 0;
                            s.calculate_f(curr_state.path_cost);
                            Closed[s.get_hash_id()] = s.path_cost;
                            
        print("no solution found");
MAP_SMALL_EASY = "map_1.map";
MAP_LARGE_MEDIUM = "map_large_medium.map";
MAP_LARGE_HARD= "map_large_hard.map";
MAP_PATH_OUTPUT = "map_path_output.txt";
TEST_ALL_CASES = True;
heuristic_func = h_manhattan_distance;
JPS = False;
TEST_FILE = MAP_LARGE_HARD;
with open(TEST_FILE) as map_file:
    for line in map_file:
        char_nodes = [];
        for char in line:
            if(char != '\n'):
                char_nodes.append(char);
        char_map.append(char_nodes);
map_file.close;
init_state = None;
for i in range(0, len(char_map)):
    state_nodes = [];
    for j in range(0, len(char_map[i])):
        s = State(char_map[i][j], (i,j), (len(char_map), len(char_map[i])));
        if(char_map[i][j] == 's'):
            init_state = s;
        if(char_map[i][j] == 'd'):
            dest_location = (i , j);
        state_nodes.append(s);
    state_map.append(state_nodes);
if(TEST_ALL_CASES):
    path_map = deepcopy(char_map);
    print("Manhattan, no JPS: \n");
    heuristic_func = h_manhattan_distance;
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();

    print("Euclidean, no JPS: \n");
    heuristic_func = h_euclidean_distance;
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();

    JPS = True;    
    print("Manhattan, with JPS: \n");
    heuristic_func = h_manhattan_distance;
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();

    print("Euclidean, with JPS: \n");
    heuristic_func = h_euclidean_distance;
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();
else:  
    path_map = deepcopy(char_map);
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds.");
