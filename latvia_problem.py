"""2022-02-28
Authors: provided in the LICENSE file
Modified by: Arvydas Jurkevičius, Dovydas Gudauskas, Rolandas Romanovskis, Vytenis Bačkauskas"""

import sys
from copy import deepcopy
from tkinter import *

#from search import *

from search import Node, deque, GraphProblem, UndirectedGraph

from utils import *


from utils import PriorityQueue

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

root = None
city_coord = {}
latvia_problem = None
algo = None
start = None
goal = None
counter = -1
city_map = None
frontier = None
front = None
node = None
next_button = None
explored = None

latvia_map = UndirectedGraph(dict(
    Liepaja=dict(Ventspils=105, Saldus=90),
    Ventspils=dict(Kuldiga=52),
    Saldus=dict(Tukums=53, Bauska=109),
    Kuldiga=dict(Talsi=48),
    Talsi=dict(Tukums=46),
    Tukums=dict(Riga=58),
    Riga=dict(Bauska=61, Limbazi=73, Cesis=82, Madona=129),
    Limbazi=dict(Valga=82),
    Valga=dict(Gulbene=80, Madona=103),
    Madona=dict(Rezekne=78, Gulbene=48, Cesis=77),
    Rezekne=dict(Kraslava=69)))
latvia_map.locations = dict(
    Liepaja=(55, 350), Ventspils=(120, 492), Talsi=(215, 470),
    Tukums=(230, 415), Saldus=(185, 355), Riga=(310, 405),
    Bauska=(305, 330), Limbazi=(390, 480), Valga=(510, 510),
    Madona=(510, 390), Kraslava=(570, 240), Rezekne=(580, 315),
    Kuldiga=(150, 415), Cesis=(415, 445), Gulbene=(550, 430))

def create_map(root):
    """Šis metodas grafinėje vartotojo sąsajoje išdėlioja žemėlapį"""
    global city_map, start, goal
    latvia_locations = latvia_map.locations
    width = 750
    height = 670
    margin = 5
    city_map = Canvas(root, width=width, height=height)
    city_map.pack()

    # Since lines have to be drawn between particular points, we need to list
    # them separately
    make_line(
        city_map,
        latvia_locations['Liepaja'][0],
        height -
        latvia_locations['Liepaja'][1],
        latvia_locations['Ventspils'][0],
        height -
        latvia_locations['Ventspils'][1],
        latvia_map.get('Liepaja', 'Ventspils'))
    make_line(
        city_map,
        latvia_locations['Liepaja'][0],
        height -
        latvia_locations['Liepaja'][1],
        latvia_locations['Saldus'][0],
        height -
        latvia_locations['Saldus'][1],
        latvia_map.get('Liepaja', 'Saldus'))
    make_line(
        city_map,
        latvia_locations['Kuldiga'][0],
        height -
        latvia_locations['Kuldiga'][1],
        latvia_locations['Ventspils'][0],
        height -
        latvia_locations['Ventspils'][1],
        latvia_map.get('Kuldiga', 'Ventspils'))
    make_line(
        city_map,
        latvia_locations['Kuldiga'][0],
        height -
        latvia_locations['Kuldiga'][1],
        latvia_locations['Talsi'][0],
        height -
        latvia_locations['Talsi'][1],
        latvia_map.get('Kuldiga', 'Talsi'))
    make_line(
        city_map,
        latvia_locations['Saldus'][0],
        height -
        latvia_locations['Saldus'][1],
        latvia_locations['Tukums'][0],
        height -
        latvia_locations['Tukums'][1],
        latvia_map.get('Saldus', 'Tukums'))
    make_line(
        city_map,
        latvia_locations['Tukums'][0],
        height -
        latvia_locations['Tukums'][1],
        latvia_locations['Talsi'][0],
        height -
        latvia_locations['Talsi'][1],
        latvia_map.get('Tukums', 'Talsi'))
    make_line(
        city_map,
        latvia_locations['Tukums'][0],
        height -
        latvia_locations['Tukums'][1],
        latvia_locations['Riga'][0],
        height -
        latvia_locations['Riga'][1],
        latvia_map.get('Tukums', 'Riga'))
    make_line(
        city_map,
        latvia_locations['Saldus'][0],
        height -
        latvia_locations['Saldus'][1],
        latvia_locations['Bauska'][0],
        height -
        latvia_locations['Bauska'][1],
        latvia_map.get('Saldus', 'Bauska'))
    make_line(
        city_map,
        latvia_locations['Bauska'][0],
        height -
        latvia_locations['Bauska'][1],
        latvia_locations['Riga'][0],
        height -
        latvia_locations['Riga'][1],
        latvia_map.get('Bauska', 'Riga'))
    make_line(
        city_map,
        latvia_locations['Riga'][0],
        height -
        latvia_locations['Riga'][1],
        latvia_locations['Limbazi'][0],
        height -
        latvia_locations['Limbazi'][1],
        latvia_map.get('Riga', 'Limbazi'))
    make_line(
        city_map,
        latvia_locations['Riga'][0],
        height -
        latvia_locations['Riga'][1],
        latvia_locations['Cesis'][0],
        height -
        latvia_locations['Cesis'][1],
        latvia_map.get('Riga', 'Cesis'))
    make_line(
        city_map,
        latvia_locations['Riga'][0],
        height -
        latvia_locations['Riga'][1],
        latvia_locations['Madona'][0],
        height -
        latvia_locations['Madona'][1],
        latvia_map.get('Riga', 'Madona'))
    make_line(
        city_map,
        latvia_locations['Madona'][0],
        height -
        latvia_locations['Madona'][1],
        latvia_locations['Cesis'][0],
        height -
        latvia_locations['Cesis'][1],
        latvia_map.get('Madona', 'Cesis'))
    make_line(
        city_map,
        latvia_locations['Madona'][0],
        height -
        latvia_locations['Madona'][1],
        latvia_locations['Valga'][0],
        height -
        latvia_locations['Valga'][1],
        latvia_map.get('Madona', 'Valga'))
    make_line(
        city_map,
        latvia_locations['Madona'][0],
        height -
        latvia_locations['Madona'][1],
        latvia_locations['Gulbene'][0],
        height -
        latvia_locations['Gulbene'][1],
        latvia_map.get('Madona', 'Gulbene'))
    make_line(
        city_map,
        latvia_locations['Valga'][0],
        height -
        latvia_locations['Valga'][1],
        latvia_locations['Gulbene'][0],
        height -
        latvia_locations['Gulbene'][1],
        latvia_map.get('Valga', 'Gulbene'))
    make_line(
        city_map,
        latvia_locations['Madona'][0],
        height -
        latvia_locations['Madona'][1],
        latvia_locations['Rezekne'][0],
        height -
        latvia_locations['Rezekne'][1],
        latvia_map.get('Madona', 'Rezekne'))
    make_line(
        city_map,
        latvia_locations['Rezekne'][0],
        height -
        latvia_locations['Rezekne'][1],
        latvia_locations['Kraslava'][0],
        height -
        latvia_locations['Kraslava'][1],
        latvia_map.get('Rezekne', 'Kraslava'))
    make_line(
        city_map,
        latvia_locations['Limbazi'][0],
        height -
        latvia_locations['Limbazi'][1],
        latvia_locations['Valga'][0],
        height -
        latvia_locations['Valga'][1],
        latvia_map.get('Limbazi', 'Valga'))

    for city in latvia_locations.keys():
        make_rectangle(
            city_map,
            latvia_locations[city][0],
            height -
            latvia_locations[city][1],
            margin,
            city)

    make_legend(city_map)


def make_line(map, x0, y0, x1, y1, distance):
    """Šis metodas nubrėžia linijas tarp dviejų koordinačių plokštumos taškų"""
    map.create_line(x0, y0, x1, y1)
    map.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=distance)


def make_rectangle(map, x0, y0, margin, city_name):
    """Šis metodas nubrėžia kvadratėlius miestams ir uždeda miestų pavadinimus"""
    global city_coord
    rect = map.create_rectangle(
        x0 - margin,
        y0 - margin,
        x0 + margin,
        y0 + margin,
        fill="white")
    if "Tukums" in city_name or "Cesis" in city_name or "Rezekne" in city_name:
        map.create_text(
            x0 - 2 * margin,
            y0 - margin,
            text=city_name,
            anchor=E)
    elif "Kuldiga" in city_name or "Madona" in city_name:
        map.create_text(
            x0 - 2 * margin,
            y0 + 2 * margin,
            text=city_name,
            anchor=N)
    elif "Riga" in city_name:
        map.create_text(
            x0 - 1 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=SE)
    elif "Gulbene" in city_name:
        map.create_text(
            x0 + 8 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=SE)
    else:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=SE)
    city_coord.update({city_name: rect})


def make_legend(map):
    """Šis metodas nubrėžia žemėlapio legendą"""
    rect1 = map.create_rectangle(600, 100, 610, 110, fill="white")
    text1 = map.create_text(615, 105, anchor=W, text="Un-explored")

    rect2 = map.create_rectangle(600, 115, 610, 125, fill="orange")
    text2 = map.create_text(615, 120, anchor=W, text="Frontier")

    rect3 = map.create_rectangle(600, 130, 610, 140, fill="red")
    text3 = map.create_text(615, 135, anchor=W, text="Currently Exploring")

    rect4 = map.create_rectangle(600, 145, 610, 155, fill="grey")
    text4 = map.create_text(615, 150, anchor=W, text="Explored")

    rect5 = map.create_rectangle(600, 160, 610, 170, fill="dark green")
    text5 = map.create_text(615, 165, anchor=W, text="Final Solution")


def tree_search(problem):
    """Medžio paieška"""
    global counter, frontier, node

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def graph_search(problem):
    """Grafo paieška"""
    global counter, frontier, node, explored
    if counter == -1:
        frontier.append(Node(problem.initial))
        explored = set()

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def display_frontier(queue):
    """Šis metodas oranžine spalva pažymi eilėje esančius miestus"""
    global city_map, city_coord
    qu = deepcopy(queue)
    while qu:
        node = qu.pop()
        for city in city_coord.keys():
            if node.state == city:
                city_map.itemconfig(city_coord[city], fill="orange")


def display_current(node):
    """Šis metodas raudonai pažymi šiuo metu nagrinėjamą miestą"""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="red")


def display_explored(node):
    """Šis metodas pilkai pažymi jau aplankytą miestą"""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="gray")


def display_final(cities):
    """Šis metodas žaliai pažymi galutinio sprendimo miestus"""
    global city_map, city_coord
    for city in cities:
        city_map.itemconfig(city_coord[city], fill="green")


def breadth_first_tree_search(problem):
    """Medžio paieškos į plotį algoritmas"""
    global frontier, counter, node
    if counter == -1:
        frontier = deque()

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.popleft()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def depth_first_tree_search(problem):
    """Medžio paieškos į gylį algoritmas"""
    # This search algorithm might not work in case of repeated paths.
    global frontier, counter, node
    if counter == -1:
        frontier = []  # stack

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def breadth_first_graph_search(problem):
    """Grafo paieškos į plotį algoritmas"""
    global frontier, node, explored, counter
    if counter == -1:
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node

        frontier = deque([node])  # FIFO queue

        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.popleft()
        display_current(node)
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def depth_first_graph_search(problem):
    """Grafo paieškos į gylį algoritmas"""
    global counter, frontier, node, explored
    if counter == -1:
        frontier = []  # stack
    if counter == -1:
        frontier.append(Node(problem.initial))
        explored = set()

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def best_first_graph_search(problem, f):
    """Grafo pirmo geriausio paieškos algoritmas"""
    global frontier, node, explored, counter

    if counter == -1:
        f = memoize(f, 'f')
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node
        frontier = PriorityQueue('min', f)
        frontier.append(node)
        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        display_current(node)
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def uniform_cost_search(problem):
    """Vienodos 'kainos' paieškos algoritmas"""
    return best_first_graph_search(problem, lambda node: node.path_cost)


def astar_search(problem, h=None):
    """A* paieškos algoritmas"""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))


# TODO:
# Remove redundant code.
# Make the interchangeability work between various algorithms at each step.
def on_click():
    """Šis metodas naudojamas mygtuko 'Next' funkcionalumui"""
    global algo, counter, next_button, latvia_problem, start, goal
    romania_problem = GraphProblem(start.get(), goal.get(), latvia_map)
    if "Breadth-First Tree Search" == algo.get():
        node = breadth_first_tree_search(romania_problem)
        if node is not None:
            final_path = breadth_first_tree_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Depth-First Tree Search" == algo.get():
        node = depth_first_tree_search(romania_problem)
        if node is not None:
            final_path = depth_first_tree_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Breadth-First Graph Search" == algo.get():
        node = breadth_first_graph_search(romania_problem)
        if node is not None:
            final_path = breadth_first_graph_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Depth-First Graph Search" == algo.get():
        node = depth_first_graph_search(romania_problem)
        if node is not None:
            final_path = depth_first_graph_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Uniform Cost Search" == algo.get():
        node = uniform_cost_search(romania_problem)
        if node is not None:
            final_path = uniform_cost_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "A* - Search" == algo.get():
        node = astar_search(romania_problem)
        if node is not None:
            final_path = astar_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1


def reset_map():
    """Šis metodas atstato žemėlapį į pradinę padėtį"""
    global counter, city_coord, city_map, next_button
    counter = -1
    for city in city_coord.keys():
        city_map.itemconfig(city_coord[city], fill="white")
    next_button.config(state="normal")


# TODO: Add more search algorithms in the OptionMenu
if __name__ == "__main__":
    # global algo, start, goal, next_button
    root = Tk()
    root.title("Road Map of Latvia")
    root.geometry("950x1050")
    algo = StringVar(root)
    start = StringVar(root)
    goal = StringVar(root)
    algo.set("Breadth-First Tree Search")
    start.set('Liepaja')
    goal.set('Riga')
    cities = sorted(latvia_map.locations.keys())
    algorithm_menu = OptionMenu(
        root,
        algo, "Breadth-First Tree Search", "Depth-First Tree Search",
        "Breadth-First Graph Search", "Depth-First Graph Search",
        "Uniform Cost Search", "A* - Search")
    Label(root, text="\n Search Algorithm").pack()
    algorithm_menu.pack()
    Label(root, text="\n Start City").pack()
    start_menu = OptionMenu(root, start, *cities)
    start_menu.pack()
    Label(root, text="\n Goal City").pack()
    goal_menu = OptionMenu(root, goal, *cities)
    goal_menu.pack()
    frame1 = Frame(root)
    next_button = Button(
        frame1,
        width=6,
        height=2,
        text="Next",
        command=on_click,
        padx=2,
        pady=2,
        relief=GROOVE)
    # next_button.pack(side=RIGHT)
    next_button.pack(side="right")
    reset_button = Button(
        frame1,
        width=6,
        height=2,
        text="Reset",
        command=reset_map,
        padx=2,
        pady=2,
        relief=GROOVE)
    #reset_button.pack(side=RIGHT)
    reset_button.pack(side="right")
    frame1.pack(side=BOTTOM)
    create_map(root)
    root.mainloop()
