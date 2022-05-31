import heapq
from random import randint, choice
from collections import deque


def save_graph(file_name, number_of_nodes, edges):
    with open(file_name, "w+") as file:
        file.write(str(number_of_nodes))
        for edge in edges:
            edge = [str(i) for i in edge]
            edge = "\n" + " ".join(edge)
            file.write(edge)


def make_random_tree(number_of_nodes, val_min=1, val_max=1):
    nodes_used = 0
    graph = [[] for node in range(number_of_nodes)]
    edges = []
    for node in range(1, number_of_nodes):
        neighbour = randint(0, nodes_used)
        dist = randint(val_min, val_max)
        graph[neighbour].append((node, dist))
        graph[node].append((neighbour, dist))
        edges.append((node, neighbour, dist))
        edges.append((neighbour, node, dist))
        nodes_used += 1
    return graph, number_of_nodes, edges


def make_random_connected_graph(number_of_nodes, number_of_edges, val_min=1, val_max=1):
    if (number_of_nodes > number_of_edges + 1):
        raise ValueError("Number of edges must be at max 1 lower than number of nodes!")
    graph, n_tree, edges = make_random_tree(number_of_nodes)
    number_of_edges -= number_of_nodes - 1
    for _ in range(number_of_edges):
        while(True):
            nodeA = randint(0, number_of_nodes - 1)
            nodeB = randint(0, number_of_nodes - 1)
            if (nodeB == nodeA):
                continue
            edge_exist = False
            for node, dist in graph[nodeA]:
                if (node == nodeB):
                    edge_exist = True
                    break
            if not(edge_exist):
                break

        dist = randint(val_min, val_max)
        graph[nodeA].append((nodeB, dist))
        graph[nodeB].append((nodeA, dist))
        edges.append((nodeA, nodeB, dist))
        edges.append((nodeB, nodeA, dist))
    return graph, number_of_nodes, edges



def dijkstra(start, graph):
    distances = [None for _ in graph]
    halda = []
    heapq.heapify(halda)
    heapq.heappush(halda, (0, start))
    while(halda):
        dist, node = heapq.heappop(halda)
        if not(distances[node]):
            distances[node] = dist
            for neighbour, edge in graph[node]:
                if not(distances[neighbour]):
                    heapq.heappush(halda, (dist+edge, neighbour))
    return distances

def load_graph(file_name):
    with open(file_name, "r") as file:
        edges = []
        first = True
        data = file.read().strip()
        data = [[int(i) for i in line.strip().split()] for line in data.split("\n")]
        return data[0][0], data[1::]

def make_graph(nodes, edges):
    graph = [[] for _ in range(nodes)]
    for nodeA, nodeB, dist in edges:
        graph[nodeA].append((nodeB, dist))
    return graph

def BFS(start, graf):
    queue = deque()
    distances = [None for _ in graf]
    queue.appendleft((0, start))
    while(queue):
        dist, node = queue.pop()
        if not(distances[node]):
            distances[node] = dist
            for neighbour, edge in graf[node]:
                if not(distances[neighbour]):
                    queue.appendleft((dist+1, neighbour))
    return distances

def vytvor_tabulky(graph):
    dijkstra_tabulka = []
    bfs_tabulka = []
    for node in range(len(graph)):
        dijkstra_tabulka.append(dijkstra(node, graph))
        bfs_tabulka.append(BFS(node, graph))
    return dijkstra_tabulka, bfs_tabulka



nodes, edges = load_graph("data/graf02")
graph = make_graph(nodes, edges)

