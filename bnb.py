from heapq import heappop, heappush, heapify
import numpy as np
import networkx as nx
import argparse
import math
import time
import os
from memory_profiler import memory_usage
from functools import partial

INF = float('inf')

class Node:
    def __init__(self,bound, cost, level, path):
        self.bound = bound
        self.cost = cost
        self.level = level
        self.path = path
        
    def __lt__(self, other):
        return self.bound < other.bound

    def __le__(self, other):
        return self.bound <= other.bound

    def __gt__(self, other):
        return self.bound > other.bound

    def __ge__(self, other):
        return self.bound >= other.bound

    def __eq__(self, other):
        return self.bound == other.bound

    def __ne__(self, other):
        return self.bound != other.bound
    def __str__(self):
        return f"bound: {self.bound}, cost: {self.cost}, level: {self.level}, path: {self.path}"
    
def read_tsp_file(file_path):
    # Cria um grafo completo não direcionado
    G = nx.Graph()

    with open(file_path, 'r') as file:
        linhas = file.readlines()

        # Encontra a seção de coordenadas
        coordenadas_inicio = linhas.index("NODE_COORD_SECTION\n") + 1
        coordenadas_fim = linhas.index("EOF\n") - 1

        # Adiciona os vértices e  as coordenadas ao grafo
        for linha in linhas[coordenadas_inicio:coordenadas_fim]:
            partes = linha.split()
            numero_vertice = int(partes[0])
            coordenada_x = float(partes[1])
            coordenada_y = float(partes[2])
            G.add_node(numero_vertice, pos=(coordenada_x, coordenada_y))


        # Adiciona arestas ao grafo com pesos igual à distância euclidiana
        for i in range(1, len(G) + 1):
            for j in range(i + 1, len(G) + 1):
                node_i = G.nodes[i]['pos']
                node_j = G.nodes[j]['pos']
                distance = - (math.sqrt((node_i[0] - node_j[0]) ** 2 + (node_i[1] - node_j[1]) ** 2))
                G.add_edge(i, j, weight=distance)

    return G

def adj_matrix(G):
    # Cria uma matriz de adjacência
    N = len(G)
    adj = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            if i != j:
                adj[i][j] = G[i + 1][j + 1]['weight']
    return adj

def two_min_edge(adj, i):
    first, second = INF, INF
    N=len(adj[i])
    for j in [vertex for vertex in range(N) if vertex != i]:
        weight = adj[i][j]
        # Atualiza first e second conforme necessário
        if weight <= first:
            second, first = first, weight
        elif weight < second:
            second = weight

    return first, second

def bound(adj, path, current_cost):
    bound = 0
    # Itera pelos vértices que não estão no caminho, calculando o lower bound
    for i in [vertex for vertex in range(len(adj)) if vertex not in path]:
        first, second = two_min_edge(adj, i)
        bound += first + second
    # Divide por 2, pois cada aresta foi contada duas vezes
    bound = math.ceil(bound/2) 
    # Soma o peso do caminho atual
    bound += current_cost
    return bound

def bnb_tsp (adj):
    N=len(adj)
    # Cria a fila de prioridade
    pq = []
    heapify(pq)
    # Cria o primeiro nó e adiciona na fila
    root = Node(bound(adj, [0], 0), 0, 0, [0])
    heappush(pq, root)
    # Inicializa a melhor solução, que é o caminho trivial
    best_path = []
    best = 0
    for i in range(N):
        best_path.append(i)
        best +=  adj[i][(i + 1) % N]
    best_path.append(best_path[0])
    best += adj[N-1][0]
    # Enquanto a fila não estiver vazia
    while pq:
        # Remove o nó com menor bound
        node = (heappop(pq))
        node= Node(node.bound,node.cost,node.level,node.path)
        # Se o nó for uma folha e o custo for menor que o da melhor solução, atualiza a melhor solução
        if node.level == N:
            if best > node.cost:
                best_path = node.path
                best = node.cost
        # Se o bound do nó for menor que o da melhor solução, expande o nó
        elif node.bound < best:
            # Se o nó não for uma folha, cria os filhos
            if node.level < N-1:
                for i in range(N):
                    # Se o vértice não estiver no caminho
                    if i not in node.path:
                        # Cria um novo nó
                        new_path = node.path + [i]
                        new_bound = bound(adj, new_path, node.cost)
                        new_cost = node.cost + adj[node.path[-1]][i]
                        new_level = node.level + 1
                        new_node = Node(new_bound, new_cost, new_level, new_path)
                        # Se o bound for menor que o da melhor solução, adiciona o nó na fila
                        if new_bound < best:
                            heappush(pq, (new_node))
            # Se o nó for uma folha, cria um nó com o caminho completo
            else:
                new_path = node.path + [node.path[0]]
                if bound(adj, new_path, node.cost) < best : 
                    new_bound = bound(adj, new_path, node.cost)
                    new_level = node.level + 1
                    new_cost = node.cost + adj[node.path[-1]][node.path[0]]
                    new_node = Node(new_bound, new_cost, new_level, new_path)
                    heappush(pq, new_node)
                
    return best_path, best

parser = argparse.ArgumentParser(description='Processar um arquivo TSP.')
parser.add_argument('file', type=str, help='Caminho para o arquivo TSP')
args = parser.parse_args()
file_path = args.file
graph = read_tsp_file(file_path)
adj = adj_matrix(graph)
path, cost = bnb_tsp(adj)
print('O melhor caminho é:', path, 'com custo:', cost)



