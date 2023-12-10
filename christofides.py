import networkx as nx
import numpy as np
import pandas as pd
import math
import time
import argparse
import os
from memory_profiler import memory_usage
from functools import partial

def read_tsp_file(file_path):
    # Cria um grafo completo não direcionado
    G = nx.Graph()

    with open(file_path, 'r') as file:
        linhas = file.readlines()

        # Encontrar a seção de coordenadas
        coordenadas_inicio = linhas.index("NODE_COORD_SECTION\n") + 1

        # Adicionar nós e coordenadas ao grafo
        for linha in linhas[coordenadas_inicio:-1]:
            if linha == "EOF\n":
                break
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
                distance = math.sqrt((node_i[0] - node_j[0]) ** 2 + (node_i[1] - node_j[1]) ** 2)
                G.add_edge(i, j, weight=distance)

    return G

def path_weight(G, path):
    weight = 0
    for edge in path:
        if edge[0] == edge[1]:
            weight += 0
        else:
            weight += G[edge[0]][edge[1]]['weight']
    return weight

def christofides(G): 
    #Computa T uma árvore geradora mínima de G
    T = nx.minimum_spanning_tree(G)
    #Seja I o conjunto de vértices de grau ímpar de T. Computa M um matching perfeito de peso mínimo no subgrafo induzido por I
    odd_degree_nodes = [node for node, degree in T.degree() if degree % 2 == 1]
    induced_subgraph = G.subgraph(odd_degree_nodes)
    matching = nx.min_weight_matching(induced_subgraph)
    #Seja G’ o multigrafo formado com os vértices de V e aresta de M e T. Compute um circuito euleriano em G’
    G_prime = nx.MultiGraph(T)
    G_prime.add_edges_from(matching)
    eulerian_circuit = list(nx.eulerian_circuit(G_prime))
    #Elimina vértices duplicados, substituindo subcaminhos u-w-v por arestas u-v
    final_path = []
    visited = set()
    #Para cada aresta (u, w) do circuito euleriano, se existe um vértice v tal que (w, v) é a próxima aresta do circuito euleriano e w já foi visitado, então substitua o subcaminho u-w-v por aresta u-v
    for edge in eulerian_circuit:
        u, w = edge[0], edge[1]
        next_edge = eulerian_circuit[eulerian_circuit.index(edge) + 1] if eulerian_circuit.index(edge) < len(eulerian_circuit) - 1 else None
        if w in visited and next_edge:
            v = next_edge[0]
            x = next_edge[1]
            if w == v:
                final_path.append((u, x))
                visited.add(x)
                eulerian_circuit.remove(next_edge)
        else:
            final_path.append((u, w))
        visited.add(u)
        visited.add(w)
                
    return final_path, path_weight(G, final_path)
    
parser = argparse.ArgumentParser(description='Processar um arquivo TSP.')
parser.add_argument('file', type=str, help='Caminho para o arquivo TSP')
args = parser.parse_args()
file_path = args.file
graph = read_tsp_file(file_path)
path, cost = christofides(graph)
print('O melhor caminho encontrado foi: ', path, ' com custo: ', cost)

