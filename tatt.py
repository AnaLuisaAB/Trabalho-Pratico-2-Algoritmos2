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

        # Encontra a seção de coordenadas
        coordenadas_inicio = linhas.index("NODE_COORD_SECTION\n") + 1

        # Adiciona os vértices e as coordenadas ao grafo
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

def path_cost(G, path):
    weight = 0
    l = len(path)
    #Percorre cada aresta do caminho e soma os pesos
    for i in range(l - 1):
        weight += G[path[i]][path[i + 1]]['weight']
    return weight

def twice_around_the_tree(G, root): 
    #Computa T uma árvore geradora mínima de G
    T = nx.minimum_spanning_tree(G)
    #Seja H a lista de vértices de T em pré-ordem a partir de root
    H = list(nx.dfs_preorder_nodes(T, source=root))
    #Adiciona o vértice root no final de H para fechar o ciclo
    H.append(root)
    #Retorna o ciclo Hamiltoniano H
    return H, path_cost(G, H)

parser = argparse.ArgumentParser(description='Processar um arquivo TSP.')
parser.add_argument('file', type=str, help='Caminho para o arquivo TSP')
args = parser.parse_args()
file_path = args.file
graph = read_tsp_file(file_path)
path, cost = twice_around_the_tree(graph, list(graph.nodes())[0])
print('O melhor caminho encontrado foi: ', path, ' com custo: ', cost)



    
    
    



    
    
    