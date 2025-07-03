from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from collections import deque
import time

def gerador_de_caminhos(U, start, goal):
    caminho = []
    atual = start

    vizinhos = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # cima, baixo, esquerda, direita

    while atual != goal:
        i, j = atual
        menor = float('inf')
        proximo = None

        # Verifica todos os vizinhos
        for dx, dy in vizinhos:
            ni, nj = i + dx, j + dy
            if 0 <= ni < U.shape[0] and 0 <= nj < U.shape[1]:
                if U[ni, nj] >= 0 and U[ni, nj] < menor:
                    menor = U[ni, nj]
                    proximo = (ni, nj)

        # Fora do for: se nenhum vizinho válido foi encontrado
        if proximo is None:
            print("Caminho Bloqueado.")
            return []

        # Avança para o próximo ponto
        caminho.append(proximo)
        atual = proximo

    return caminho




