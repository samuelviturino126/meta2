import heapq
from collections import defaultdict
import matplotlib.pyplot as plt
# se isso for necessário para outras funções auxiliares
from mapa_para_grafo import *

# === 1. Criação do grafo a partir do C-Space ===
grafo = defaultdict(list)
larg, alt = c_space.shape

for i in range(larg):
    for j in range(alt):
        if c_space[i, j] == 0:
            # Verifica as 4 direções ortogonais
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                ni, nj = i + dx, j + dy
                if 0 <= ni < larg and 0 <= nj < alt and c_space[ni, nj] == 0:
                    grafo[(i, j)].append((ni, nj))

# === 2. Função heurística (distância de Manhattan) ===


def heuristica(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# === 3. Algoritmo A* ===


def a_star(grafo, inicio, objetivo):
    fila = []
    heapq.heappush(fila, (0, inicio))
    g_score = {inicio: 0}
    came_from = {}

    while fila:
        _, atual = heapq.heappop(fila)
        if atual == objetivo:
            caminho = [atual]
            while atual in came_from:
                atual = came_from[atual]
                caminho.append(atual)
            caminho.reverse()
            return caminho

        for vizinho in grafo[atual]:
            custo = g_score[atual] + 1  # Custo unitário
            if vizinho not in g_score or custo < g_score[vizinho]:
                g_score[vizinho] = custo
                f = custo + heuristica(vizinho, objetivo)
                heapq.heappush(fila, (f, vizinho))
                came_from[vizinho] = atual
    return None


# === 4. Obtenção de posições do robô e do objetivo ===
handle_robo = sim.getObject('/PioneerP3DX')
handle_goal = sim.getObject('/Goal')

x_inicio, y_inicio, _ = sim.getObjectPosition(handle_robo)
x_objetivo, y_objetivo, _ = sim.getObjectPosition(handle_goal)

inicio = ponto_para_indice(x_inicio, y_inicio)
objetivo = ponto_para_indice(x_objetivo, y_objetivo)

# === 5. Executa o A* ===
caminho = a_star(grafo, inicio, objetivo)

# === 6. Visualização do grafo com C-Space, arestas e caminho ===
plt.figure(figsize=(8, 8))

# Fundo: mapa do C-Space
plt.imshow(c_space.T, origin='lower', cmap='gray_r')

# Arestas do grafo (entre células livres)
for (i, j), vizinhos in grafo.items():
    for (ni, nj) in vizinhos:
        plt.plot([i, ni], [j, nj], color='gray', alpha=0.2, linewidth=0.5)

# Caminho A*
if caminho:
    xs, ys = zip(*caminho)
    plt.plot(xs, ys, color='red', linewidth=2, label='Caminho A*')
    plt.plot(inicio[0], inicio[1], 'go', label='Início')
    plt.plot(objetivo[0], objetivo[1], 'bo', label='Objetivo')
else:
    print("Nenhum caminho encontrado.")

plt.legend()
plt.title("Grafo sobre o Espaço de Configuração (C-Space)")
plt.xlabel("Índice X")
plt.ylabel("Índice Y")
plt.tight_layout()
plt.show()
