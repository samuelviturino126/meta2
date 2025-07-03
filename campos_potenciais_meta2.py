from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from matplotlib.colors import ListedColormap
from collections import deque
from geradordecaminhovsk_meta2 import gerador_de_caminhos
# Resolução do mapa (25cm por célula)
map_reso = 0.125

# Conecta ao CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject('sim')


def rotatedVec2d(vec_in, angle):
    return [
        math.cos(angle) * vec_in[0] - math.sin(angle) * vec_in[1],
        math.sin(angle) * vec_in[0] + math.cos(angle) * vec_in[1]
    ]


def get_rectangle_vertices(sim, handle):
    sizes, _ = sim.getShapeBB(handle)
    largura, altura = sizes[0], sizes[1]

    vertices_rel = [
        [-largura/2, -altura/2],
        [largura/2, -altura/2],
        [largura/2, altura/2],
        [-largura/2, altura/2]
    ]
    pos = sim.getObjectPosition(handle)
    gamma = sim.getObjectOrientation(handle)[-1]
    vertices_abs = [rotatedVec2d(v, gamma) for v in vertices_rel]
    return [[v[0] + pos[0], v[1] + pos[1]] for v in vertices_abs]


def coord_para_indice(x_p, y_p, map_x_min, map_y_min, map_reso):
    i = int((x_p - map_x_min) / map_reso)
    j = int((y_p - map_y_min) / map_reso)
    return i, j


def marcar_obstaculo_na_matriz(c_space, vertices, map_x_min, map_y_min, map_reso):
    path = mplPath.Path(vertices)
    xs, ys = zip(*vertices)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    i_min, j_min = coord_para_indice(
        min_x, min_y, map_x_min, map_y_min, map_reso)
    i_max, j_max = coord_para_indice(
        max_x, max_y, map_x_min, map_y_min, map_reso)

    i_min = max(i_min, 0)
    j_min = max(j_min, 0)
    i_max = min(i_max, c_space.shape[1] - 1)
    j_max = min(j_max, c_space.shape[0] - 1)

    for i in range(i_min, i_max + 1):
        for j in range(j_min, j_max + 1):
            x = map_x_min + i * map_reso + map_reso/2
            y = map_y_min + j * map_reso + map_reso/2
            if path.contains_point((x, y)):
                c_space[j, i] = -999


handle_robo = sim.getObjectHandle('/PioneerP3DX')
pos_robo = sim.getObjectPosition(handle_robo)
floor_handle = sim.getObject('/Floor')
vertices_floor = get_rectangle_vertices(sim, floor_handle)
xs, ys = zip(*vertices_floor)
map_x_min, map_x_max = min(xs), max(xs)
map_y_min, map_y_max = min(ys), max(ys)
map_larg = map_x_max - map_x_min
map_alt = map_y_max - map_y_min
n_larg = int(map_larg / map_reso)
n_alt = int(map_alt / map_reso)
c_space = np.full((n_alt, n_larg), 999)

nomes_a_ignorar = ['/PioneerP3DX', '/Floor']
handles_ignorados = []
for nome in nomes_a_ignorar:
    try:
        h = sim.getObject(nome)
        handles_ignorados.append(h)
    except Exception:
        print(f"Aviso: objeto '{nome}' não encontrado.")

todos_objetos = sim.getObjectsInTree(
    sim.handle_scene, sim.sceneobject_shape, 0b00000010)
obstaculos = [h for h in todos_objetos if h not in handles_ignorados]
print("Obstáculos no cenário (ignorando PioneerP3DX e Floor):")
for shape in obstaculos:
    name = sim.getObjectAlias(shape)
    print(f"- {name} (handle: {shape})")

for obj_handle in obstaculos:
    vertices_obj = get_rectangle_vertices(sim, obj_handle)
    marcar_obstaculo_na_matriz(
        c_space, vertices_obj, map_x_min, map_y_min, map_reso)


def marcar_zona_de_segurança(c_space, raio=2):
    zonas = []

    for j in range(c_space.shape[0]):
        for i in range(c_space.shape[1]):
            if c_space[j, i] == -999:  # obstáculo
                for dj in range(-raio, raio+1):
                    for di in range(-raio, raio+1):
                        if dj == 0 and di == 0:
                            continue
                        nj = j + dj
                        ni = i + di
                        if 0 <= nj < c_space.shape[0] and 0 <= ni < c_space.shape[1]:
                            if c_space[nj, ni] == 999:
                                zonas.append((nj, ni))

    for (j, i) in zonas:
        c_space[j, i] = -998  # zona de segurança


marcar_zona_de_segurança(c_space, raio=2)

try:
    goal_handle = sim.getObject('/Goal')
    pos_goal = sim.getObjectPosition(goal_handle)
    x_goal, y_goal = pos_goal[0], pos_goal[1]
    i_goal, j_goal = coord_para_indice(
        x_goal, y_goal, map_x_min, map_y_min, map_reso)
    if 0 <= j_goal < c_space.shape[0] and 0 <= i_goal < c_space.shape[1]:
        c_space[j_goal, i_goal] = 0
        print(f"Goal marcado em: ({i_goal}, {j_goal})")
    else:
        print("Posição do Goal está fora dos limites do mapa.")
except Exception as e:
    print(f"Erro ao localizar /Goal: {e}")


def propagar_potencial_manhattan(c_space, i_goal, j_goal):
    fila = deque()
    fila.append((j_goal, i_goal))
    visitados = set()
    visitados.add((j_goal, i_goal))
    direcoes = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while fila:
        j_atual, i_atual = fila.popleft()
        valor_atual = c_space[j_atual, i_atual]

        for dj, di in direcoes:
            j_viz = j_atual + dj
            i_viz = i_atual + di
            if 0 <= j_viz < c_space.shape[0] and 0 <= i_viz < c_space.shape[1]:
                if c_space[j_viz, i_viz] == 999:
                    c_space[j_viz, i_viz] = valor_atual + 1
                    fila.append((j_viz, i_viz))
                    visitados.add((j_viz, i_viz))


propagar_potencial_manhattan(c_space, i_goal, j_goal)


x_robo, y_robo = pos_robo[0], pos_robo[1]
i_start, j_start = coord_para_indice(
    x_robo, y_robo, map_x_min, map_y_min, map_reso)

if 0 <= j_start < c_space.shape[0] and 0 <= i_start < c_space.shape[1]:
    start = (j_start, i_start)
    goal = (j_goal, i_goal)
    caminho = gerador_de_caminhos(c_space, start, goal)
    if caminho:
        print("Caminho gerado com sucesso:")
        print(caminho)
    else:
        print("Não foi possível gerar o caminho.")
else:
    print("Posição inicial do robô está fora dos limites do mapa.")


def plot_cspace(c_space, map_reso, caminho=None):
    fig, ax = plt.subplots(figsize=(10, 10))
    n_linhas, n_colunas = c_space.shape
    cax = ax.imshow(c_space, cmap='viridis', origin='lower')

    for i in range(n_linhas):
        for j in range(n_colunas):
            valor = c_space[i, j]
            ax.text(j, i, f'{valor}', ha='center',
                    va='center', color='black', fontsize=6)

    ax.set_xticks(np.arange(-0.5, n_colunas, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, n_linhas, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    ax.set_aspect('equal')

    # Se recebeu caminho, plota sobre o mapa
    if caminho is not None and len(caminho) > 0:
        # Extrai listas x e y para plotar (note que caminho é lista de (linha, coluna) = (i, j))
        xs = [p[1] for p in caminho]  # coluna (x)
        ys = [p[0] for p in caminho]  # linha (y)
        ax.plot(xs, ys, color='red', linewidth=2, marker='o', label='Caminho')

        ax.legend()

    plt.colorbar(cax, ax=ax, label='Potencial')
    plt.title('Mapa de Potenciais com valores nas células')
    plt.xlabel('Índice X (coluna)')
    plt.ylabel('Índice Y (linha)')
    plt.show()


plot_cspace(c_space, map_reso, caminho=caminho)
