from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

# Conecta ao CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject('sim')
map_reso = 0.125  # Defino em 12,5cm quadrados por célula
# vetor de rotação (para rotacionar os objetos pelo angulo)


def rotatedVec2d(vec_in, angle):
    return [
        math.cos(angle) * vec_in[0] - math.sin(angle) * vec_in[1],
        math.sin(angle) * vec_in[0] + math.cos(angle) * vec_in[1]
    ]


def get_rectangle_vertices(sim, handle):
    """Retorna os 4 vértices do objeto como [(x1, y1), ..., (x4, y4)]"""
    sizes, _ = sim.getShapeBB(handle)  # largura, altura, profundidade
    largura, altura = sizes[0], sizes[1]

    # vértices relativos ao centro
    vertices_rel = [[x/2, y/2] for y in [altura, -altura]
                    for x in [largura, -largura]]
    vertices_rel[1:] = vertices_rel[2:] + [vertices_rel[1]]  # reordenamento

    pos = sim.getObjectPosition(handle)
    gamma = sim.getObjectOrientation(handle)[-1]  # ângulo em Z
    vertices_abs = [rotatedVec2d(v, gamma) for v in vertices_rel]
    return [[v[0] + pos[0], v[1] + pos[1]] for v in vertices_abs]


def expandir_vertices(vertices, fator=5 * map_reso):
    """Expande os vértices do retângulo em torno do centro"""
    cx = sum(v[0] for v in vertices) / 4
    cy = sum(v[1] for v in vertices) / 4
    vertices_expandido = []
    for x, y in vertices:
        dx = x - cx
        dy = y - cy
        d = math.sqrt(dx**2 + dy**2)
        if d == 0:
            vertices_expandido.append((x, y))
        else:
            fator_escala = (d + fator) / d
            ex = cx + dx * fator_escala
            ey = cy + dy * fator_escala
            vertices_expandido.append((ex, ey))
    return vertices_expandido


# agora obtemos o mapa baseado no Floor
# Obtém vértices do chão
floor_handle = sim.getObject('/Floor')
vertices_floor = get_rectangle_vertices(sim, floor_handle)

# Calcula limites do mapa
xs, ys = zip(*vertices_floor)
map_x_min, map_x_max = min(xs), max(xs)
map_y_min, map_y_max = min(ys), max(ys)

map_larg = map_x_max - map_x_min
map_alt = map_y_max - map_y_min

# agora vou ignorar o Chão, para plotar o mapa apenas com os obstáculos destacados e o Pionner sendo um deles

# Handles a serem ignorados (algum que quisermos utilizar como o próprio robo nos campos potenciais no futuro e piso)
nomes_a_ignorar = ['/Floor', '/PioneerP3DX']
handles_ignorados = []
for nome in nomes_a_ignorar:
    h = sim.getObject(nome)
    handles_ignorados.append(h)

# Coleta todos os objetos do tipo SHAPE
todos_objetos = sim.getObjectsInTree(
    sim.handle_scene, sim.sceneobject_shape, 0b00000010)

# Remove os objetos ignorados
obstaculos = [h for h in todos_objetos if h not in handles_ignorados]

# Cria matriz do C-Space: linhas = largura, colunas = altura
c_space = np.zeros((int(map_larg / map_reso), int(map_alt / map_reso)))


def ponto_para_indice(x, y):
    """Converte coordenadas do mundo real para índices da matriz c_space"""
    i = int((x - map_x_min) / map_reso)
    j = int((y - map_y_min) / map_reso)
    return i, j


def marcar_obstaculo_na_matriz(c_space, vertices):
    """Marca na matriz as células ocupadas pelo polígono definido pelos vértices"""
    path = mplPath.Path(vertices)  # cria polígono dos vértices

    # Para acelerar, percorremos só a região do polígono (bounding box)
    xs, ys = zip(*vertices)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    i_min, j_min = ponto_para_indice(min_x, min_y)
    i_max, j_max = ponto_para_indice(max_x, max_y)

    # Ajusta para limites da matriz (evitar index error)
    i_min = max(i_min, 0)
    j_min = max(j_min, 0)
    i_max = min(i_max, c_space.shape[0] - 1)
    j_max = min(j_max, c_space.shape[1] - 1)

    for i in range(i_min, i_max + 1):
        for j in range(j_min, j_max + 1):
            # Coordenadas do centro da célula
            x = map_x_min + i * map_reso + map_reso/2
            y = map_y_min + j * map_reso + map_reso/2
            if path.contains_point((x, y)):
                c_space[i, j] = 1


# marcamos os obstaculos na matriz
for obs_handle in obstaculos:
    vertices = get_rectangle_vertices(sim, obs_handle)
    vertices = expandir_vertices(vertices)
    marcar_obstaculo_na_matriz(c_space, vertices)
