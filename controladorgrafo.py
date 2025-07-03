import numpy as np
import time
from scipy.interpolate import splprep, splev
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from grafo import *  # Deve conter a variável 'caminho', map_x_min, map_y_min, map_reso

# --- Funções auxiliares ---


def normalizeAngle(angle):
    return np.mod(angle + np.pi, 2*np.pi) - np.pi


def indices_para_mundo(caminho, map_x_min, map_y_min, map_reso):
    """Converte índices da matriz para coordenadas reais"""
    caminho_mundo = []
    for i, j in caminho:
        x = map_x_min + i * map_reso + map_reso / 2
        y = map_y_min + j * map_reso + map_reso / 2
        caminho_mundo.append((x, y))
    return caminho_mundo


# --- Caminho em coordenadas do mundo ---
caminho_mundo = indices_para_mundo(caminho, map_x_min, map_y_min, map_reso)

# --- Criação da spline suave a partir do caminho discreto ---
xs, ys = zip(*caminho_mundo)
tck, u = splprep([xs, ys], s=1.0)
u_fino = np.linspace(0, 1, 200)
x_suave, y_suave = splev(u_fino, tck)

# --- Parâmetros do robô (Pioneer P3DX) ---
L = 0.381      # distância entre rodas (m)
r = 0.0975     # raio da roda (m)
maxv = 1.0     # velocidade linear máxima (m/s)
maxw = np.deg2rad(45)  # velocidade angular máxima (rad/s)

kr = 1.0       # ganho proporcional linear
kt = 2.0       # ganho proporcional angular
lookahead_dist = 0.15  # distância lookahead (m)

# --- Conexão com CoppeliaSim ---
print('Program started')
client = RemoteAPIClient()
sim = client.getObject('sim')
print('Connected to CoppeliaSim via ZMQ Remote API')

# --- Obtenção dos motores e handle do robô ---
robotname = '/PioneerP3DX'
robotHandle = sim.getObject(robotname)
leftMotor = sim.getObject(robotname + '/leftMotor')
rightMotor = sim.getObject(robotname + '/rightMotor')

# --- Inicialização do controle ---
index = 0
total_pontos = len(x_suave)

while index < total_pontos:
    pos = sim.getObjectPosition(robotHandle, -1)
    ori = sim.getObjectOrientation(robotHandle, -1)
    x, y, theta = pos[0], pos[1], ori[2]

    # Busca o ponto lookahead mais distante dentro do raio
    lookahead_index = index
    for i in range(index, total_pontos):
        dx = x_suave[i] - x
        dy = y_suave[i] - y
        dist = np.hypot(dx, dy)
        if dist > lookahead_dist:
            break
        lookahead_index = i

    # Atualiza índice para continuar progresso
    index = lookahead_index
    xg, yg = x_suave[lookahead_index], y_suave[lookahead_index]

    dx = xg - x
    dy = yg - y
    rho = np.hypot(dx, dy)

    # Controle proporcional
    heading_desired = np.arctan2(dy, dx)
    ang_error = normalizeAngle(heading_desired - theta)

    v = kr * rho
    w = kt * ang_error

    # Saturação
    v = np.clip(v, -maxv, maxv)
    w = np.clip(w, -maxw, maxw)

    # Velocidade das rodas
    wr = ((2 * v) + (w * L)) / (2 * r)
    wl = ((2 * v) - (w * L)) / (2 * r)

    sim.setJointTargetVelocity(rightMotor, wr)
    sim.setJointTargetVelocity(leftMotor, wl)

    time.sleep(0.05)

# --- Parada dos motores ---
sim.setJointTargetVelocity(rightMotor, 0)
sim.setJointTargetVelocity(leftMotor, 0)

print('Trajectory completed.')
