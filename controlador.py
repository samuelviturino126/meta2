import numpy as np
import time
from scipy.interpolate import splprep, splev
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from campos_potenciais_meta2 import *


def normalizeAngle(angle):
    return np.mod(angle + np.pi, 2*np.pi) - np.pi


def indices_para_mundo(caminho, map_x_min, map_y_min, map_reso):
    caminho_mundo = []
    for linha, coluna in caminho:
        x = map_x_min + coluna * map_reso + map_reso / 2
        y = map_y_min + linha * map_reso + map_reso / 2
        caminho_mundo.append((x, y))
    return caminho_mundo


# --- Aqui você deve carregar/definir seu caminho discreto em índices ---
# Exemplo fictício, substitua pelo seu caminho real:

# Converte para coordenadas reais
caminho_mundo = indices_para_mundo(caminho, map_x_min, map_y_min, map_reso)

# Cria spline suavizada do caminho
xs, ys = zip(*caminho_mundo)
tck, u = splprep([xs, ys], s=1.0)
u_fino = np.linspace(0, 1, 200)
x_suave, y_suave = splev(u_fino, tck)

# Parâmetros do robô (Pioneer P3DX)
L = 0.381      # distância entre rodas
r = 0.0975     # raio da roda
maxv = 1.0     # velocidade máxima linear (m/s)
maxw = np.deg2rad(45)  # velocidade máxima angular (rad/s)

kr = 1.0       # ganho linear
kt = 2.0       # ganho angular
lookahead_dist = 0.15  # distância para avançar no ponto seguinte

print('Program started')
client = RemoteAPIClient()
sim = client.getObject('sim')
print('Connected to CoppeliaSim via ZMQ Remote API')

robotname = '/PioneerP3DX'
robotHandle = sim.getObject(robotname)
leftMotor = sim.getObject(robotname + '/leftMotor')
rightMotor = sim.getObject(robotname + '/rightMotor')

index = 0
total_pontos = len(x_suave)

while index < total_pontos:
    pos = sim.getObjectPosition(robotHandle, -1)
    ori = sim.getObjectOrientation(robotHandle, -1)
    x, y, theta = pos[0], pos[1], ori[2]

    # Ponto alvo da spline
    xg, yg = x_suave[index], y_suave[index]

    dx = xg - x
    dy = yg - y
    rho = np.hypot(dx, dy)

    # Avança para próximo ponto se estiver perto o suficiente
    if rho < lookahead_dist:
        index += 1
        continue

    # Controle proporcional simples
    v = kr * (dx * np.cos(theta) + dy * np.sin(theta))
    ang_error = np.arctan2(dy, dx) - theta
    w = kt * normalizeAngle(ang_error)

    # Limita velocidades
    v = np.clip(v, -maxv, maxv)
    w = np.clip(w, -maxw, maxw)

    # Converte para velocidades das rodas
    wr = ((2 * v) + (w * L)) / (2 * r)
    wl = ((2 * v) - (w * L)) / (2 * r)

    sim.setJointTargetVelocity(rightMotor, wr)
    sim.setJointTargetVelocity(leftMotor, wl)

    time.sleep(0.05)

# Para motores ao final do caminho
sim.setJointTargetVelocity(rightMotor, 0)
sim.setJointTargetVelocity(leftMotor, 0)

print('Trajectory completed.')
