from pynput import keyboard
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import threading
import matplotlib.pyplot as plt

# Conecta ao CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject('sim')
# Obtém os motores
left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')
robo = sim.getObject('/PioneerP3DX')
sensor = sim.getObject('/PioneerP3DX/sonar')
#velocidade de cada roda
vl = 0.0
vr = 0.0
#ajustes para controlar as rodas em movimento contínuo
step = 0.5
vel_base = 1.0
# Variáveis para coleta de dados do sensor
tempo = 0.0
intervalo = 0.1
tempos = []
distancias = []
posicoes_x = []
posicoes_y = []
coletando = True

def coletar_dados():
    global tempo
    while coletando:
        # Posição do robô em relação ao mundo (-1)
        pos = sim.getObjectPosition(robo, -1)
        posicoes_x.append(pos[0])
        posicoes_y.append(pos[1])
        # Sensor de distância
        estado, ponto, *nusados = sim.readProximitySensor(sensor)
        #estado (true se detectar algo, false se nao)
        #ponto (coordenadas relativas ao sensor)
        if estado:
            distancia = (ponto[0]**2 + ponto[1]**2 + ponto[2]**2)**0.5
        else:
            distancia = None
        distancias.append(distancia)
        tempos.append(tempo)

        tempo += intervalo
        time.sleep(intervalo)

def set_velocidade():
    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)

# Função chamada ao pressionar uma tecla
def on_press(key):
    global vl, vr
    try:
        if key.char == 'w':
            vl = vr = vel_base
        elif key.char == 't':
            sim.startSimulation()
        elif key.char == 's':
            vl = vr = -vel_base
        elif key.char == 'a':
             vl -= step
             vr += step
        elif key.char == 'd':
             vl += step
             vr -= step
        elif key.char == 'p':
            sim.stopSimulation()
        set_velocidade()
    except AttributeError:
        pass  # Ignora teclas especiais

# Função chamada ao soltar uma tecla
def on_release(key):
    if key == keyboard.Key.esc:
        vl = vr = 0.0
        set_velocidade()
        global coletando
        coletando = False
        return False  # Encerra o listener

# Iniciar thread de coleta
thread_dados = threading.Thread(target=coletar_dados)
thread_dados.start()
# Inicia o listener de teclado
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
    thread_dados.join()

    # Plotar gráficos
    plt.figure(figsize=(12, 5))

    # Distância ao longo do tempo
    plt.subplot(1, 2, 1)
    t_validas = [t for t, d in zip(tempos, distancias) if d is not None]
    d_validas = [d for d in distancias if d is not None]
    plt.plot(t_validas, d_validas)
    plt.xlabel("Tempo (s)")
    plt.ylabel("Distância (m)")
    plt.title("Sensor de Alcance")

    # Trajetória (x, y)
    plt.subplot(1, 2, 2)
    plt.plot(posicoes_x, posicoes_y, marker='o')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Trajetória do Robô")

    plt.tight_layout()
    plt.show()
