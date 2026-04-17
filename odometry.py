from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.tools import wait
from umath import sin, cos, atan2, sqrt, radians, degrees

# ================= 1. CONFIGURAÇÕES GERAIS (ALTERE AQUI, PREÇO 100 RS) =================
VEL_THEFLASH = 1000      
VEL_DESVIO = 100      
VEL_ACELERACAO = 700      
DIST_MARGEM = 140      
DIST_RECUO = -5      

# Ganhos PID
PID_GIRO = {"kp": 4.5, "ki": 0.08, "kd": 2.2, "limite_i": 5}



OBSTACULOS = [{"x": (-100, 100), "y": (300, 500)}]

# ================= 2. HARDWARE E ESTADO =================
hub = PrimeHub()
motor_esq = Motor(Port.E, Direction.COUNTERCLOCKWISE)
motor_dir = Motor(Port.F)
drive = DriveBase(motor_esq, motor_dir, wheel_diameter=60, axle_track=110)
drive.settings(straight_speed=VEL_THEFLASH, straight_acceleration=VEL_ACELERACAO)

x_atual, y_atual = 0.0, 0.0
dist_anterior = 0.0
erro_anterior = 0
integral = 0

# ================= 3. MOTORES DE CÁLCULO =================

def atualizar_odometria():
    global x_atual, y_atual, dist_anterior
    dist_total = drive.distance()
    delta_dist = dist_total - dist_anterior
    theta = radians(hub.imu.heading())
    x_atual += delta_dist * sin(theta)
    y_atual += delta_dist * cos(theta)
    dist_anterior = dist_total

def calcular_pid(erro):
    global erro_anterior, integral
    P = PID_GIRO["kp"] * erro
    integral += erro
    if abs(integral) > PID_GIRO["limite_i"]:
        integral = (integral / abs(integral)) * PID_GIRO["limite_i"]
    I = PID_GIRO["ki"] * integral
    D = PID_GIRO["kd"] * (erro - erro_anterior)
    erro_anterior = erro
    return P + I + D

# ================= 4. NAVEGAÇÃO FLUIDA =================

def andar_ate(x_alvo, y_alvo, velocidade, detectar=True):
    global x_atual, y_atual, integral, erro_anterior, dist_anterior
   
    drive.reset()
    dist_anterior = 0
    integral = 0
    erro_anterior = 0

    while True:
        atualizar_odometria()
        dx, dy = x_alvo - x_atual, y_alvo - y_atual
        distancia_restante = sqrt(dx**2 + dy**2)

        if distancia_restante < 20:
            drive.stop()
            return "CHEGOU"

        angulo_alvo = degrees(atan2(dx, dy))
        angulo_atual = hub.imu.heading()
        erro_angulo = (angulo_alvo - angulo_atual + 180) % 360 - 180

        correcao = calcular_pid(erro_angulo)
       
        # Reduz velocidade se a curva for muito fechada para não derrapar
        v_ajustada = velocidade if abs(erro_angulo) < 30 else velocidade * 0.4
       
        drive.drive(v_ajustada, correcao)
        wait(10)

        if detectar:
            for obs in OBSTACULOS:
                if (obs["x"][0] - 10 <= x_atual <= obs["x"][1] + 10 and
                    obs["y"][0] - 10 <= y_atual <= obs["y"][1] + 10):
                    drive.stop()
                    return obs

def IR_PARA(x_final, y_final, angulo_final=None):
    resultado = andar_ate(x_final, y_final, VEL_THEFLASH, detectar=True)

    if resultado != "CHEGOU":
        print("Obstáculo detectado! Contornando...")
        drive.straight(DIST_RECUO)
        atualizar_odometria()
       
        # Cálculo de desvio mais curto (contorna pela lateral mais próxima da quina)
        ponto_desvio_x = resultado["x"][1] + DIST_MARGEM
        ponto_desvio_y = y_atual
       
        # Passo 1: Lado do obstáculo
        andar_ate(ponto_desvio_x, ponto_desvio_y, VEL_DESVIO, detectar=False)
        # Passo 2: Frente do obstáculo (passa a quina Y)
        andar_ate(ponto_desvio_x, resultado["y"][1] + DIST_MARGEM, VEL_DESVIO, detectar=False)
       
        # Tenta novamente o destino final
        return IR_PARA(x_final, y_final, angulo_final)

    if angulo_final is not None:
        erro_final = (angulo_final - hub.imu.heading() + 180) % 360 - 180
        drive.turn(erro_final)
   
    print("Cheguei!")

# ================= 5. EXECUÇÃO =================
hub.imu.reset_heading(0)
drive.reset()

IR_PARA(0, 1000, angulo_final=0)

print("ai ai o robô me mordeu")
