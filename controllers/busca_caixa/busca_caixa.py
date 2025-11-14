from controller import Robot, Motor, DistanceSensor, Supervisor
import math
import random
import time

TIME_STEP = 32
NUM_SENSORS = 8
MAX_SPEED = 6.28
NUM_BOXES = 20
THRESHOLD_MOV = 0.005
ESPERA_MOVIMENTO = 60
MAX_COLISOES_ENROSCO = 3
INTERVALO_ENROSCO_PASSOS = 94
ENROSCO_DIST_LIMIAR = 0.03
RANDOM_MOVEMENT_TIMER = 100
UNSTUCK_BACKOFF_STEPS = 20
UNSTUCK_TURN_STEPS = 40
UNSTUCK_FORWARD_STEPS = 30

passos_desde_primeira_colisao = -1
colisoes_recentes = 0
ultima_vl = 0.0
ultima_vr = 0.0
ultima_pos_robo = [0.0, 0.0, 0.0]


def initialize_motors(robot):
    left = robot.getDevice("left wheel motor")
    right = robot.getDevice("right wheel motor")

    left.setPosition(float('inf'))
    right.setPosition(float('inf'))

    left.setVelocity(0)
    right.setVelocity(0)

    return left, right


def initialize_sensors(robot):
    prox = []
    for i in range(NUM_SENSORS):
        name = f"ps{i}"
        sensor = robot.getDevice(name)
        sensor.enable(TIME_STEP)
        prox.append(sensor)
    return prox


def update_box_positions(supervisor, caixas, caixa_pos_ant):
    for i in range(NUM_BOXES):
        def_name = f"CAIXA{i+1}"
        caixas[i] = supervisor.getFromDef(def_name)

        if caixas[i] is not None:
            pos = caixas[i].getPosition()
            caixa_pos_ant[i][0] = pos[0]
            caixa_pos_ant[i][1] = pos[1]
            caixa_pos_ant[i][2] = pos[2]


def unstuck_routine(left_motor, right_motor, robot):
    print("Modo enrosco ativado! Executando rotina de desatolamento...")

    # Ré
    for _ in range(UNSTUCK_BACKOFF_STEPS):
        left_motor.setVelocity(-MAX_SPEED * 0.5)
        right_motor.setVelocity(-MAX_SPEED * 0.5)
        robot.step(TIME_STEP)


    sentido = 1 if random.randint(0, 1) == 0 else -1
    turn_speed = MAX_SPEED * (0.5 + random.random() * 0.5)

    for _ in range(UNSTUCK_TURN_STEPS):
        left_motor.setVelocity(sentido * turn_speed)
        right_motor.setVelocity(-sentido * turn_speed)
        robot.step(TIME_STEP)


    for _ in range(UNSTUCK_FORWARD_STEPS):
        left_motor.setVelocity(MAX_SPEED * 0.6)
        right_motor.setVelocity(MAX_SPEED * 0.6)
        robot.step(TIME_STEP)

    print("Rotina de desatolamento concluída.")


def handle_collision(left_motor, right_motor, supervisor, caixas, caixa_pos_ant):
    global passos_desde_primeira_colisao, colisoes_recentes, ultima_pos_robo

    print("COLIDIU!")

    for _ in range(ESPERA_MOVIMENTO):
        supervisor.step(TIME_STEP)


    for i in range(NUM_BOXES):
        if caixas[i] is None:
            continue

        nova_pos = caixas[i].getPosition()
        dx = abs(nova_pos[0] - caixa_pos_ant[i][0])
        dz = abs(nova_pos[2] - caixa_pos_ant[i][2])

        if dx > THRESHOLD_MOV or dz > THRESHOLD_MOV:
            print(f"Caixa {i+1} se moveu! DANÇA!")
            while True:
                left_motor.setVelocity(MAX_SPEED)
                right_motor.setVelocity(-MAX_SPEED)
                supervisor.step(TIME_STEP)

    print("Nenhuma caixa se moveu.")


    if passos_desde_primeira_colisao < 0:
        passos_desde_primeira_colisao = 0
        colisoes_recentes = 1
    else:
        passos_desde_primeira_colisao += 1
        colisoes_recentes += 1

    pos_robo = supervisor.getSelf().getPosition()
    dx = pos_robo[0] - ultima_pos_robo[0]
    dz = pos_robo[2] - ultima_pos_robo[2]
    dist = math.sqrt(dx * dx + dz * dz)

    if passos_desde_primeira_colisao == 1:
        ultima_pos_robo = list(pos_robo)

    if passos_desde_primeira_colisao > INTERVALO_ENROSCO_PASSOS:
        passos_desde_primeira_colisao = -1
        colisoes_recentes = 0

    elif colisoes_recentes >= MAX_COLISOES_ENROSCO and dist < ENROSCO_DIST_LIMIAR:
        unstuck_routine(left_motor, right_motor, supervisor)
        passos_desde_primeira_colisao = -1
        colisoes_recentes = 0


def random_movement(left_motor, right_motor, timer):
    global ultima_vl, ultima_vr

    timer -= 1
    if timer <= 0:
        v_left = random.random() * MAX_SPEED
        v_right = random.random() * MAX_SPEED

        ultima_vl = v_left
        ultima_vr = v_right

        left_motor.setVelocity(v_left)
        right_motor.setVelocity(v_right)

        timer = RANDOM_MOVEMENT_TIMER + random.randint(0, 100)

    return timer


def main():
    supervisor = Supervisor()
    random.seed(time.time())

    left_motor, right_motor = initialize_motors(supervisor)
    prox = initialize_sensors(supervisor)

    caixas = [None] * NUM_BOXES
    caixa_pos_ant = [[0, 0, 0] for _ in range(NUM_BOXES)]
    update_box_positions(supervisor, caixas, caixa_pos_ant)

    timer = RANDOM_MOVEMENT_TIMER

    while supervisor.step(TIME_STEP) != -1:
        print("Timer:", timer)

        bateu = False
        for sensor in prox:
            val = sensor.getValue()
            if val > 80.0:
                bateu = True

        if bateu:
            handle_collision(left_motor, right_motor, supervisor, caixas, caixa_pos_ant)
            timer = 0

        timer = random_movement(left_motor, right_motor, timer)


if __name__ == "__main__":
    main()
