import serial
import time
import cv2
import numpy as np
import threading
from simple_pid import PID
from queue import Queue
import math


exit_flag = False  # Señal para terminar los threads
lock = threading.Lock()  # Para asegurar el acceso seguro a las variables compartidas
output_queue = Queue()
ser = serial.Serial("COM3", baudrate=38400, timeout=1)
time.sleep(2)  # Tiempo para establecer la conexión

theta = 0
dist = 0

modo_de_juego = "pelota"

theta_p = 0
dist_p = 0
theta_am = 0
dist_am = 0

theta_centro = 0
dist_centro = 0

r1Ar = 0
r1Al = 0
output_queue = Queue()
ancho = 0
alto = 0



def calcular_angulo(p1, p2, p3):
    angle1 = math.degrees(math.atan2(p1[1] - p2[1], p1[0] - p2[0]))
    angle2 = math.degrees(math.atan2(p1[1] - p3[1], p1[0] - p3[0]))

    theta = angle1 - angle2
    
    # Descomponemos los puntos en coordenadas
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    # Calculamos las distancias
    a = math.sqrt((x2 - x3) ** 2 + (y2 - y3) ** 2)  # Distancia de adelante a la pelota
    b = math.sqrt((x1 - x3) ** 2 + (y1 - y3) ** 2)  # Distancia de atras a la pelota
    c = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)  # Distancia entre cosas del robot

    return (theta, a, b, c)




def detectar_color(low_color, high_color, color_name, img, nombre, color_caja=(0, 0, 0)):
    # Convertimos la imagen a HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Creamos la máscara para el color
    color_mask = cv2.inRange(img_hsv, low_color, high_color)
    
    # Aplicamos la máscara a la imagen original
    img_masked = cv2.bitwise_and(img, img, mask=color_mask)
    
    # Encontramos los contornos en la máscara
    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Lista para almacenar los centros de cada objeto detectado
    centros = []
    
    for contour in contours:
        # Ignorar contornos muy pequeños
        if cv2.contourArea(contour) > 100:
            # Obtenemos el bounding box de cada contorno
            x, y, w, h = cv2.boundingRect(contour)
            prom_x = int(x + w / 2)
            prom_y = int(y + h / 2)
            
            # Agregamos el centro a la lista
            centros.append((prom_x, prom_y))
            
            # Dibujamos el bounding box y el nombre del color
            cv2.rectangle(img_masked, (x, y), (x + w, y + h), color_caja, 2)
            cv2.rectangle(img, (x, y), (x + w, y + h), color_caja, 2)
            cv2.putText(img_masked, color_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_caja, 1, cv2.LINE_AA)
            cv2.putText(img, color_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_caja, 1, cv2.LINE_AA)
            cv2.circle(img, (prom_x, prom_y), 5, (0, 0, 0), -1)
    
    # Mostramos las imágenes
    # cv2.imshow(nombre, img_masked) #No es necesario que se vean las ventanas individuales de cada color
    cv2.imshow('original', img)
    
    # Retornamos la lista de centros
    return centros




def start():
    global r1Ar
    global r1Al
    global modo_de_juego

    global theta_p
    global dist_p
    
    global theta_am
    global dist_am
    
    global theta_centro
    global dist_centro

    global modo_de_juego
    global output_queue


    pid_ang = PID(1.5, 0, 0, setpoint=0)
    pid_lin = PID(1.5, 0, 0, setpoint=20)

    pid_ang_2 = PID(0.5, 0, 0, setpoint=0)

    ################ Faltaría tunear los PDI ###############################

    pid_ang.output_limits = (-100, 100)
    pid_lin.output_limits = (-100, 100)

    pid_ang.sample_time = 0.2
    pid_lin.sample_time = 0.2

    pid_ang_2.sample_time = 0.2

    while True:
        theta = 0
        dist = 0

        if modo_de_juego == "pelota":
            theta = theta_p
            dist = dist_p
        if modo_de_juego == "arco_m":
            theta = theta_am
            dist = dist_am
        if modo_de_juego == "centro":
            theta = theta_centro
            dist = dist_centro

        if modo_de_juego == "goal":
            theta1 = theta_p
            dist1 = dist_p
            theta2 = theta_am
            dist2 = dist_am
        


        if modo_de_juego == "pelota" or modo_de_juego == "arco_m" or modo_de_juego == "centro":
            objetivo(theta, dist, pid_ang, pid_lin, 
                     lambda: print("LLego")
                     )

        elif modo_de_juego == "goal":
            objetivo(theta1, dist1, pid_ang, pid_lin, 
                     lambda: objetivo(theta2, dist2, pid_ang_2, pid_lin, 
                                      lambda: print("Objetivo alcanzado: centro"), nombre="arco"
                                      ), nombre="pelota"
                     )

        time.sleep(0.2)



def procesar_camara():
    global modo_de_juego

    global theta_p
    global dist_p
    global theta_am
    global dist_am
    global ser
    global output_queue
    global ancho
    global alto

    global theta_centro
    global dist_centro

    vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    ancho = 600
    alto = 500

    # Definimos los rangos de los colores
    low_g = np.array([40, 100, 100])
    high_g = np.array([80, 255, 255])

    low_orange = np.array([5, 130, 100])
    high_orange = np.array([20, 255, 255])

    low_yellow = np.array([20, 50, 5])
    high_yellow = np.array([40, 255, 255])

    low_pink = np.array([135, 110, 110])
    high_pink = np.array([165, 255, 255])

    low_purple = np.array([135, 5, 20])
    high_purple= np.array([165, 130, 130])



    modo_de_juego = "pelota" #Cambiar directamente esta linea para cambiar el modo por ahora. Es más seguro de que funcione

    pid_ang = PID(1.5, 0.001, 0.005, setpoint=0)
    pid_lin = PID(1.5, 0, 0, setpoint=20)

    pid_ang_2 = PID(0.5, 0.001, 0.005, setpoint=0)

    ################ Faltaría tunear los PDI ###############################

    pid_ang.output_limits = (-100, 100)
    pid_lin.output_limits = (-100, 100)

    pid_ang.sample_time = 0.1
    pid_lin.sample_time = 0.1

    pid_ang_2.sample_time = 0.1


    while True:
        # Leer frame de la cámara
        ret, img = vid.read()

        if not ret:
            print("No se pudo leer de la cámara.")
            break

        # Procesar colores
        centros_naranja = detectar_color(low_orange, high_orange, "naranja", img, "naranja", (255, 255, 255))
        centros_amarillo = detectar_color(low_yellow, high_yellow, "yellow", img, "amarillo", (0, 255, 255))
        centros_pink = detectar_color(low_pink, high_pink, "pink", img, "pink", (255, 0, 255))
        centro_arco_purple = detectar_color(low_purple, high_purple, "purple", img, "morado", (255, 0, 255))

        # Calcular ángulo y distancia si hay suficientes centros
        if centros_amarillo and centros_pink and centros_naranja and centro_arco_purple:
            centro_amarillo = centros_amarillo[0]
            centro_pink = centros_pink[0]
            centro_naranja = centros_naranja[0]
            centro_arco_morado = centro_arco_purple[0]


            # if centro_amarillo[0] > centro_pink[0]:
            #     modo_de_juego = "pelota"
            # else:
            #     modo_de_juego = "arco_m"

            ################################ Probar una vez tuneado el pid #####################################


            # Calcular ángulo
            angulo_pelota = calcular_angulo(centro_naranja, centro_pink, centro_amarillo) #Esto puede que esté alreves
            angulo_arco_m = calcular_angulo(centro_naranja, centro_pink, centro_arco_morado)
            angulo_centro = calcular_angulo(centro_naranja, centro_pink, (ancho/2, alto/2))
            angulo_arriba = calcular_angulo(centro_naranja, centro_pink, (centro_naranja[0], 1000))
            angulo_abajo = calcular_angulo(centro_naranja, centro_pink, (centro_naranja[0], 0))
            angulo_lateral = calcular_angulo(centro_naranja, centro_pink, (0, centro_naranja[1]))

            theta_p = angulo_pelota[0]
            dist_p = angulo_pelota[1]

            theta_am = angulo_arco_m[0]
            dist_am = angulo_arco_m[1]

            theta_centro = angulo_centro[0]
            dist_centro = angulo_centro[1]

            theta_arriba = angulo_arriba[0]
            dist_arriba = angulo_arriba[1]

            theta_abajo = angulo_abajo[0]
            dist_abajo = angulo_abajo[1]

            theta_lateral = angulo_lateral[0]
            dist_lateral = angulo_lateral[1]


            cv2.line(img, centro_naranja, centro_pink, (0, 0, 0), 2)


            if centro_naranja[0] >= centro_amarillo[0] - 30:
                if abs(centro_naranja[1] - centro_amarillo[1]) <= 50:
                    if centro_naranja[1] >= 250:
                        objetivo(theta_abajo, dist_abajo, pid_ang, pid_lin, lambda: print("bajando"), tolerancia=20)
                        cv2.line(img, centro_naranja, (centro_naranja[0], 0), (0, 0, 0), 2)

                    else:
                        objetivo(theta_arriba, dist_arriba, pid_ang, pid_lin, lambda: print("subiendo"), tolerancia=20)
                        
                        cv2.line(img, centro_naranja, (centro_naranja[0], 1000), (0, 0, 0), 2)

                else:
                    objetivo(theta_lateral, dist_lateral, pid_ang, pid_lin, lambda: print("llendo a nuestro lado"), tolerancia=20)
                    cv2.line(img, centro_naranja, (0, centro_naranja[1]), (0, 0, 0), 2)

            else:

                objetivo(theta_p, dist_p, pid_ang, pid_lin, 
                        lambda: objetivo(theta_am, dist_am, pid_ang_2, pid_lin, 
                                        lambda: print("Objetivo alcanzado: centro"), nombre="arco"
                                        ), nombre="pelota", tolerancia=7
                        )
                cv2.line(img, centro_naranja, centro_amarillo, (0, 0, 0), 2)
                cv2.line(img, centro_naranja, centro_arco_morado, (0, 0, 0), 2)






        else:
            print("No se encontraron centros suficientes para calcular el ángulo.")

        # Mostrar imagen procesada
        cv2.imshow('original', img)

        # Leer valores de la cola y enviar comandos
        if not output_queue.empty():
            r1Ar, r1Al = output_queue.get()
            mensaje_a = f"A{int(r1Ar)};"
            mensaje_b = f"B{int(r1Al)};"
            ser.write(mensaje_a.encode())
            ser.write(mensaje_b.encode())
            print(f"Enviado al Motor A: {mensaje_a}")
            print(f"Enviado al Motor B: {mensaje_b}")
            time.sleep(0.1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag = True
            break


    vid.release()
    cv2.destroyAllWindows()
    ser.close()



def gestionar_inputs():
    global modo_de_juego

    while True:
        user_input = input("Ingresa un comando: ")
        modo_de_juego = user_input
            
        # Aquí puedes procesar otros comandos si es necesario
        
def pid_controler(mesure, setpoint=0, kp=1, ki=0, kd=0, dt=0.1, error=[]):
    """
    Función de control PID, que recibe una variable a controlar (mesure) y un setpoint al que
    se quiere llegar. En la que los coeficientes (kp, ki, kd) son independientes entre sí.
    """
    # Calculamos el error actual y lo agregamos a la lista de errores
    error.append(setpoint - mesure)
    
    # Limitamos la cantidad de errores almacenados
    if len(error) > 10:
        error.pop(0)
    
    P = kp * error[-1]
    I = ki * sum(error) * dt
    if len(error) > 1:
        D = kd * (error[-1] - error[-2]) / dt 
    else:
        D = 0
        
    u = P + I + D
    
    return u, error

def objetivo(theta, dist, pid_ang, pid_lin, funcion_final, nombre = "", tolerancia = 5):
    global r1Ar, r1Al

    # Verificar si el robot ha alcanzado el objetivo
    if abs(dist) < 25:
        r1Ar = 0
        r1Al = 0
        funcion_final()  # Ejecutar la función final
    else:
        # Control angular
        control_R = pid_ang(theta)
        control_L = -pid_ang(theta)

        # Corrección lineal si el ángulo es pequeño
        if abs(theta) <= tolerancia:
            control_R += pid_lin(dist)
            control_L += pid_lin(dist)

        # Actualización de las señales PWM
        r1Ar = control_R # Limitar a rango de -100 a 100
        r1Al = control_L

    print(f"Ángulo: {theta}, Distancia: {dist}")
    output_queue.put((r1Ar, r1Al))
    print(nombre)
    