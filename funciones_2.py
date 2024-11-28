import serial
import time
import cv2
import numpy as np
import threading
from simple_pid import PID
from queue import Queue
import math


ser = serial.Serial("COM3", baudrate=38400, timeout=1)
time.sleep(2)  # Tiempo para establecer la conexión

thread1_done = threading.Event()
thread2_done = threading.Event()
modo_de_juego = None
estrategia = "Manual"


def calcular_angulo(p1, p2, p3):
    angle1 = math.degrees(math.atan2(p1[1] - p2[1], p1[0] - p2[0]))
    angle2 = math.degrees(math.atan2(p1[1] - p3[1], p1[0] - p3[0]))

    theta = round(angle1 - angle2, 2)
    
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



def start(PID_ang, PID_lin, theta, dist):



    if abs(theta) < 10 and abs(dist) < 20:
        print("Llego")

    else:
        # Control angular
        control_R = PID_ang(theta)
        control_L = -PID_ang(theta)

        # Corrección lineal en función del ángulo

        if np.sqrt(abs(theta)) < 3:
            control_R += PID_lin(dist)
            control_L += PID_lin(dist)

        # Ajuste del setpoint de distancia dinámico
        PID_lin.setpoint = max(5, dist * 0.5)
    
    print("Angulo: " + str(theta))
    print("Distancia: " + str(dist))
    return (control_R, control_L)




def control():

    global ser
    global modo_de_juego

    output_queue = Queue()

    pid_ang = PID(1.8, 0.02, 0.1, setpoint=0)
    pid_lin = PID(1.5, 0, 0, setpoint=20)

    pid_ang.output_limits = (-150, 150)
    pid_lin.output_limits = (-150, 150)

    vid = cv2.VideoCapture(1, cv2.CAP_DSHOW)

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

    low_blue = np.array([110, 100, 100])
    high_blue = np.array([130, 255, 255])

    """Colocamos esta deteccion de color fuera del while ya que solo nos interesa calcularla 1 vez para saber dónde estan los arcos,
    pero en realidad sólo nos interesa hacerlo una pura vez. De esta forma no corremos el riesgo que al tapar el arco o al haber alguien
    con el mismo color del arco que se pierda la posición de este."""

    centros_purple = detectar_color(low_purple, high_purple, "purple", img, "morado", (255, 0, 255))
    centros_blue = detectar_color(low_blue, high_blue, "blue", img, "azul", (50, 255, 50))

    centro_arco_morado = centros_purple[0]
    centro_arco_azul = centros_blue[0]

    modo_de_juego = "pelota" #Cambiar directamente esta linea para cambiar el modo por ahora. Es más seguro de que funcione
    modo_de_juego_anterior = modo_de_juego

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


        # Calcular ángulo y distancia si hay suficientes centros
        if centros_amarillo and centros_pink and centros_naranja:
            centro_amarillo = centros_amarillo[0]
            centro_pink = centros_pink[0]
            centro_naranja = centros_naranja[0]



            # if centro_amarillo[0] > centro_naranja[0]:
            #     modo_de_juego = "pelota"
            # else:
            #     modo_de_juego = "arco_m"

            ################################ Probar una vez tuneado el pid #####################################


            # Calcular ángulo
            angulo_pelota = calcular_angulo(centro_naranja, centro_pink, centro_amarillo) #Esto puede que esté alreves
            angulo_arco_m = calcular_angulo(centro_naranja, centro_pink, centro_arco_morado)
            angulo_arco_a = calcular_angulo(centro_naranja, centro_pink, centro_arco_azul)

            angulo_centro = calcular_angulo(centro_naranja, centro_pink, (ancho/2, alto/2))

            theta_p = angulo_pelota[0]
            dist_p = angulo_pelota[1]

            theta_am = angulo_arco_m[0]
            dist_am = angulo_arco_m[1]

            theta_aa = angulo_arco_a[0]
            dist_aa = angulo_arco_a[1]

            theta_centro = angulo_centro[0]
            dist_centro = angulo_centro[1]          


            cv2.line(img, centro_naranja, centro_pink, (0, 0, 0), 2)


            if modo_de_juego == "pelota":
                if modo_de_juego_anterior != "pelota":
                    pid_ang.integral = 0
                    pid_ang.last_output = None
                    pid_ang.last_input = None

                    pid_lin.integral = 0
                    pid_lin.last_output = None
                    pid_lin.last_input = None

                r1Ar, r1Al = start(pid_ang, pid_lin, theta_p, dist_p)
                cv2.line(img, centro_naranja, centro_amarillo, (0, 0, 0), 2)


            if modo_de_juego == "arco_m":
                if modo_de_juego_anterior != "arco_m":
                    pid_ang.integral = 0 ###################### Crear funcionar para reiniciar el PID######################
                    pid_ang.last_output = None
                    pid_ang.last_input = None

                    pid_lin.integral = 0
                    pid_lin.last_output = None
                    pid_lin.last_input = None

                r1Ar, r1Al = start(pid_ang, pid_lin, theta_am, dist_am)
                cv2.line(img, centro_naranja, centro_arco_morado, (0, 0, 0), 2)

            if modo_de_juego == "arco_a":
                if modo_de_juego_anterior != "arco_a":
                    pid_ang.integral = 0 ###################### Crear funcionar para reiniciar el PID######################
                    pid_ang.last_output = None
                    pid_ang.last_input = None

                    pid_lin.integral = 0
                    pid_lin.last_output = None
                    pid_lin.last_input = None

                r1Ar, r1Al = start(pid_ang, pid_lin, theta_aa, dist_aa)
                cv2.line(img, centro_naranja, centro_arco_morado, (0, 0, 0), 2)

            if modo_de_juego == "centro":
                if modo_de_juego_anterior != "centro":
                    pid_ang.integral = 0
                    pid_ang.last_output = None
                    pid_ang.last_input = None

                    pid_lin.integral = 0
                    pid_lin.last_output = None
                    pid_lin.last_input = None

                r1Ar, r1Al = start(pid_ang, pid_lin, theta_centro, dist_centro)
                cv2.line(img, centro_naranja, (ancho//2, alto//2), (0, 0, 0), 2)
            
            output_queue.put((r1Ar, r1Al))

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
            print(f"Motor A: {mensaje_a}")
            print(f"Motor B: {mensaje_b}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag = True
            break

        modo_de_juego_anterior = modo_de_juego
        # Señalar que Thread 1 terminó su ciclo
        thread1_done.set()

        # Esperar a que Thread 2 termine antes de iniciar el siguiente ciclo
        thread2_done.wait()
        thread2_done.clear()
    vid.release()
    cv2.destroyAllWindows()
    ser.close()



def gestionar_inputs():
    global modo_de_juego

    while True:
        thread1_done.wait()
        thread1_done.clear()

        user_input = input("Ingresa un comando: ")
        modo_de_juego = user_input


        thread2_done.set()
 
def go_center():
    pass

def move_to_ball():
    pass

def move_ball_straight():
    pass

def ball_to_goal():
    pass

def kick_ball_to_goal():
    pass

def pid_controler(mesure, setpoint=0, kp=1, ki=0, kd=0, dt=0.1, error=[]):
    """
    Función de control PID, que recibe una variable a controlar (mesure) y un setpoint al que
    se quiere llegar. En la que los coeficientes (kp, ki, kd) son independientes entre sí.
    """
    # Gestion lista de errores
    error.append(setpoint - mesure)
    if len(error) > 10:
        error.pop(0)
    
    # Cálculo de las componentes del controlador PID
    P = kp * error[-1]
    I = ki * sum(error) * dt
    if len(error) > 1:
        D = kd * (error[-1] - error[-2]) / dt
    else:
        D = 0
    
    # Cálculo de la señal de control
    u = P + I + D
    
    return u, error