from funciones import start, procesar_camara, gestionar_inputs, ser
import threading



# Crear threads
control_thread = threading.Thread(target=start)
camera_thread = threading.Thread(target=procesar_camara)
input_thread = threading.Thread(target=gestionar_inputs)


# Iniciar threads
control_thread.start()
camera_thread.start()
input_thread.start()

# Esperar a que los threads terminen
control_thread.join()
camera_thread.join()
input_thread.join()


# Cerrar conexión serial
ser.close()

print("Programa terminado.")
