from funciones_2 import control, gestionar_inputs, ser
import threading



# Crear threads
control_thread = threading.Thread(target=control)
input_thread = threading.Thread(target=gestionar_inputs)

# Iniciar threads
control_thread.start()
input_thread.start()

# Esperar a que los threads terminen
control_thread.join()
input_thread.join()


# Cerrar conexión serial
ser.close()

print("Programa terminado.")
