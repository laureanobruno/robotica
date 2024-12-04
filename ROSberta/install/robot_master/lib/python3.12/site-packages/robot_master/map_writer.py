class MapWriter:
    def __init__(self, file_name='map.txt'):
        """
        Inicializa la clase y crea (o sobrescribe) un archivo con el nombre especificado.
        :param file_name: Nombre del archivo de texto donde se almacenarán las coordenadas.
        """
        self.file_name = file_name

        # Crear o sobrescribir el archivo con un encabezado
        with open(self.file_name, 'w') as file:
            file.write("")

        self.coordinates = []

        print(f'Archivo "{self.file_name}" inicializado.')

    def add_coordinates(self, x, y, yaw):
        """
        Agrega un par de coordenadas al archivo.
        :param x: Coordenada x (float).
        :param y: Coordenada y (float).
        """
        try:
            # Validar que los valores sean flotantes
            x = float(x)
            y = float(y)
            yaw = float(yaw)

            # Escribir las coordenadas en el archivo
            with open(self.file_name, 'a') as file:
                file.write(f"{x} {y} {yaw}\n")

            self.coordinates.append((x, y))

            # print(f"Coordenadas ({x}, {y}) añadidas correctamente al archivo.")
        except ValueError as e:
            print(f"Error al añadir las coordenadas: {e}")

