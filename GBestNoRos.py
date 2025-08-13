from gadentools.Utils import Vector3

class GBestNoRos:
    def __init__(self, position: Vector3, concentration: float):
        self.position = position
        self.concentration = concentration

    def update_gbest(self, new_position: Vector3, new_concentration: float):
        self.position = new_position
        self.concentration = new_concentration

    def get_position(self):
        return self.position

    def get_concentration(self):
        return self.concentration