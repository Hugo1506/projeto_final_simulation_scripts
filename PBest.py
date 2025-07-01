from gadentools.Utils import Vector3

class PBest:
    def __init__(self, position: Vector3, concentration: float):
        self.position = position  
        self.concentration = concentration 

    def __repr__(self):
        return f"PBest(position={self.position}, concentration={self.concentration})"

    def update_pbest(self, new_position: Vector3, new_concentration: float):
        if self.concentration < new_concentration:
            self.position = new_position
            self.concentration = new_concentration
        

    def get_position(self):

        return self.position

    def get_concentration(self):
        return self.concentration
