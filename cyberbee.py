import tellopy

class CyberBee:
    
    def __init__(self):
        self.drone = tellopy.Tello()
        
    def GetDrone(self):
        return self.drone
    
    