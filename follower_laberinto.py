import sim
import sys
import time
import cv2
import numpy as np
import math
from time import perf_counter


class Bot:
    def __init__(self, ip="127.0.0.1", port=19999):
        self.sen_Izq = "LeftSensor"
        self.sen_Der = "RightSensor"
        self.sen_Mid = "MiddleSensor"
        self.camaraSuperior = "Vision_sensor"
        self.ultrasonido = "Proximity_sensor"
        self.mot_Izq = "DynamicLeftJoint"
        self.mot_Der = "DynamicRightJoint"
        self.laser_Izq = "Laser_I"
        self.laser_Der = "Laser_D"
        self.ip = ip
        self.port = port
        self.velI = 0.03
        self.velD = 0.03
        self.delta = 0.1

        #propiedades relacionadas al seguidor
        self.estado = 1
        self.lastime = 0
        self.radio_rueda = 0.027
        self.dmax = 0.5 #distancia maxima al sensor
        self.dmin = 0.2 #distancia minima del objeto al sensor
        self.maxlaser = 0.5#distancia maxima del sensor
        self.distwallright = 0.13
        self.distwallfront = 0.20
        self.tiempogiro90 = 0
        self.tiempocurva = 0
        self.v = 0.025
        self.b = 0.058#distancia centro a la llanta
        self.len_celda = 0.5 #longitud de celda
        self.dwall = 0.14
        self.dtol = 0.05

        self.connect()
        self.init_elements()

    def connect(self):
        print ('Programa inicio')
        sim.simxFinish(-1) # cerrar todas las conexiones
        # Conectar a CoppeliaSim
        self.clientID=sim.simxStart(self.ip,self.port,True,True,5000,5)
        self.activo = True
        if(self.clientID == -1):
            print("Imposible conectar")
            self.activo = False

    def init_elements(self):
        if(self.activo == False):
            return
        #Guardar la referencia de la camara
        _, self.camLeft = sim.simxGetObjectHandle(self.clientID, self.sen_Izq, sim.simx_opmode_oneshot_wait)
        _, self.camMid = sim.simxGetObjectHandle(self.clientID,self.sen_Mid, sim.simx_opmode_oneshot_wait)
        _, self.camRight = sim.simxGetObjectHandle(self.clientID, self.sen_Der, sim.simx_opmode_oneshot_wait)
        _, self.motorRight = sim.simxGetObjectHandle(self.clientID, self.mot_Der, sim.simx_opmode_oneshot_wait)
        _, self.motorLeft = sim.simxGetObjectHandle(self.clientID, self.mot_Izq, sim.simx_opmode_oneshot_wait)
        _, self.camSup = sim.simxGetObjectHandle(self.clientID, self.camaraSuperior, sim.simx_opmode_oneshot_wait)
        _, self.ultra = sim.simxGetObjectHandle(self.clientID, self.ultrasonido, sim.simx_opmode_oneshot_wait)

        _, self.laserI = sim.simxGetObjectHandle(self.clientID,self.laser_Izq, sim.simx_opmode_oneshot_wait)
        _, self.laserD = sim.simxGetObjectHandle(self.clientID,self.laser_Der, sim.simx_opmode_oneshot_wait)

        imgL = self.sensor_ir(self.camLeft)
        imgM = self.sensor_ir(self.camMid)
        imgR = self.sensor_ir(self.camRight)
        img = self.get_image(self.camSup)
        self.get_distance(self.laserI,self.maxlaser)
        self.get_distance(self.laserD,self.maxlaser)
        self.get_distance(self.ultra, self.dmax*2)
        time.sleep(1)

    def get_image(self, idcam):
        if(self.activo == False):
            return
        _, resolution, image=sim.simxGetVisionSensorImage(self.clientID, idcam, 0, sim.simx_opmode_streaming)
        if(len(resolution) == 0):
            print("No pudo conectarse a la camara {}".format(idcam))
            return None
        img = np.array(image, dtype = np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        img = np.fliplr(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def execute(self):
        if(self.activo == False):
            return

        while(1):
            img = self.get_image(self.camSup)

            cv2.imshow('Image', img)

            tecla = cv2.waitKey(5) & 0xFF
            if tecla == 27:
                break
            self.estados_transicion()
            self.aplicar_estado()
            #self.vel_follow()

        #cerrar
        sim.simxFinish(self.clientID)

    def tiempoSimulador(self):
        _ , tiempo = sim.simxGetFloatSignal(self.clientID,"tiemposimulacion",sim.simx_opmode_streaming)
        return tiempo

    def muroFrente(self):
        dultra = self.get_distance(self.ultra, self.dmax*2)
        if(dultra < self.distwallfront and dultra < self.dmax*2  ): # pared en frente
            return True
        return False

    def muroDerecha(self):
        dlD = self.get_distance(self.laserD, self.maxlaser*2)

        return abs(dlD - self.dwall) < self.dtol

    def meta(self):
        imgL = self.sensor_ir(self.camLeft)
        imgM = self.sensor_ir(self.camMid)
        imgR = self.sensor_ir(self.camRight)

        if( imgL < 40 and imgR <40 and imgM < 40):
            return True
        return False

    def aplicar_vel(self, idvel, vel):
        sim.simxSetJointTargetVelocity(self.clientID, idvel, vel, sim.simx_opmode_oneshot)

    def estados_transicion(self):
        '''
        estado del robot actual
        1: pared al costado, sigue avanzando
        2: pared al frente, debe girar
        3: pared al costo no detectada, curva 90
        4: parar
        '''
        if(self.meta()):
            self.estado == 4 #llego a la meta
        if(self.estado == 1 ):
            self.lastime =  self.tiempoSimulador()
            if( self.muroFrente()):
                self.estado = 2
            if( not self.muroDerecha()):
                self.estado = 3
        elif(self.estado == 2):
            tiempoCorriendo = (self.tiempoSimulador() - self.lastime) > self.tiempogiro90
            if( self.muroDerecha() and tiempoCorriendo):
                self.estado = 1
        elif(self.estado == 3):
            tiempoCorriendo = (self.tiempoSimulador() - self.lastime ) > self.tiempocurva
            if(self.muroDerecha() and tiempoCorriendo):
                self.estado = 1
        print("estado: {}, tiempo simulador: {}".format(self.estado, self.tiempoSimulador()))

    def aplicar_estado(self):

        if(self.estado == 1):
            wL,wR = self.seguirMuro()
        elif(self.estado == 2): #girar lento
            wL,wR = self.girar90()
        elif(self.estado == 3): # girar curva
            wL, wR = self.girar_curva()
        else:
            wL = 0
            wR = 0
        #aplicar velocidad a los motores
        self.aplicar_vel(self.motorRight, wR)
        self.aplicar_vel(self.motorLeft,wL)

    def seguirMuro(self):
        print("seguir muro")
        vl = self.v
        vr = self.v
        wL = vl/self.radio_rueda
        wR = vr/self.radio_rueda
        return (wL, wR)

    def girar90(self):
        print("girar 90")
        vel = self.v / 4
        self.tiempogiro90 = (self.b*(math.pi /2)/vel)
        print("Tiempo de giro: {}".format(self.tiempogiro90))
        wL = -vel/self.radio_rueda
        wR = vel/self.radio_rueda
        return (wL, wR)

    def girar_curva(self):
        print("girar curva")
        vcurva = self.v/2
        w = (vcurva)/(self.len_celda/2)
        wL = (vcurva + self.b*w)/self.radio_rueda
        wR = (vcurva - self.b*w)/self.radio_rueda
        self.tiempocurva = ((math.pi/2)*(self.len_celda/2)) /vcurva
        return (wL, wR)

    def sensor_ir(self, idcam):
        if(self.activo == False):
            return

        _, resolution, image=sim.simxGetVisionSensorImage(self.clientID, idcam, 1, sim.simx_opmode_streaming)
        if(len(resolution) == 0):
            print("No pudo conectarse a la camara {}".format(idcam))
            return None
        img = np.array(image, dtype = np.uint8)
        return img[0]

    def get_distance(self,sensor,max_dist):
        _, sta, point, objh, vec = sim.simxReadProximitySensor(self.clientID, sensor, sim.simx_opmode_streaming)

        if(sta ==  False): #no se detecto nada
            distance = max_dist
        else:
            distance = math.sqrt( point[0]**2 + point[1]**2 + point[2]**2)
        return distance


def main():
    ex = Bot()
    ex.execute()


if __name__ == '__main__':
    main()
