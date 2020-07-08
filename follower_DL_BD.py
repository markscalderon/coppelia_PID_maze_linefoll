import sim
import sys
import time
import cv2
import numpy as np
import math

class Bot:
    def __init__(self, ip="127.0.0.1", port=19999):
        self.sen_Izq = "LeftSensor"
        self.sen_Der = "RightSensor"
        self.sen_Mid = "MiddleSensor"
        self.camaraSuperior = "Vision_sensor"
        self.ultrasonido = "Proximity_sensor"
        self.mot_Izq = "DynamicLeftJoint"
        self.mot_Der = "DynamicRightJoint"
        self.ip = ip
        self.port = port
        self.velI = 0.03
        self.velD = 0.03
        self.delta = 0.01
        self.radio_rueda = 0.027
        self.dmax = 0.35 #distancia maxima al sensor
        self.dmin = 0.08 #distancia minima del objeto al sensor
        #procesamiento imagenes
        self.estado = -1
        self.nombreimg = "img"


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

        print("delta {}".format(self.delta))
        imgL = self.sensor_ir(self.camLeft)
        imgM = self.sensor_ir(self.camMid)
        imgR = self.sensor_ir(self.camRight)
        img = self.get_image(self.camSup)
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

        list_estados = []
        cont = 0
        while(1):
            img = self.get_image(self.camSup)
            cv2.imshow('Image', img)

            tecla = cv2.waitKey(5) & 0xFF
            if tecla == 27:
                break
            self.vel_follow()
            list_estados.append(self.estado)
            cv2.imwrite("BD/"+self.nombreimg+str(cont)+".jpg", img)
            cont = cont + 1
            #d = self.get_distance(self.ultra,self.dmax)
            #print("distancia sensado: {}".format(d))


        #cerrar
        x = np.array(list_estados)
        np.savetxt('salidas2.csv',x, delimiter='\n', fmt='%d')
        sim.simxFinish(self.clientID)

    def sensor_ir(self, idcam):
        #Verdadero fuera de pista
        if(self.activo == False):
            return

        _, resolution, image=sim.simxGetVisionSensorImage(self.clientID, idcam, 1, sim.simx_opmode_streaming)
        if(len(resolution) == 0):
            print("No pudo conectarse a la camara {}".format(idcam))
            return None
        img = np.array(image, dtype = np.uint8)
        return img[0] > 40

    def vel_follow(self):
        v = 0.01
        enew = 0
        eold = 0
        kp , kd , ki = 0.0085 , 0.001 ,  0
        imgM = self.sensor_ir(self.camMid)
        imgL = self.sensor_ir(self.camLeft)
        imgR = self.sensor_ir(self.camRight)

        #estados"
        #    0: izquierda
        #    1: centro
        #    2: derecha
        #    3: parar
        if((imgM == False and imgL == False) and imgR == False):#estado 2(velocidad normal)
            vL ,vD = v,v
            self.estado = 1
        elif(( imgL == True and imgR == False and imgM == True) or (imgL == True and imgR == False and imgM == False)): #sensor izquierda activo
            enew = -1
            vL = v - (kp*enew + kd*(enew - eold) + ki*(enew +  eold))
            vD = 0
            eold = enew

            self.estado = 0
        elif((imgM == False and imgL == False and imgR == True) or (imgM == True and imgL == False and imgR == True)): #sensor derecha activo
            enew = 1
            vD = v + (kp*enew + kd*(enew - eold) + ki*(enew +  eold))
            vL = 0
            eold = enew
            #print("estado 3")
            self.estado = 2
        else:
            vL, vD = 0,0

            self.estado = 3

        wL = vL/self.radio_rueda
        wR = vD/self.radio_rueda
        #print("wL: {}, wR: {}, imgM: {} , imgL: {} , imgR: {}".format(wL,wR, imgM, imgL, imgR))
        sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, wR, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, wL, sim.simx_opmode_oneshot)

    #ajusta la velocidad acorde a la deteccion del sensor
    def get_velocity(self,d, vmax):
        if(d < self.dmin):
            return 0
        else:
            return vmax*d/self.dmax

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
