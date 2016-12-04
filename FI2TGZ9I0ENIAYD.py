#!/usr/bin/python
# coding: utf-8

#----------------------------------------------------------------
# Autor: Saymon C. A. Oliveira
# Email: saymowan@gmail.com
# Descrição: este algoritmo descreve a décima implementação de OpenCV
# Funções: Imagem digital -> Transformação HSV -> Imagem binária -> Erosão binária -> Encontrar área -> Encontrar coordenadas ->
# Funções 2: desenhar circulo no centroide (x,y) -> declaração de pinos -> declaração de funções de movimento do motor -> 
# Funções 3: declaração Função Z -> declaração Função X -> configurar servo motor -> declaração Função Y -> modularização do SVC*
# Funções 4: realizar movimento do robô  de profundidade (area) e vertical (y) -> outros ajustes
# Tecnologias: OpenCV, Python, GPIO e NumPy
#---------------------------------------------------------------

#SVC - Sistema de Visão Computacional

import cv2.cv as cv
import cv2 as cv2
import time
import numpy as np
import RPi.GPIO as gpio

# Tipo de GPIO
gpio.setmode(gpio.BOARD)
# Desliga alertas
gpio.setwarnings(False)

#---------------------------------------------------------------------
#  C O N F I G U R A C A O   D O    S E R V O 
#---------------------------------------------------------------------
def set(property, value):
	try:
		f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
		f.write(value)
		f.close()	
	except:
		print("Error writing to: " + property + " value: " + value)
		
def setServo(angle):
	set("servo", str(angle))
	
set("delayed", "0")
set("mode", "servo")
set("servo_max", "160")  #160
set("active", "1")
servo = 100
servoMin = 30
servoMax = 180
setServo(servo)



#---------------------------------------------------------------------------
#   F U N Ç Ã O   D E   A J U S T E   V E R T I C A L
#---------------------------------------------------------------------------
# Parametros:
# y -> posição atual no eixo Y
# may -> limite do centro
# centroy -> posição estática do centro

def ajusteY(y, may, centroy, servo):
	   # Se o limite for maior que a distancia do centro que é considerado centralizado
       if (y - centroy) >= may:
        servo = servo - 5
	setServo(servo)
		
	   # Se o limite for menor ...
       elif (centroy - y) >= may:
        servo = servo + 5
	setServo(servo)
		
	   #Se o  valor de deslocamento for inferior ao Minimo, faz o minimo
       if servo < servoMin:
        servo = servoMin
	setServo(servo)
		
	   # Se o valor de deslocamento for superior ao Maximo, faz o maximo
       if servo > servoMax:
        servo = servoMax
        setServo(servo)
		

      
#--------------------------------------------------------------------------
 #---------------------------------------
#Declara pinos como saida GPIO - Motor A
 
#pino de ativação do motor A via Rasp 1
gpio.setup(7, gpio.OUT)
 
#pino de ativação do motor A via Rasp 2 
gpio.setup(11, gpio.OUT)
 
# Iniciar Pino 13 como saida - Motor A
gpio.setup(13, gpio.OUT)
 
#Iniciar Pino 15 como saida - Motor A
gpio.setup(15, gpio.OUT)
 
#---------------------------------------
#Declara pinos como saida GPIO - Motor B
 
#pino de ativação do motor B via Rasp 1
gpio.setup(26, gpio.OUT)
 
#pino de ativação do motor B via Rasp 2 
gpio.setup(16, gpio.OUT)
 
# Iniciar Pino 5 como saida - Motor B
gpio.setup(18, gpio.OUT)
 
#Iniciar Pino 22 como saida - Motor B
gpio.setup(22, gpio.OUT)
 


#-----------------------------------------
# Permitir que o L298N seja controlado pelo GPIO:
#---------------------------------------
#Valores iniciais - True - Motor A ativado
gpio.output(7, True) #Motor A - Rasp 1
gpio.output(11, True) #Motor A - Rasp 2
#---------------------------------------
#Valores iniciais - True - Motor B ativado
gpio.output(26, True) #Motor B - Rasp 1
gpio.output(16, True) #Motor B - Rasp 2
#---------------------------------------

def Frente():
# Motor 1
 gpio.output(13, True)
 gpio.output(15, False)
# Motor 2
 gpio.output(18, False)
 gpio.output(22, True)
	
def Tras():
# Motor 1
 gpio.output(13, False)
 gpio.output(15, True)
# Motor 2
 gpio.output(18, True)
 gpio.output(22, False)
 
 
def Parar():
# Motor 1
 gpio.output(18, False)
 gpio.output(22, False)
# Motor 2
 gpio.output(13, False)
 gpio.output(15, False)


def Direita():
# Motor 1
 gpio.output(13, True)
 gpio.output(15, False)
# Motor 2
 gpio.output(18, True)
 gpio.output(22, False)


def Esquerda():
# Motor 1
 gpio.output(13, False)
 gpio.output(15, True)
# Motor 2
 gpio.output(18, False)
 gpio.output(22, True)


# Faixa de HSV que usamos para detectar o objeto colorido
# Neste exemplo, pré definidos para uma bola verde
Hmin = 42
Hmax = 92
Smin = 62
Smax = 255
Vmin = 63
Vmax = 235


#Padrão RED
#Hmin = 0
#Hmax = 179 
#Smin = 131
#Smax = 255
#Vmin = 126
#Vmax = 255


# Cria-se um array de valores HSV(mínimo e máximo)
rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)

# Função de processamento  da imagem
def processamento(entrada):
     imgMedian = cv2.medianBlur(entrada,1)
     imgHSV = cv2.cvtColor(imgMedian,cv2.cv.CV_BGR2HSV)	
     imgThresh = cv2.inRange(imgHSV, rangeMin, rangeMax)
     imgErode = cv2.erode(imgThresh, None, iterations = 3)
     return imgErode
#-------------------------------------------------------------------------

cv.NamedWindow("Entrada")
#cv.NamedWindow("HSV")
#cv.NamedWindow("Thre")
cv.NamedWindow("Erosao")
capture = cv2.VideoCapture(0)




#------------------------------------------------------------------------
#  P A D R O E S     D E      A J U S T E
#------------------------------------------------------------------------  
# Parametros do tamannho da imagem de captura
largura = 160
altura = 120

# Area minima a ser detectada
minArea = 50 #cerca de 80 cm

#Centro dos eixos
centroy = altura/2


# Limite do centro
may = altura/5 #24
#----------------------------------------------------------------------



# Definir um tamanho para os frames (descartando o PyramidDown)
if capture.isOpened():
  capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, largura)
  capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, altura)

  
while True:
    ret, entrada = capture.read()
    imagem_processada = processamento(entrada)
    moments = cv2.moments(imagem_processada, True)
    area = moments['m00']
    setServo(servo)
    if moments['m00'] >= minArea:
     x = moments['m10'] / moments['m00']
     y = moments['m01'] / moments['m00']
     cv2.circle(entrada, (int(x), int(y)), 5, (0, 255, 0), -1)
     if(area<=120):
      Frente()
     elif(area>=600):
      Tras()
     else:
      Parar()
# Se o limite for maior que a distancia do centro que é considerado centralizado
      if (y - centroy) >= may:
         servo = servo - 3
	 setServo(servo)
		
	   # Se o limite for menor ...
      elif (centroy - y) >= may:
         servo = servo + 3
	 setServo(servo)
		
	   #Se o  valor de deslocamento for inferior ao Minimo, faz o minimo
      if servo < servoMin:
         servo = servoMin
	 setServo(servo)
		
	   # Se o valor de deslocamento for superior ao Maximo, faz o maximo
      if servo > servoMax:
         servo = servoMax
         setServo(servo)	 
    else:
     Parar()
     setServo(servo)
    
    cv2.imshow("Entrada",entrada)
    #cv2.imshow("HSV", imgHSV)
    #cv2.imshow("Thre", imgThresh)
    cv2.imshow("Erosao", imagem_processada)

    if cv.WaitKey(10) == 27:
        break
	cv.DestroyAllWindows()	
	gpio.cleanup()	

	