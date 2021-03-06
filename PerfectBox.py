import time
import RPi.GPIO as GPIO
from i2clibraries import i2c_hmc5883l
from wiringpi import pcf8591

##IN1 = 19   
##IN2 = 13
##IN3 = 6
##IN4 = 5
##ENA = 26
##ENB = 22
##SCL = 3
##SDA = 2
##
##GPIO.cleanup()
##GPIO.setmode(GPIO.BCM)
##GPIO.setup(IN1)
##GPIO.setup(IN2)
##GPIO.setup(IN3)
##GPIO.setup(IN4)
##GPIO.setup(ENA)
##GPIO.setup(ENB)


class controller():

    def __init__(self, name):

        self.name = name
        self.sourceCoordinates
        self.isWheelsSyncing = False
        self.isDirectionMatching = False
        self.isDistanceMatching = False
        self.targetDirection
        self.targetSpeed
        self.targetDistance
        self.distance = 0

    def countRoute(self, pathToMap): # определение целевого направления и дистанции, используя карту местности
        return self.targetDirection, self.targetDistance


    def _setup_GPIO(self, IN1, IN2, IN3, IN4, ENA, ENB): # настройка параметров GPIO
        
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)  
    
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(ENB, GPIO.OUT)

    def getCurrentDirection(self, hmc5883l): # получить текущее направление и проверить, совпадает ли оно с заданным
        currentDirection = hmc5883l.getHeading()
        directionDiff = abs(int(currentDirection[0])-self.targetDirection) 
        if directionDiff == 0:
            self.isDirectionMatching = True
        else:
            self.isDirectionMatching = False
        return int(currentDirection[0])

    def getCurrentSpeed(self, PCF8591): # получить текущую скорость и проверить синхронизацию колес
        speedOfWheels = PCF8591.analogRead()
        if speedOfWheels[0] == speedOfWheels[1]
            self.isWheelsSyncing = True
        else:
            self.isWheelsSyncing = False
        return speedOfWheels

    def getDistance(self): # поулчить текущее расстояние и проверить, совпадает ли оно с целевым
        if self.distance == targetDistance:
            self.isDistanceMatching = True
        return self.distance
		
    def countDistance(self, lastCheckpointSpeed, iterationTime): # подсчет расстояния, пройденного от последнего обновления пройденного пути
        currentSpeed = self.getCurrentSpeed()
        acceleration = (abs(int(currentSpeed[0]) - lastCheckpointSpeed)) / iterationTime
        pathFromLastCheckpoint = (lastCheckpointSpeed*iterationTime) + (acceleration*iterationTime**2) / 2
        self.distance += pathFromLastCheckpoint
    
    def startMotion(self, DUTY_CYCLE_L, DUTY_CYCLE_R, l_Motor_Pwm, r_Motor_Pwm): # начало движения
        l_Motor_Pwm.start(DUTY_CYCLE_L)
        r_Motor_Pwm.start(DUTY_CYCLE_R)

    def unloading(self): # разгрузка
        GPIO.output(hydrocylinder, HIGH)
        time.sleep(10)
        GPIO.output(hydrocylinder, LOW)

    def uploading(self): # погрузка
        time.sleep(20)
    
    def correctDirection(self): # корректировка направления
        temp = 0
        while self.isDirectionMatching != True:
            directionDiff = self.getCurrentDirection()-self.targetDirection
            temp+=1
            if diff > 0:
                self.l_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_L+temp)
            if diff < 0:
                self.r_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_R+temp)
            self.getCurrentDirection()
 
    def synchWheels(self, l_Motor_Pwm, r_Motor_Pwm): # синхронизация скорости вращения ведущих колес
        while self.isWheelsSyncing != True:
            currentSpeed = self.getCurrentSpeed()
            if currentSpeed[0]-currentSpeed[1]> 0:
                r_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_R+temp)
            if currentSpeed[0]-currentSpeed[1]< 0:
                l_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_L+temp)
            self.getCurrentSpeed()

IN1 = 19   
IN2 = 13
IN3 = 6
IN4 = 5
ENA = 26
ENB = 22
SCL = 3
SDA = 2

FREQUENCY = 600 

if __name__ == "main":

    controller = controller("penny")
    controller.uploading()
    controller._setup_GPIO(IN1, IN2, IN3, IN4, ENA, ENB)

    l_Motor_Pwm = GPIO.PWM(ENA, FREQUENCY)
    r_Motor_Pwm = GPIO.PWM(ENB, FREQUENCY)

    hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
    hmc5883l.setContinuousMode()

    pcf8591 = PCF8591(0x48)

    target = controller.countRoute("//home//kali//Documents//Map")
    controller.targetDirection = target["targetDirection"]
    controller.targetDistance = target["targetDistance"]

    controller.startMotion(100, 100, l_Motor_Pwm, r_Motor_Pwm)
    
    lastCheckpointSpeed = 0

    while controller.isDirectionMatching != True && controller.isDistanceMatching != True:

        currentDirection = controller.getCurrentDirection()
        currentDistance = controller.getDistance()
        currentSpeed = controller.getCurrentSpeed()

        if controller.isDirectionMatching() == False:
            controller.correctDirection()
            controller.synchWheels()

        if controller.isWheelsSyncing == False:
            controller.syncWheels()

        controller.countDistance(lastCheckpointSpeed, 0.2)

        lastCheckpointSpeed = currentSpeed

    controller.unloading()