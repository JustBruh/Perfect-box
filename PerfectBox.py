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
        self.speedOfWheels = {"lSpeed":, "rSpeed":}

    def countRoute(self, pathToMap):
        return self.targetDirection, self.targetDistance


    def _setup_GPIO(self, IN1, IN2, IN3, IN4, ENA, ENB, SCL, SDA, ENCODERL, ENCODERR):
        
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)  
    
        GPIO.setup(IN1, OUT)
        GPIO.setup(IN2, OUT)
        GPIO.setup(IN3, OUT)
        GPIO.setup(IN4, OUT)
        GPIO.setup(ENA, OUT)
        GPIO.setup(ENB, OUT)
        GPIO.setup(ENCODERL, IN)
        GPIO.setup(ENCODERR, IN)

    def getCurrentDirection(self, hmc5883l):
        aX = hmc5883l.getHeading()
            return aX[0]

    def getCurrentSpeed(self, PCF8591):
        self.speedOfWheels=PCF8591.analogRead()
        if self.speedOfWheels[1]==self.speedOfWheels[2]
            self.isWheelsSyncing = True
        else:
            self.isWheelsSyncing = False
        return self.speedOfWheels
			

    def isRouteMatching(self, currentDirection, syncOfWheels, currentDistance):

        if self.targetDistance == currentDistance:
            self.isDistanceMatching = True
            return self.isDistanceMatching
        else:
            self.isDistanceMatching = False

        speedDiff = abs(speedOfWheels("lSpeed") - speedOfWheels("rSpeed") 
        if speedDiff == 0:
            self.isWheelsSyncing = True
        else:
            self.isWheelsSyncing = False

        directionDiff = abs(currentDirection-self.targetDirection) 
        if directionDiff <= 5:
            self.isDirectionMatching = True
        else:
            self.isDirectionMatching = False
        return (self.isDirectionMatching, self.isWheelsSyncing, self.isDistanceMatching)

    def startMotion(self, DUTY_CYCLE_L, DUTY_CYCLE_R, l_Motor_Pwm, r_Motor_Pwm):
        l_Motor_Pwm.start(DUTY_CYCLE_L)
        r_Motor_Pwm.start(DUTY_CYCLE_R)

    def unloading(self):
        GPIO.output(hydrocylinder, HIGH)
        time.sleep(10)
        GPIO.output(hydrocylinder, LOW)

    def uploading(self):
        time.sleep(20)
    
    def correctDirection(self):
        temp = 0
        while directionDiff != 0:
            directionDiff = int(self.getCurrentDirection())-self.targetDirection
            temp+=1
            if diff > 0:
                self.l_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_L+temp)
            if diff < 0:
                self.r_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_R+temp)

    def synchWheels(self, l_Motor_Pwm, r_Motor_Pwm):
        if self.isWheelsSyncing == False:
            while self.isWheelsSyncing != True:
                currentSpeed = self.getCurrentSpeed()
                if currentSpeed[1]-currentSpeed[2]> 0:
                    r_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_R+temp)
                if currentSpeed[1]-currentSpeed[2]< 0:
                    l_Motor_Pwm.ChangeDutyCycle(DUTY_CYCLE_L+temp)
            return True
        else:
            return "Wheels already synchronized"

hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
hmc5883l.setContinuousMode()

pcf8591 = PCF8591(0x48)

l_Motor_Pwm = GPIO.PWM(ENA, FREQUENCY)
r_Motor_Pwm = GPIO.PWM(ENB, FREQUENCY)