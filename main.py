import machine
from machine import Pin, ADC
import utime

a0 = ADC(0)

def mean(a, b):
    return (a + b) / 2


class Multiplexer:
    def __init__(self, a: int, b: int, c: int):
        self.a = Pin(a, Pin.OUT)
        self.b = Pin(b, Pin.OUT)
        self.c = Pin(c, Pin.OUT)
        self.inOut = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [1, 1, 0],
            [0, 0, 1],
            [1, 0, 1],
            [0, 1, 1],
            [1, 1, 1]
        ]

    def setInOut(self, inOutPin):
        self.a.value(self.inOut[inOutPin][0])
        self.b.value(self.inOut[inOutPin][1])
        self.c.value(self.inOut[inOutPin][2])
        print(self.a.value(), self.b.value(), self.c.value())


class LDRCluster:
    def __init__(self, multiplexer, llMultiPin, lrMultiPin, ulMultiPin, urMultiPin):
        self.multiplexer = multiplexer
        self.llMultiPin = llMultiPin
        self.lrMultiPin = lrMultiPin
        self.ulMultiPin = ulMultiPin
        self.urMultiPin = urMultiPin
        self.llCalibration = [0, 0]
        self.lrCalibration = [0, 0]
        self.ulCalibration = [0, 0]
        self.urCalibration = [0, 0]
    
    def readLDRs(self):
        self.multiplexer.setInOut(2)
        utime.sleep(0.002)
        ll = a0.read_u16()
        print("ll--", ll)
        utime.sleep(0.002)
        self.multiplexer.setInOut(1)
        utime.sleep(0.002)
        lr = a0.read_u16()
        print("lr--", lr)
        utime.sleep(0.002)
        self.multiplexer.setInOut(0)
        utime.sleep(0.002)
        ul = a0.read_u16()
        print("ul--", ul)
        utime.sleep(0.002)
        self.multiplexer.setInOut(3)
        utime.sleep(0.002)
        ur = a0.read_u16()
        print("ur--", ur)
        utime.sleep(0.002)
        return ll, lr, ul, ur
    
    #TODO: Implement calibration




class Stepper:
    def __init__(self, in1, in2, in3, in4):
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.in3 = Pin(in3, Pin.OUT)
        self.in4 = Pin(in4, Pin.OUT)
        self.stepSequence = [
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
            [1, 0, 0, 1]
        ]
        self.stepCount = 0
    
    def step(self, direction, delay, amount=8):
        if direction == 1:
            for i in range(amount):
                self.stepCount += 1
                if self.stepCount == 8:
                    self.stepCount = 0
                self.in1.value(self.stepSequence[self.stepCount][0])
                self.in2.value(self.stepSequence[self.stepCount][1])
                self.in3.value(self.stepSequence[self.stepCount][2])
                self.in4.value(self.stepSequence[self.stepCount][3])
                utime.sleep(delay)
        elif direction == 0:
            for i in range(amount):
                self.stepCount -= 1
                if self.stepCount == -1:
                    self.stepCount = 7
                self.in1.value(self.stepSequence[self.stepCount][0])
                self.in2.value(self.stepSequence[self.stepCount][1])
                self.in3.value(self.stepSequence[self.stepCount][2])
                self.in4.value(self.stepSequence[self.stepCount][3])
                utime.sleep(delay)
    
class Servo:
    def __init__(self, pin):
        self.pin = Pin(pin, Pin.OUT)
        self.pwm = machine.PWM(self.pin)
        self.pwm.freq(50)
        self.pwm.duty(77)
    
    def moveUp(self, diff):
        if self.pwm.duty() < 110:
            self.pwm.duty(self.pwm.duty() + diff)
            utime.sleep(0.02)
            print(self.pwm.duty())

    def moveDown(self, diff):
        if self.pwm.duty() > 65:
            self.pwm.duty(self.pwm.duty() - diff)
            utime.sleep(0.02)
            print(self.pwm.duty())


class Solartracker:
    def __init__(self, ldrCluster, stepper, servo, sensitivityZ, sensitivityX):
        self.ldrCluster = ldrCluster
        self.stepper = stepper
        self.servo = servo
        self.sensitivityZ = sensitivityZ
        self.sensitivityX = sensitivityX
    
    def track(self):
        ll, lr, ul, ur = self.ldrCluster.readLDRs()
        if (mean(ll, lr) / mean(ul, ur)) > self.sensitivityX: #move down
            self.servo.moveDown(1)
        elif (mean(ul, ur) / mean(ll, lr)) >  self.sensitivityX: #move up
            self.servo.moveUp(1)
        if (mean(ll, ul) / mean(lr, ur)) > self.sensitivityZ: #move left
            self.stepper.step(0, 0.002, 64)
        elif (mean(lr, ur) / mean(ll, ul)) > self.sensitivityZ: #move right
            self.stepper.step(1, 0.002, 64)




multiplexer = Multiplexer(13, 15, 2)
ldrCluster = LDRCluster(multiplexer, 2, 1, 0, 3)
stepper = Stepper(4, 0, 14, 12)
servo = Servo(5)
solartracker = Solartracker(ldrCluster, stepper, servo, 1.08, 1.12)

while True:
    solartracker.track()
    utime.sleep(0.005)