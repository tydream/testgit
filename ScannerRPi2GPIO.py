#!/usr/bin/env python2

# scannerRPi2GPIO.py

import time
import thread
import RPi.GPIO as GPIO

# contro printing messages
PRINT = 1  

class ScannerRPi2GPIO(object) :
  # class variables for GPIO pin assignment -- BCM mode
  ledred = 17    # pin 11
  ledyellow = 27 # pin 13
  ledgreen = 22  # pin 15
  pir = 24       # pin 18
  switch = 12    # pin 32
  buzzer = 18    # pin 12 PCM_CLK   

  # class variables for PIR detection control
  pirblockingduration = 5000   # in msec to set diable window length
  pirmotiondetected = False
  pirfirstsensed = False
  pircurrclock = 0.00
  pirstartclock = 0.00
  lock = thread.allocate_lock()

  # constructor
  def __init__(self) :
    GPIO.setmode(GPIO.BCM)   # pin assignment is based on e.g. GPIO18 not physical pin numer
    GPIO.setup(self.__class__.ledred, GPIO.OUT)
    GPIO.setup(self.__class__.ledyellow, GPIO.OUT)
    GPIO.setup(self.__class__.ledgreen, GPIO.OUT) 
    GPIO.setup(self.__class__.pir, GPIO.IN)  # no pull-down, pull-up resistor
    GPIO.setup(self.__class__.switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # full-down resistor: 1M Ohm
                                                                # full-up resistor: 470 Ohm
    GPIO.setup(self.__class__.buzzer, GPIO.OUT) 
    lock = thread.allocate_lock()
    
  def ResetGPIO(self) :
    GPIO.remove_event_detect(self.pir)
    GPIO.cleanup()

  def LED_RedTurnOn(self) :
    GPIO.output(self.__class__.ledred, True) 

  def LED_RedTurnOff(self) :
    GPIO.output(self.__class__.ledred, False) 

  def LED_YellowTurnOn(self) :
    GPIO.output(self.__class__.ledyellow, True) 

  def LED_YellowTurnOff(self) :
    GPIO.output(self.__class__.ledyellow, False) 

  def LED_GreenTurnOn(self) :
    GPIO.output(self.__class__.ledgreen, True) 

  def LED_GreenTurnOff(self) :
    GPIO.output(self.__class__.ledgreen, False) 

  def Buzzer(self, freq, duty, duration) :  # generate buzz for duration secs
    if freq == 0 :
      time.sleep(duration)
      return
    p = GPIO.PWM(self.__class__.buzzer, freq)
    p.start(duty)
    time.sleep(duration)
    p.stop()

  def BuzzerOld(self, freq, duration) :  # generate buzz for duration secs
    if freq == 0 :
      time.sleep(duration)
      return
    period = 1.0 / freq
    delay = period / 2
    iteration = int(freq * duration)
    for i in range(iteration) :
      GPIO.output(self.__class__.buzzer, True)
      time.sleep(delay)
      GPIO.output(self.__class__.buzzer, False)
      time.sleep(delay)

  def PIR_SetupInterrupt(self) :
    self.__class__.lock.acquire()
    self.__class__.pirmotiondetected = False
    self.__class__.lock.release()
    GPIO.add_event_detect(self.pir, GPIO.RISING, callback=self.PIR_ISR, \
                          bouncetime=self.__class__.pirblockingduration)
  
  def PIR_ISR(self, channel) :
    self.__class__.lock.acquire()
    self.__class__.pirmotiondetected = True
    self.__class__.lock.release()
    if PRINT == 1 :
      processclock = time.clock()
      print "ISR: Motion detected at process time " + "%.2f" %processclock


  def PIR_IsHumanMotionDetected(self) :
    self.__class__.lock.acquire()
    if self.__class__.pirmotiondetected == True :
      self.__class__.pirmotiondetected = False
      self.__class__.lock.release() 
      return True
    else :
      self.__class__.pirmotiondetected = False
      self.__class__.lock.release() 
      return False


