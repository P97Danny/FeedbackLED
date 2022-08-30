#!/usr/bin/sudo /usr/bin/python
"""
19-1-2019
Geschreven door Danny Peters uit V1k groep 3
Dit is het basisprogramma voor de geluidslamp
Eerst wordt het geluid gemeten door een slave arduino
Daarna wordt aan de hand van dat gemeten signaal het niveau bepaalt
Vervolgens worden er LEDs aan de hand van het niveau aangestuurd
"""

import RPi.GPIO as GPIO
import serial
import os
import time
import statistics
import socket
import threading
import math

# Hardcode your client IP and port here:
#host = '192.168.42.2'
#port = 12345

# Commands om waardes uit te lezen gegeven van de slave arduino uno
ser = serial.Serial('/dev/ttyACM0')
ser.baudrate = 115200
ser.flushInput()

# Wat globale variabelen die in meerdere functies gebruikt worden
minWaardeTop = 0
minWaardeBottom = 0
niveau = ""
LEDopslag = []
copyBytes = 0
gemiddelde = 0
kleur = ""
lcdKleur = ""
dB = 0
s = 0
ADC_bytes = 0

# Koppel LED variablele aan een pin nummer
G1LED = 21
G2LED = 20
Y1LED = 16
Y2LED = 26
R1LED = 19
R2LED = 13

# LCD variabelen koppelen aan een pin nummer
LCD_RS = 25
LCD_E =  24
LCD_D4 = 23
LCD_D5 = 12
LCD_D6 = 5
LCD_D7 = 6

# Extra LCD variabelen
LCD_WIDTH = 16
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM adres voor 1e lijn
LCD_LINE_2 = 0xC0 # LCD RAM adres voor 2e lijn

E_PULSE = 0.0005
E_DELAY = 0.0005

def LEDsetup():
    # Elke LED en LCD pin wordt als een OUTPUT geschreven en bij start van het programma uit gezet
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(G1LED, GPIO.OUT)
    GPIO.setup(G2LED, GPIO.OUT)
    GPIO.setup(Y1LED, GPIO.OUT)
    GPIO.setup(Y2LED, GPIO.OUT)
    GPIO.setup(R1LED, GPIO.OUT)
    GPIO.setup(R2LED, GPIO.OUT)

    GPIO.setup(LCD_E, GPIO.OUT)
    GPIO.setup(LCD_RS, GPIO.OUT)
    GPIO.setup(LCD_D4, GPIO.OUT)
    GPIO.setup(LCD_D5, GPIO.OUT)
    GPIO.setup(LCD_D6, GPIO.OUT)
    GPIO.setup(LCD_D7, GPIO.OUT)
    
    lcd_init()
    LEDuit()

def connect():
    global s

    # Setup connection to the client
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.connect((host, port))
    

def LEDuit():
    # Functie die de LEDs uitzet om overbodige herhaling in code te voorkomen
    # Veder geen primaire functie
    GPIO.output(G1LED, GPIO.LOW)
    GPIO.output(G2LED, GPIO.LOW)
    GPIO.output(Y1LED, GPIO.LOW)
    GPIO.output(Y2LED, GPIO.LOW)
    GPIO.output(R1LED, GPIO.LOW)
    GPIO.output(R2LED, GPIO.LOW)

def bepaalMinWaardes():
    # Functie die de minimale en maximale waardes meet die gebruikt worden
    # om geluidsniveau te bepalen
    global minWaardeTop
    global minWaardeBottom
    global ADC_bytes
    global gemiddelde
    
    byteOpslag = []
    gemmidelde = 0
    print("Microfoon kalibreren...")
    
    for i in range(20): # Losse meting om de ruis uit de arduino te filteren
        ser_bytes = ser.readline()
        #decoded_bytes = ord(ser_bytes)
        decoded_bytes = float(ser_bytes[0:int(len(ser_bytes))-2].decode("utf-8"))
    for i in range(30):
        ser_bytes = ser.readline()
        #decoded_bytes = ord(ser_bytes)
        decoded_bytes = float(ser_bytes[0:int(len(ser_bytes))-2].decode("utf-8"))
        waarde = decoded_bytes
        byteOpslag.append(waarde)
        ADC_bytes = decoded_bytes
        if(decoded_bytes > gemiddelde):
            copyBytes = float(decoded_bytes)
          
    gemiddelde = statistics.median(byteOpslag)
    minWaardeTop = gemiddelde +4
    minWaardeBottom = gemiddelde -4

    print("Klaar met kalibreren!")
    print("Minimum ADC waarde: ", minWaardeTop,"\nMaximum ADC waarde: ", minWaardeBottom)

def bepaalNiveau():
    # Functie die aan de hand van het gemeten geluid het niveau ervan te bepaalt
    global minWaardeTop
    global minWaardeBottom
    global niveau
    global LEDopslag
    global ADC_bytes
    global gemiddelde
    
    top = minWaardeTop
    bottom = minWaardeBottom
    ser_bytes = ser.readline()
    #print(ser_bytes)
    #decoded_bytes = ord(ser_bytes)
    decoded_bytes = float(ser_bytes[0:int(len(ser_bytes))-2].decode("utf-8"))
    if(decoded_bytes > gemiddelde):
        ADC_bytes = decoded_bytes

    if (decoded_bytes >= bottom and decoded_bytes <= top):
        niveau = "1" 
        LEDopslag.append(1)
    elif ((decoded_bytes >= (bottom - 7) and decoded_bytes < bottom) or (decoded_bytes <= (top +7) and decoded_bytes > top)):
        niveau = "2" 
        LEDopslag.append(2)
    elif ((decoded_bytes >= (bottom - 15) and decoded_bytes < (bottom -7)) or (decoded_bytes <= (top + 15) and decoded_bytes > (top + 7))):
        niveau = "3" 
        LEDopslag.append(3)
    elif ((decoded_bytes >= (bottom - 30) and decoded_bytes < (bottom -15)) or (decoded_bytes <= (top + 30) and decoded_bytes > (top + 15))):
        niveau = "4" 
        LEDopslag.append(4)
    elif ((decoded_bytes >= (bottom - 60) and decoded_bytes < (bottom -30)) or (decoded_bytes <= (top + 60) and decoded_bytes > (top + 30))):
        niveau = "5" 
        LEDopslag.append(5)
    elif (decoded_bytes > (top + 60) or decoded_bytes < (bottom - 60)):
        niveau = "6" 
        LEDopslag.append(6)

def liveLED():
    # Functie die LEDs aanstuurt aan de hand van het geluidsniveau
    global niveau
    LEDuit() # Begin met alle LEDs uit
    if niveau in "654321":
        GPIO.output(G1LED, GPIO.HIGH)
    if niveau in "65432":
        GPIO.output(G2LED, GPIO.HIGH)
    if niveau in "6543":
        GPIO.output(Y1LED, GPIO.HIGH)
    if niveau in "654":
        GPIO.output(Y2LED, GPIO.HIGH)
    if niveau in "65":
        GPIO.output(R1LED, GPIO.HIGH)
    if niveau in "6":
        GPIO.output(R2LED, GPIO.HIGH)
    niveau = ""

def veranderKleur():
    # Functie die de algemene lamp kleurt op algemeen geluidsniveau
    global LEDopslag
    global kleur
    global lcdKleur
    if (len(LEDopslag) >= 500): # Update de lamp elke 5 seconden
        LEDgemiddelde = statistics.median(LEDopslag)
        if (LEDgemiddelde <= 2):
            kleur = "G"
            lcdKleur = "Groen"
            os.system("pigs p 17 0")
            os.system("pigs p 22 20")
            os.system("pigs p 27 0")
            LEDopslag = []
        elif (LEDgemiddelde > 2 and LEDgemiddelde <= 4):
            kleur = "O"
            lcdKleur = "Oranje"
            os.system("pigs p 17 255")
            os.system("pigs p 22 10")
            os.system("pigs p 27 0")
            LEDopslag = []
        elif (LEDgemiddelde > 4):
            kleur = "R"
            lcdKleur = "Rood"
            os.system("pigs p 17 255")
            os.system("pigs p 22 0")
            os.system("pigs p 27 0")
            LEDopslag = []

def zendData():
    # Functie die een timer van 60 seconden laat lopen en na die timer de gemeten data naar de database stuurt
    global kleur
    global dB
    global s

    threading.Timer(600.0, zendData).start()

    send_data = "{}^{}".format(dB, kleur)
    #s.sendall(send_data.encode('utf-8'))

def showLCD():
    # Functie de elke 5 seconden de LCD scherm update met huidig gemeten decibel en kleur van de algemene LED
    global dB
    global lcdKleur
    global ADC_bytes

    threading.Timer(5.0, showLCD).start()
    dB = round((239.06 * math.log(ADC_bytes)) - 1372.3)

    lcd_string("Decibel: {}".format(dB), LCD_LINE_1)
    lcd_string("Kleur: {}".format(lcdKleur), LCD_LINE_2)
    

def lcd_init():
    lcd_byte(0x33,LCD_CMD) # 110011 Initialise
    lcd_byte(0x32,LCD_CMD) # 110010 Initialise
    lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display
  
  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def cleanUp():
    # Functie die na afsluit van het programma de GPIO pinnen en de lamp 'opruimen'
    LEDuit()
    GPIO.cleanup()
    os.system("pigs p 17 0")
    os.system("pigs p 22 0")
    os.system("pigs p 27 0")
        
if __name__ == '__main__':
    # Programma begint vanaf hier functies uit te voeren
    LEDsetup()
    bepaalMinWaardes()
    connect()
    showLCD()
    zendData()
    os.system("sudo pigpiod")
    while True:
        try:
            bepaalNiveau()
            liveLED()
            veranderKleur()
        except KeyboardInterrupt:
            lcd_byte(0x01, LCD_CMD)
            lcd_string("Programma", LCD_LINE_1)
            lcd_string("onderbroken!", LCD_LINE_2)
            cleanUp()
            break
