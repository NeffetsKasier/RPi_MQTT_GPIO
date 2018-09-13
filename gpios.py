# coding=utf-8
#~~~~~~~~~~~ Benoetigte Module importieren [START] ~~~~~~~~~~~
from Adafruit_ADS1x15 import ADS1x15
import paho.mqtt.client as mqtt 
import RPi.GPIO as GPIO
import time, signal, sys, os
#~~~~~~~~~~~ Benoetigte Module importieren [ENDE] ~~~~~~~~~~~

#~~~~~~~~~~~ MQTT Client initialisieren [START] ~~~~~~~~~~~~~
# Auf Wireless-Verbindung warten bevor MQTT gestartet wird
time.sleep(1) 
mqtt_name="Modellhaus"
mqtt_user = "openadmin"
mqtt_pw = "adminopen"
mqtt_ip = "192.168.0.2"
sleep_between_publish = 5

# Login fuer Broker

#~~~~~~~~~~~ MQTT Client initialisieren [ENDE] ~~~~~~~~~~~

# Pins ueber GPIO-Nummerierung ansprechen (alternativ: "board")
GPIO.setmode(GPIO.BCM) 
GPIO.setwarnings(False)

# Hier wird der ADC initialisiert - beim KY-053 verwendeten ADC handelt es sich um einen 16bit Chipsatz
#adc = ADS1x15(ic=0x01)



#~~~~~~~~~~~ Klasse Sensor definieren [START] ~~~~~~~~~~~
class Pin:
	name = ""
	gpio_pin = 0
	gpio_status = 0 #0 oder 1
	mqtt_sub_channel = ""
	mqtt_pub_channel = ""
	mqtt_client = ""

	def __init__(self, name, gpio_pin=0):
		self.name = name
		if (gpio_pin != 0):
			self.gpio_pin = gpio_pin
		self.set_mqtt_client()
	
	def set_mqtt_client(self):
		# Eindeutige Bezeichnung des Client um von Broker identifiziert werden zu koennen
		self.mqtt_client = mqtt.Client(client_id=self.name)
		self.mqtt_client.username_pw_set(mqtt_user, password=mqtt_pw)
		self.mqtt_client.connect(mqtt_ip, port=1883, keepalive=60)
		self.mqtt_client.loop_start()
		self.mqtt_sub_channel = mqtt_name+"/"+self.name+"/SetGpio"
		self.mqtt_pub_channel = mqtt_name+"/"+self.name+"/Status"
		self.mqtt_client.subscribe(self.mqtt_sub_channel)
		self.mqtt_client.publish(self.mqtt_pub_channel, self.gpio_status)
		print "~~~~~~~~~  "+self.name+" wurde gestartet ~~~~~~~~~"
		print "MQTT Channel: "+self.mqtt_pub_channel

	
class DigitalSensor(Pin):
	def __init__(self, name, gpio_pin,mode=""):
		Pin.__init__(self,name,gpio_pin)
		if mode=="up":
			GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		elif mode=="down":
			GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		elif mode=="off":
			GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
		else:
			GPIO.setup(self.gpio_pin, GPIO.IN)
		self.pub_gpio_status()
		
	def get_gpio_status(self):
		self.gpio_status = GPIO.input(self.gpio_pin)
		return self.gpio_status
	
	def pub_gpio_status(self):
		status = self.get_gpio_status()
		print(self.name+": "+str(status)+" [mqtt publish event]")
		self.mqtt_client.publish(self.mqtt_pub_channel, status)

	def mqtt_publish_callback(self,channel):
		self.pub_gpio_status() 
	
	def digital_event_listener(self, mode, bounce):
		if mode == "falling" :
			GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING,bouncetime=bounce)
		elif mode == "rising":
			GPIO.add_event_detect(self.gpio_pin, GPIO.RISING, bouncetime=bounce)
		else:
			GPIO.add_event_detect(self.gpio_pin, GPIO.BOTH, bouncetime=bounce)
		GPIO.add_event_callback(self.gpio_pin,self.mqtt_publish_callback)
		print "~~~~~~~~~~~  "+self.name+" Event Listener wird ausgefuehrt [modus:"+mode+"]~~~~~~~~~~~"
	
class Aktor(Pin):
	def __init__(self, name, gpio_pin):
		Pin.__init__(self,name,gpio_pin)
		GPIO.setup(self.gpio_pin, GPIO.OUT)
		self.set_gpio_status(0)
		self.mqtt_client.on_message=self.on_message
	
	def get_gpio_status(self):
		return self.gpio_status
	
	def pub_gpio_status(self):
		status = self.get_gpio_status()
		print(self.name+": "+str(status)+" [mqtt publish event]")
		self.mqtt_client.publish(self.mqtt_pub_channel, status)
	
	def set_gpio_status(self,status):
		self.gpio_status = status
		GPIO.output(self.gpio_pin, self.gpio_status)
		self.pub_gpio_status()
		
	# The callback for when a PUBLISH message is received from the server.
	def on_message(self, client, userdata, msg):
		set_request = int(msg.payload)
		print 'Topic: ' + msg.topic + '   Message: ' + str(set_request)
		self.set_gpio_status(set_request)
		
	def switch (self):
		status = self.get_gpio_status()
		if status == 0:
			self.set_gpio_status(1)
		else:
			self.set_gpio_status(0)

class Pwm_Aktor(Pin):
	pwm = 0
	def __init__(self, name, gpio_pin,freq):
		Pin.__init__(self,name,gpio_pin)
		GPIO.setup(self.gpio_pin, GPIO.OUT)
		self.pwm = GPIO.PWM(self.gpio_pin, freq)
		self.set_gpio_status(0)
		self.mqtt_client.on_message=self.on_message
	
	def get_gpio_status(self):
		return self.gpio_status
	
	def pub_gpio_status(self):
		status = self.get_gpio_status()
		print(self.name+": "+str(status)+" [mqtt publish event]")
		self.mqtt_client.publish(self.mqtt_pub_channel, status)
	
	def set_gpio_status(self,status):
		self.gpio_status = status
		if self.gpio_status==1:
			self.pwm.start(50)
		else:
			self.pwm.stop()	
		self.pub_gpio_status()
		
	# The callback for when a PUBLISH message is received from the server.
	def on_message(self, client, userdata, msg):
		set_request = int(msg.payload)
		print 'Topic: ' + msg.topic + '   Message: ' + str(set_request)
		self.set_gpio_status(set_request)
		
	def switch (self):
		status = self.get_gpio_status()
		if status == 0:
			self.set_gpio_status(1)
		else:
			self.set_gpio_status(0)
	
			
adc = ADS1x15(ic=0x01)
class Analog_Sensor(Pin):
	adc_pin = 0 # Pin [0|1|2|3]
	# Verstaerkung (Gain) wird ausgewaehlt
	gain = 0  # [4096|2048|1024|512|256] +/- 4.096V,2.048V,..0,256V
	# Abtasterate des ADC (SampleRate) wird ausgewaehlt
	sps = 0    # [8|16|32|64|128|250|475|860] Samples pro Sekunde
	oldBeatTime = 0
	isPeak = False
	result = False
	maxValue = 0
	
	def __init__(self, name,adc_pin,gain,sps):
		Pin.__init__(self,name)
		self.adc_pin = adc_pin
		self.gain=gain
		self.sps=sps
	
	def pub_gpio_status(self,value):
		print(self.name+": "+str(value)+" [mqtt publish event]")
		self.mqtt_client.publish(self.mqtt_pub_channel, value)

				
	def heartBeatDetect(self,schwelle):
		# Benutzte Variablen werden initialisiert
		beatsPerMinute = 0
		schwelle = schwelle
		beatTime = 0

		# Hier wird der aktuelle Spannungswert am Fototransistor ausgelesen
		# und in der rawValue - Variable zwischengespeichert
		# Mit "adc_channel" wird der am ADC angeschlossene Channel ausgewaehlt
		rawValue = adc.readADCSingleEnded(self.adc_pin, self.gain, self.sps)

		# Reset der Ergebnis Variable
		if self.result == True:
			self.result = False

		# Sollte der aktuelle Wert vom letzten maximalen Wert zu weit abweichen
		# (z.B.da der Finger neu aufgesetzt oder weggenommen wurde)
		# So wird der MaxValue resetiert, um eine neue Basis zu erhalten.
		if rawValue * 4 < self.maxValue:
			  self.maxValue = rawValue * 0.8;

		# Hier wurd der eigentliche Peak detektiert. Sollte ein neuer RawValue groeßer sein
		# als der letzte maximale Wert, so wird das als Spitze der aufgezeichneten Daten erkannt.
		if rawValue > (self.maxValue - schwelle):

			  if rawValue > self.maxValue:
					self.maxValue = rawValue
			  # Zum erkannten Peak soll nur ein Herzschlag zugewiesen werden
			  if self.isPeak == False:
					self.result = True

			  self.isPeak = True

		else:
			if rawValue < self.maxValue - schwelle:
			  self.isPeak = False
			  # Hierbei wird der maximale Wert bei jedem Durchlauf
			  # etwas wieder herabgesetzt. Dies hat den Grund, dass
			  # nicht nur der Wert sonst immer stabil bei jedem Schlag
			  # gleich oder kleiner als maxValue sein wuerde, sondern auch,
			  # falls der Finder sich minimal bewegen sollte und somit
			  # das Signal generell schwaecher sein sollte.
			self.maxValue = self.maxValue - schwelle/2

		# Wurde in der oberen Abfrage ein Herzschlag detektiert, so wird nun die Ausgabe freigegeben
		if self.result == True:

			# Berechnung des Puls
			# Hierbei wird bei jedem registrierten Herzschlag die System-Zeit aufgenommen
			# Beim naechsten Herzschlag wird dann die aktuelle Systemzeit mit der gespeicherten verglichen
			# Die Differenz der beiden ergibt dann die Zeit zwischen den Herz-Schlaegen
			# womit man dann auch den Puls berechnen kann.
			beatTime = time.time()
			timedifference = beatTime - self.oldBeatTime
			beatsPerMinute = 60/timedifference
			self.oldBeatTime = beatTime
			# Erechneter Puls wird der Funktion übergeben
			herzschlag.pub_gpio_status(beatsPerMinute)
	
	
	
	
	
	

##DigitalSensoren
klopf_sensor = DigitalSensor("Klopf-Sensor",11,"up")
klopf_sensor.digital_event_listener("falling",100)
flammen_sensor = DigitalSensor("Flammen-Sensor",5,"up")
flammen_sensor.digital_event_listener("rising",100)
neigung_sensor = DigitalSensor("Neigungs-Sensor",6,"up")
neigung_sensor.digital_event_listener("falling",100)
thermo_sensor = DigitalSensor("Thermo-Sensor",4,"off")
thermo_sensor.digital_event_listener("rising",100)
druck_schalter = DigitalSensor("Druck-Schalter",26,"up")
druck_schalter.digital_event_listener("rising",100)
fenster = DigitalSensor("Fenster",27,"up")
fenster.digital_event_listener("falling",100)

##Aktoren
treppe = Aktor("Treppen-Beleuchtung",10)
steckdose = Aktor("Aussen-Steckdose",9)
beleuchtung = Aktor("Aussen-Beleuchtung",13)
ventilator = Aktor("Ventilator",19)
alarm = Pwm_Aktor("Alarm-Buzzer",15,2500)

#herzschlag = Analog_Sensor("Herzschlag",1,4096,8)

# Hauptprogrammschleife
try:
        while True:
			#herzschlag.heartBeatDetect(50)	
			time.sleep(sleep_between_publish)			
			##DigitalSensoren
			druck_schalter.pub_gpio_status()
			fenster.pub_gpio_status()
			klopf_sensor.pub_gpio_status()
			thermo_sensor.pub_gpio_status()
			flammen_sensor.pub_gpio_status()
			neigung_sensor.pub_gpio_status()

				

			
# Aufraeumarbeiten nachdem das Programm beendet wurde
except KeyboardInterrupt:
        GPIO.cleanup()
