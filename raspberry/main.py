import serial
import time
from buildhat import MotorPair, Motor
import threading

arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) #Dai ma Arduino scritto male...
spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

route = MotorPair('A', 'B')
pinza_spike = Motor('C')

# Così quì lasciamo tutte le variabili
colore1 = None
colore2 = None
distanza_pros = None
dati_lock = threading.Lock()


def send_to_arduino(command):
    arduino.write(f"{command}\n".encode())

def send_to_spike(command):
    spike.write(f"{command}\n".encode())

def receive_from_spike():
    global colore1, colore2, distanza_pros
    while True:
        if spike.in_waiting:
            riga = spike.readline().decode().strip()
            try:
                parti = riga.split("|")
                dati = {}
                for parte in parti:
                    chiave, valore = parte.split(":")
                    dati[chiave] = valore
                with dati_lock:
                    colore1 = dati.get("COL1")
                    colore2 = dati.get("COL2")
                    distanza_pros = float(dati.get("DIST"))
            except Exception as e:
                print(f"Errore nel parsing dati Spike: {e}")
        time.sleep(0.05)

def get_dati():
    with dati_lock:
        return colore1, colore2, distanza_pros

def main():
    try:
        while True:
            col1, col2, dist = get_dati()
            if dist < 15:
                route.stop()
                send_to_arduino("APRI")
                pinza_spike.run_for_seconds(1)
                send_to_arduino("CHIUDI")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nChiusura programma...")
        arduino.close()
        spike.close()
        route.stop()

if __name__ == "__main__":
    thread_ricezione = threading.Thread(target=receive_from_spike, daemon=True)
    thread_ricezione.start()
    main()

# Versione vecchia, non so se si possa vedere direttamente con Github lo uso poco, almeno così è salvata casomai. Vedi com'è fammi sapere.
#import serial
#import time
#from buildhat import MotorPair, Motor
#import threading

#arudino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
#spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

#route = MotorPair('A', 'B')
#pinza_spike = Motor('C')

#colore1 = None
#colore2 = None
#distanza_pros = None

#dati_lock = threading.Lock()

#def send_to_arduino(command):
#arudino.write(f"{command}\n".encode())

#def send_to_spike(command):
#spike.write(f"{command}\n".encode())

#def recive_from_spike():
#global colore1, colore2, distanza_pros
#if spike.in_waiting:
#riga = spike.readline().decode().strip()
#try:
#parti = riga.split("|")
#dati = {}
#for parte in parti:
#chiave, valore = parte.split(":")
#dati[chiave] = valore

#colore1 = dati.get("COL1")
#colore2 = dati.get("COL2")
#distanza_pros = float(dati.get("DIST"))
#except Exception as e:
#print(f"Errore nel parsing dati Spike: {e}")
#time.sleep(0.05)

#def main():
#return 1

#read_from_spike_thread = threading.Thread(target=recive_from_spike(), daemon=True)
#read_from_spike_thread.start()

#main()
