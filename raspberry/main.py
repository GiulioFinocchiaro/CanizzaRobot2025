# -*- coding: utf-8 -*-
"""
Programma di controllo per robot con Arduino
Autore: Giulio Finocchiaro
Versione: 1.1
"""  # Avanzi va, pi sta vota l'autore lo lasciamo - Zitto Costa

# Importazione librerie standard
import signal
import threading
import os
import sys
from time import sleep, time
from datetime import datetime

# Importazione librerie esterne
import buildhat
import serial
from colorama import Fore, Back, Style, init as colorama_init

# Importazione moduli interni
from ColorSensorA import ColorSensorA
from ServoMotor import ServoMotor
from UltrasonicSensor import UltrasonicSensor
from buildhat import Motor
from robot import Robot

# Inizializzazione colori per logging
colorama_init(autoreset=True)

# --------------------------
# COSTANTI DI CONFIGURAZIONE
# --------------------------
TEMPO_TIMER = 180
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
MAX_SERIAL_RETRIES = 5
SERIAL_DELAY = 1
TIMEOUT_START = 5

# --------------------------
# CONFIGURAZIONE LOGGER
# --------------------------
COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED + Back.WHITE,
    "SYSTEM": Fore.MAGENTA,
    "SUCCESS": Fore.GREEN + Style.BRIGHT,
    "DATA": Fore.WHITE,
    "RESET": Style.RESET_ALL
}

# --------------------------
# INIZIALIZZAZIONE GLOBALS
# --------------------------
serial_lock = threading.Lock()
shutdown_flag = threading.Event()


# ========================
# FUNZIONE DI LOGGING
# ========================
def log(msg, level="INFO"):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    reset = COLORI_LOG["RESET"]
    print(f"{colore}[{level}] {ts}: {msg}{reset}")


# ========================
# GESTIONE SERIALE
# ========================
def safe_serial_connect(port=SERIAL_PORT, baud=BAUDRATE, timeout=1.5):
    """Crea connessione seriale con sistema di retry

    Returns:
        serial.Serial: Oggetto seriale connesso

    Raises:
        SystemExit: Se fallisce dopo MAX_SERIAL_RETRIES tentativi
    """
    for attempt in range(1, MAX_SERIAL_RETRIES + 1):
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
            log(f"Connessione seriale riuscita su {port} (tentativo {attempt})", "SYSTEM")
            return ser
        except (serial.SerialException, OSError) as e:
            log(f"Errore connessione seriale: {str(e)}", "WARN")
            sleep(SERIAL_DELAY)

    log("Connessione seriale non disponibile", "ERROR")
    sys.exit(1)


arduino = safe_serial_connect()


# ========================
# INIZIALIZZAZIONE HARDWARE
# ========================
def inizializza_sensore(sensore, nome_sensore):
    try:
        log(f"Inizializzazione sensore {nome_sensore}...", "INFO")
        return sensore
    except Exception as e:
        log(f"Errore inizializzazione sensore {nome_sensore}: {str(e)}", "ERROR")
        risposta = input(f"Il sensore {nome_sensore} non è stato rilevato. Vuoi continuare senza di esso? (y/n): ")
        if risposta.lower() == 'y':
            log(f"Bypassato sensore {nome_sensore}.", "WARN")
            return None
        else:
            log(f"Interruzione a causa della mancata inizializzazione di {nome_sensore}.", "CRITICAL")
            sys.exit(1)


try:
    colorLego = inizializza_sensore(buildhat.ColorSensor('A'), "ColorSensor")
    color1 = inizializza_sensore(ColorSensorA(arduino, serial_lock, "COL1", "5"), "ColorSensorA")
    ultrasonic = inizializza_sensore(UltrasonicSensor(arduino, serial_lock), "UltrasonicSensor")
    ultrasonicLaterale = inizializza_sensore(
        UltrasonicSensor(arduino, serial_lock, command_code="6", sensor_id="DIST2"), "UltrasonicSensor Laterale")
    servo = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO1"), "ServoMotor 1")
    servo_alza = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO2", min_angle=0, max_angle=360),
                                     "ServoMotor 2")
    arduino = safe_serial_connect()
    robot = Robot('C', 'D')
    gabbia = Motor('B')
except Exception as e:
    log(f"Errore inizializzazione hardware: {str(e)}", "ERROR")
    sys.exit(1)


# ========================
# FUNZIONI DI CONTROLLO ROBOT
# ========================
def restart_program():
    log("Inizio procedura di riavvio...", "SYSTEM")
    shutdown_flag.set()
    try:
        with serial_lock:
            if arduino.is_open:
                arduino.write(b'0')
                arduino.flush()
                arduino.close()
    except Exception as e:
        log(f"Errore durante il reset: {str(e)}", "WARN")
    os.execl(sys.executable, sys.executable, *sys.argv)

def muovi_avanti(durata=2, velocita=40):
    log(f"Muovo avanti per {durata} secondi a velocità {velocita}", "INFO")
    robot.muovi_avanti_for("seconds", durata, speed=velocita)

def muovi_indietro(durata=2, velocita=40):
    log(f"Muovo indietro per {durata} secondi a velocità {velocita}", "INFO")
    robot.muovi_indietro_for("seconds", durata, speed=velocita)

def gira_destra(gradi=90, velocita=30):
    log(f"Giro a destra di {gradi} gradi a velocità {velocita}", "INFO")
    robot.gira_destra(gradi, speed=velocita)

def gira_sinistra(gradi=90, velocita=30):
    log(f"Giro a sinistra di {gradi} gradi a velocità {velocita}", "INFO")
    robot.gira_sinistra(gradi, speed=velocita)

def stop():
    log("Stop movimento", "INFO")
    robot.stop_movimento()

def abbassa_gabbia():
    log("Abbasso la gabbia usando Motor B", "INFO")
    gabbia.run_for_seconds(0.25, -70)
    sleep(0.5)

def alza_gabbia():
    log("Alzo la gabbia usando Motor B", "INFO")
    gabbia.run_for_seconds(0.25, 70)
    sleep(0.5)

def chiudi_gabbia():
    log("Chiudo la gabbia", "INFO")
    gabbia.run_for_seconds(1, 50)

def apri_gabbia():
    log("Apro la gabbia", "INFO")
    gabbia.run_for_seconds(1, -50)

def prendi_auto():
    log("Prelievo auto in corso...", "PARKING")
    abbassa_gabbia()
    chiudi_gabbia()
    log("Auto prelevata con successo", "PARKING")

def rilascia_auto():
    log("Rilascio auto in corso...", "PARKING")
    apri_gabbia()
    alza_gabbia()
    log("Auto rilasciata con successo", "PARKING")

def orienta_sensore(angolo=0):
    log(f"Oriento il sensore a {angolo} gradi", "INFO")
    servo.set_angle(angolo)
    sleep(0.3)

def leggi_distanza_frontale():
    try:
        distanza = ultrasonic.get_distance()
        log(f"Distanza frontale: {distanza} cm", "INFO")
        return distanza
    except Exception as e:
        log(f"Errore lettura distanza frontale: {str(e)}", "ERROR")
        return -1

def leggi_distanza_laterale():
    try:
        distanza = ultrasonicLaterale.get_distance()
        log(f"Distanza laterale: {distanza} cm", "INFO")
        return distanza
    except Exception as e:
        log(f"Errore lettura distanza laterale: {str(e)}", "ERROR")
        return -1

def leggi_colore():
    try:
        if color1:
            rgb = color1.get_color_rgb()
            log(f"Colore rilevato: {rgb}", "INFO")
            return rgb
        else:
            log("Sensore di colore non disponibile", "WARN")
            return None
    except Exception as e:
        log(f"Errore lettura colore: {str(e)}", "ERROR")
        return None
