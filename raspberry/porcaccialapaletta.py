#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Smart Parking - Versione con controllo manuale
"""

import signal
import threading
import sys
from time import sleep
from datetime import datetime
import queue

# Importazione librerie esterne
import serial
import buildhat
from colorama import Fore, Style, init as colorama_init
from buildhat import Motor, ColorSensor

# Importazione moduli interni
from ColorSensorA import ColorSensorA
from ServoMotor import ServoMotor
from UltrasonicSensor import UltrasonicSensor
from robot import Robot

# Inizializzazione colorama per colori cross-platform
colorama_init(autoreset=True)

# --------------------------
# COSTANTI DI CONFIGURAZIONE
# --------------------------
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
MAX_SERIAL_RETRIES = 5
SERIAL_DELAY = 0.5  # Pausa tra tentativi (secondi)

# --------------------------
# CONFIGURAZIONE LOGGER
# --------------------------
COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED + Style.BRIGHT,
    "CRITICAL": Fore.RED + Style.BRIGHT,
    "SYSTEM": Fore.MAGENTA,
    "PARKING": Fore.BLUE + Style.BRIGHT,
}

# --------------------------
# VARIABILI GLOBALI
# --------------------------
serial_lock = threading.Lock()
shutdown_flag = threading.Event()

# --------------------------
# FUNZIONI DI UTILITY
# --------------------------
def log(msg, level="INFO"):
    """Registra messaggi con timestamp e colori

    Args:
        msg (str): Messaggio da registrare
        level (str): Livello di gravità
    """
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    reset = Style.RESET_ALL
    print(f"{colore}[{level}] {ts}: {msg}{reset}")

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

def termina_programma():
    """Spegne il sistema in modo controllato"""
    log("Avvio shutdown programmato...", "SYSTEM")
    shutdown_flag.set()
    
    try:
        robot.stop_movimento()
        
        with serial_lock:
            arduino.write(b'3\n')  # Comando spegnimento Arduino
            arduino.flush()
    except Exception as e:
        log(f"Errore invio comando shutdown: {str(e)}", "WARN")
    
    sys.exit(0)

# --------------------------
# FUNZIONI MANUALI DI CONTROLLO
# --------------------------
def muovi_avanti(durata=2, velocita=40):
    """Muove il robot in avanti per un tempo specificato
    
    Args:
        durata (float): Durata del movimento in secondi
        velocita (int): Velocità del movimento (0-100)
    """
    log(f"Muovo avanti per {durata} secondi a velocità {velocita}", "INFO")
    robot.muovi_avanti_for("seconds", durata, speed=velocita)

def muovi_indietro(durata=2, velocita=40):
    """Muove il robot indietro per un tempo specificato
    
    Args:
        durata (float): Durata del movimento in secondi
        velocita (int): Velocità del movimento (0-100)
    """
    log(f"Muovo indietro per {durata} secondi a velocità {velocita}", "INFO")
    robot.muovi_indietro_for("seconds", durata, speed=velocita)

def gira_destra(gradi=90, velocita=30):
    """Fa girare il robot a destra
    
    Args:
        gradi (int): Gradi di rotazione
        velocita (int): Velocità di rotazione (0-100)
    """
    log(f"Giro a destra di {gradi} gradi a velocità {velocita}", "INFO")
    robot.gira_destra(gradi, speed=velocita)

def gira_sinistra(gradi=90, velocita=30):
    """Fa girare il robot a sinistra
    
    Args:
        gradi (int): Gradi di rotazione
        velocita (int): Velocità di rotazione (0-100)
    """
    log(f"Giro a sinistra di {gradi} gradi a velocità {velocita}", "INFO")
    robot.gira_sinistra(gradi, speed=velocita)

def stop():
    """Ferma immediatamente il robot"""
    log("Stop movimento", "INFO")
    robot.stop_movimento()

def abbassa_gabbia():
    """Abbassa la gabbia/pinza usando il motore nella porta B"""
    log("Abbasso la gabbia usando Motor B", "INFO")
    # Uso del motore per abbassare la gabbia
    # Il valore positivo/negativo dipende dalla configurazione meccanica
    gabbia.run_for_seconds(0.25, -70)  # Velocità negativa per abbassare
    sleep(0.5)  # Pausa per stabilizzazione

def alza_gabbia():
    """Alza la gabbia/pinza usando il motore nella porta B"""
    log("Alzo la gabbia usando Motor B", "INFO")
    # Uso del motore per alzare la gabbia
    # Il segno opposto rispetto all'abbassamento
    gabbia.run_for_seconds(0.25, 70)  # Velocità positiva per alzare
    sleep(0.5)  # Pausa per stabilizzazione


def chiudi_gabbia():
    """Chiude la gabbia/pinza"""
    log("Chiudo la gabbia", "INFO")
    gabbia.run_for_seconds(1, 50)

def apri_gabbia():
    """Apre la gabbia/pinza"""
    log("Apro la gabbia", "INFO")
    gabbia.run_for_seconds(1, -50)

def prendi_auto():
    """Effettua la manovra per prendere un'auto"""
    log("Prelievo auto in corso...", "PARKING")
    
    # Abbassiamo la gabbia/pinza per prendere l'auto
    abbassa_gabbia()
    
    # Chiusura pinza/gabbia 
    chiudi_gabbia()
    
    log("Auto prelevata con successo", "PARKING")

def rilascia_auto():
    """Effettua la manovra per rilasciare un'auto"""
    log("Rilascio auto in corso...", "PARKING")
    
    # Apertura pinza/gabbia
    apri_gabbia()
    
    # Alziamo la gabbia/pinza
    alza_gabbia()
    
    log("Auto rilasciata con successo", "PARKING")

def orienta_sensore(angolo=0):
    """Orienta il sensore di colore a un angolo specifico
    
    Args:
        angolo (int): Angolo di orientamento (0-180)
    """
    log(f"Oriento il sensore a {angolo} gradi", "INFO")
    servo.set_angle(angolo)
    sleep(0.3)  # Attesa stabilizzazione

def leggi_distanza_frontale():
    """Legge la distanza frontale dall'ostacolo
    
    Returns:
        float: Distanza in cm
    """
    try:
        distanza = ultrasonic.get_distance()
        log(f"Distanza frontale: {distanza} cm", "INFO")
        return distanza
    except Exception as e:
        log(f"Errore lettura distanza frontale: {str(e)}", "ERROR")
        return -1

def leggi_distanza_laterale():
    """Legge la distanza laterale dall'ostacolo
    
    Returns:
        float: Distanza in cm
    """
    try:
        distanza = ultrasonicLaterale.get_distance()
        log(f"Distanza laterale: {distanza} cm", "INFO")
        return distanza
    except Exception as e:
        log(f"Errore lettura distanza laterale: {str(e)}", "ERROR")
        return -1

def leggi_colore():
    """Legge il colore con il sensore
    
    Returns:
        tuple: Valori RGB del colore rilevato
    """
    try:
        rgb = colorLego.get_color()
        log(f"Colore rilevato RGB: {rgb}", "INFO")
        return rgb
    except Exception as e:
        log(f"Errore lettura colore: {str(e)}", "ERROR")
        return None

def identifica_colore_officina():
    """Identifica il colore dell'officina disponibile
    
    Returns:
        str: Colore dell'officina (verde, giallo o sconosciuto)
    """
    log("Rilevamento colore officina disponibile...", "PARKING")
    
    # Orienta il sensore verso la parete
    orienta_sensore(90)
    
    # Leggi il colore con il sensore
    rgb = leggi_colore()
    
    # Identifica il colore
    if rgb is None:
        return "sconosciuto"
    
    r, g, b = rgb
    
    if g > 150 and r < 80 and b < 80:
        log("Officina VERDE attiva", "PARKING")
        return "verde"
    elif r > 150 and g > 150 and b < 80:
        log("Officina GIALLA attiva", "PARKING")
        return "giallo"
    else:
        log("Impossibile identificare colore officina", "WARN")
        return "sconosciuto"

def identifica_colore_auto():
    """Identifica il colore dell'auto di fronte al robot
    
    Returns:
        str: Colore dell'auto (rosso, verde, giallo, blu o sconosciuto)
    """
    log("Rilevamento colore auto...", "PARKING")
    
    # Orienta il sensore verso l'auto
    orienta_sensore(0)
    
    # Leggi il colore con il sensore
    rgb = leggi_colore()
    
    # Identifica il colore
    if rgb is None:
        return "sconosciuto"
    
    r, g, b = rgb
    
    if r > 150 and g < 80 and b < 80:
        log("Auto ROSSA rilevata", "PARKING")
        return "rosso"
    elif g > 150 and r < 80 and b < 80:
        log("Auto VERDE rilevata", "PARKING")
        return "verde"
    elif r > 150 and g > 150 and b < 80:
        log("Auto GIALLA rilevata", "PARKING")
        return "giallo"
    elif b > 150 and r < 80 and g < 80:
        log("Auto BLU rilevata", "PARKING")
        return "blu"
    else:
        log("Colore auto non identificato", "WARN")
        return "sconosciuto"


def inizializza_sensore(sensore, nome_sensore):
    """Funzione per inizializzare un sensore e gestire eventuali errori.
    
    Args:
        sensore: Il sensore da inizializzare.
        nome_sensore (str): Il nome del sensore per il log.
        
    Returns:
        sensore: Il sensore inizializzato o None se non riuscito.
    """
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

# --------------------
# PERCORSO
# --------------------

def esegui_percorso():
    """Esegue un percorso personalizzato con un sensore di colore Arduino per fermarsi sul rosso"""
    log("Inizio esecuzione percorso personalizzato", "SYSTEM")
    
    # Avanza finché non trova il rosso
    log("Avanzo fino a rilevare il colore ROSSO con il sensore Arduino", "PARKING")
    
    # Avvia il movimento in avanti a velocità moderata
    robot.muovi_avanti(speed=40)
    
    # Loop di controllo colore
    colore_trovato = False
    while not colore_trovato and not shutdown_flag.is_set():
        try:
            # Leggi il colore dal sensore Arduino
            colore = color1.get_color()  # Usando color1 (Arduino) invece di colorLego (porta A)
            log(f"Colore rilevato da Arduino: {colore}", "INFO")
            
            # Verifica se è rosso (adatta questa logica in base a come color1 riporta i colori)
            if str(colore).upper() == "RED" or str(colore).upper() == "ROSSO" or (isinstance(colore, tuple) and colore[0] > 150 and colore[1] < 80 and colore[2] < 80):
                log("COLORE ROSSO RILEVATO! Mi fermo.", "PARKING")
                colore_trovato = True
                muovi_avanti(1, 40)
            else:
                # Breve attesa per non sovraccaricare la lettura
                sleep(0.1)
        except Exception as e:
            log(f"Errore durante la lettura del colore: {str(e)}", "ERROR")
            sleep(0.5)  # Pausa più lunga in caso di errore
    
    # Se non ha trovato il rosso ma è uscito per shutdown, ferma il robot
    if not colore_trovato:
        robot.stop_movimento()
        log("Percorso interrotto senza trovare il rosso", "WARN")
        return
    
    # Prosegui con il resto del percorso
    gira_destra(90, 40)         # Gira a destra di 90 gradi
    muovi_avanti(1.5, 40)       # Avanti per 1.5 secondi
    abbassa_gabbia()            # Abbassa la gabbia
    chiudi_gabbia()             # Chiudi la pinza
    alza_gabbia()               # Alza la gabbia
    gira_sinistra(90, 40)       # Gira a sinistra
    muovi_avanti(3, 80)         # Avanti per 3 secondi
    abbassa_gabbia()            # Abbassa per depositare
    apri_gabbia()               # Apri la pinza
    muovi_indietro(1, 50)       # Indietro per allontanarsi
    
    log("Percorso personalizzato completato", "SYSTEM")






# --------------------------
# ENTRY POINT
# --------------------------
if __name__ == "__main__":
    # Cattura il segnale di interruzione
    signal.signal(signal.SIGINT, lambda s, f: termina_programma())
    
    try:
        # Inizializza la connessione seriale
        arduino = safe_serial_connect()
        # Inizializzazione dei sensori
        colorLego = inizializza_sensore(buildhat.ColorSensor('A'), "ColorSensor")
        color1 = inizializza_sensore(ColorSensorA(arduino, serial_lock, "COL1", "5"), "ColorSensorA")
        ultrasonic = inizializza_sensore(UltrasonicSensor(arduino, serial_lock), "UltrasonicSensor")
        ultrasonicLaterale = inizializza_sensore(UltrasonicSensor(arduino, serial_lock, command_code="6", sensor_id="DIST2"), "UltrasonicSensor Laterale")
        servo = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO1"), "ServoMotor 1")
        servo_alza = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO2", min_angle=0, max_angle=360), "ServoMotor 2")
        robot = Robot('C', 'D')
        gabbia = Motor('B')
        # Avvia il controllo manuale
        esegui_percorso()
        
    except Exception as e:
        log(f"Errore critico: {str(e)}", "CRITICAL")
        
    finally:
        # Chiudi le connessioni e rilascia le risorse
        shutdown_flag.set()
        
        try:
            if 'arduino' in locals() and arduino.is_open:
                arduino.close()
        except Exception as e:
            log(f"Errore chiusura seriale: {str(e)}", "WARN")
        
        log("Applicazione terminata", "SYSTEM")