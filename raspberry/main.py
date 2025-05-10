# -*- coding: utf-8 -*-
"""
Programma di controllo per robot con Arduino
Autore: Giulio Finocchiaro
Versione: 1.1
"""  #Avanzi va, pi sta vota l'autore lo lasciamo - Zitto Costa

# Importazione librerie standard
import signal
import threading
import os
import sys
from time import sleep, time
from datetime import datetime

import buildhat
# Importazione librerie esterne
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
TEMPO_TIMER = 180  # Durata timer sicurezza (secondi)
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
MAX_SERIAL_RETRIES = 5
SERIAL_DELAY = 0.5  # Pausa tra tentativi (secondi)
TIMEOUT_START = 5  # Timeout attesa comando start (secondi)

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
    """Registra messaggi con timestamp e colori

    Args:
        msg (str): Messaggio da registrare
        level (str): Livello di gravità (DEBUG/INFO/WARN/ERROR/SYSTEM/SUCCESS/DATA)
    """
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


# Inizializza connessione seriale
arduino = safe_serial_connect()

# ========================
# INIZIALIZZAZIONE HARDWARE
# ========================
try:
    color1 = ColorSensorA(arduino, serial_lock, "COL1", "5")
    ultrasonic = UltrasonicSensor(arduino, serial_lock)
    ultrasonicLaterale = UltrasonicSensor(arduino, serial_lock, command_code="6", sensor_id="DIST2")
    servo = ServoMotor(arduino, serial_lock, "SERVO1")
    servo_alza = ServoMotor(arduino, serial_lock, "SERVO2", min_angle=0, max_angle=360)



except Exception as e:
    log(f"Errore inizializzazione hardware: {str(e)}", "ERROR")
    sys.exit(1)

coloreLego = buildhat.ColorSensor('A')
robot = Robot('C', 'D')
gabbia = Motor('B')

# ========================
# FUNZIONI DI EMERGENZA
# ========================
def restart_program():
    """Riavvia completamente l'applicazione"""
    log("Inizio procedura di riavvio...", "SYSTEM")
    shutdown_flag.set()

    try:
        with serial_lock:
            if arduino.is_open:
                arduino.write(b'0')  # Comando reset Arduino
                arduino.flush()
                arduino.close()
    except Exception as e:
        log(f"Errore durante il reset: {str(e)}", "WARN")

    os.execl(sys.executable, sys.executable, *sys.argv)


def termina_programma():
    """Spegne il sistema in modo controllato"""
    log("Avvio shutdown programmato...", "SYSTEM")
    shutdown_flag.set()

    try:
        with serial_lock:
            arduino.write(b'3')  # Comando spegnimento Arduino
    except Exception as e:
        log(f"Errore invio comando shutdown: {str(e)}", "WARN")

    restart_program()


# ========================
# THREAD DI SICUREZZA
# ========================
def check_shutdown():
    """Monitora la seriale per comandi di emergenza"""
    buffer = bytearray()
    target_message = b"SYS|3"

    while not shutdown_flag.is_set():
        try:
            # Lettura non bloccante
            with serial_lock:
                data = arduino.read(arduino.in_waiting or 1)

            if data:
                buffer.extend(data)

                # Processa tutte le righe complete
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    if target_message in line:
                        log("Ricevuto comando di shutdown remoto", "SYSTEM")
                        termina_programma()
                        return

                # Controllo dimensione buffer
                if len(buffer) > 300:
                    buffer = buffer[-300:]
            else:
                sleep(0.005)
        except Exception as e:
            log(f"Errore monitor shutdown: {str(e)}", "ERROR")


# ========================
# UTILITIES
# ========================
def retry_on_error(func, *args, **kwargs):
    """Esegue una funzione con sistema di retry

    Args:
        func (callable): Funzione da eseguire

    Returns:
        Risultato della funzione

    Raises:
        SystemExit: Se shutdown_flag viene attivato
    """
    while not shutdown_flag.is_set():
        try:
            return func(*args, **kwargs)
        except Exception as e:
            log(f"Errore in {func.__name__}: {str(e)} - Riprovo...", "WARN")
            sleep(0.1)

    restart_program()


# ========================
# PROTOCOLLI COMUNICAZIONE
# ========================
def handshake_arduino():
    """Esegue handshake iniziale con Arduino"""
    log("Avvio procedura handshake...", "SYSTEM")

    while not shutdown_flag.is_set():
        try:
            with serial_lock:
                arduino.reset_input_buffer()
                arduino.write(b'1\n')
                arduino.flush()
                response = arduino.read_until(b'SYS|1\n').decode().strip()

            if response == "SYS|1":
                log("Handshake completato con successo", "SUCCESS")
                return
            elif response != "":
                log(f"Risposta inattesa dall'handshake: {response}", "WARN")
        except Exception as e:
            log(f"Errore durante handshake: {str(e)}", "ERROR")
            sleep(0.5)

    restart_program()


def wait_for_start():
    """Attende comando di start da Arduino"""
    log("In attesa comando START...", "SYSTEM")
    buffer = bytearray()
    start_time = time()

    while not shutdown_flag.is_set():
        try:
            # Lettura dati disponibili
            with serial_lock:
                data = arduino.read(arduino.in_waiting or 1)

            if data:
                buffer.extend(data)

                # Cerca comando start
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    if b"SYS|2" in line:
                        log("Ricevuto comando START", "SUCCESS")
                        return

                # Controllo overflow buffer
                if len(buffer) > 300:
                    buffer = buffer[-300:]
            else:
                sleep(0.005)

            # Controllo timeout
            if time() - start_time > TIMEOUT_START:
                log("Timeout attesa comando START", "ERROR")
                restart_program()
        except Exception as e:
            log(f"Errore durante l'attesa START: {str(e)}", "ERROR")
            restart_program()


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
"""
def identifica_colore_officina():
    Identifica il colore dell'officina disponibile

    Returns:
        str: Colore dell'officina (verde, giallo o sconosciuto)
    
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
"""
"""
def identifica_colore_auto():
    Identifica il colore dell'auto di fronte al robot

    Returns:
        str: Colore dell'auto (rosso, verde, giallo, blu o sconosciuto)
    
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
"""
# ========================
# LOGICA PRINCIPALE
# ========================
def main_execution():
    """Funzione principale di esecuzione"""
    log("Avvio modalità operativa", "SYSTEM")

    # Configura timer sicurezza
    safety_timer = threading.Timer(TEMPO_TIMER, termina_programma)
    safety_timer.daemon = True
    safety_timer.start()

    # Avvio thread sicurezza
    safety_thread = threading.Thread(target=check_shutdown)
    safety_thread.daemon = True
    safety_thread.start()

    try:
        robot.gira_sinistra(90, 50)
        #robot.gira_sinistra(90, 50)
        #robot.gira_destra(40, 40)
        servo.set_angle(180)
        sleep(3)
        servo.set_angle(0)
        gabbia.run_for_degrees(90, speed=-gabbia.default_speed)
    except KeyboardInterrupt:
        log("Avvio shutdown da tastiera...", "SYSTEM")
        shutdown_flag.set()
        try:
            with serial_lock:
                arduino.write(b'3')  # Comando spegnimento Arduino
        except Exception as e:
            log(f"Errore invio comando shutdown: {str(e)}", "WARN")
    finally:
        safety_timer.cancel()
        safety_thread.join()


# ========================
# ENTRYPOINT
# ========================
def main():
    """Punto d'ingresso principale"""
    signal.signal(signal.SIGINT, lambda s, f: shutdown_flag.set())

    try:
        handshake_arduino()
        wait_for_start()
        #main_execution()
    except Exception as e:
        log(f"Errore critico: {str(e)}", "ERROR")
    finally:
        shutdown_flag.set()
        shutdown_flag.wait()

        with serial_lock:
            try:
                if arduino.is_open:
                    arduino.close()
            except Exception as e:
                log(f"Errore chiusura seriale: {str(e)}", "WARN")

        log("Applicazione terminata", "SYSTEM")


if __name__ == "__main__":
    main()