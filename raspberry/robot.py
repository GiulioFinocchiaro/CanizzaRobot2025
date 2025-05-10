from time import sleep
from buildhat import Motor, ColorSensor  # Aggiunto ColorSensor
from typing import Union, Optional
from colorama import Fore, Style, init as colorama_init
from datetime import datetime

colorama_init(autoreset=True)

COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED,
    "CRITICAL": Fore.RED + Style.BRIGHT,
    "ROBOT": Fore.MAGENTA,
    "CALIB": Fore.BLUE
}

def log(msg: str, level: str = "INFO"):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    print(f"{colore}[{level}] {ts}: {msg}{Style.RESET_ALL}")

class RobotError(Exception):
    def __init__(self, message: str):
        log(message, "ERROR")
        super().__init__(message)

class Robot:
    def __init__(self, left_port: str, right_port: str, color_sensor_port: Optional[str] = None, right_speed_factor: float = 1.05):
        try:
            log(f"Inizializzazione robot - Motori: {left_port}/{right_port}", "ROBOT")
            # Inizializzazione separata dei motori
            self.motor_sinistro = Motor(left_port)
            self.motor_destro = Motor(right_port)
            self._default_speed = 50
            self._is_moving = False
            # Fattore di correzione per il motore destro
            self._right_speed_factor = right_speed_factor
            
            # Inizializzazione sensore colore se specificato
            self.color_sensor = None
            if color_sensor_port:
                log(f"Inizializzazione sensore colore: {color_sensor_port}", "ROBOT")
                self.color_sensor = ColorSensor(color_sensor_port)
                self.color_sensor.get_color()  # Prima lettura per inizializzare
                
            log(f"Robot inizializzato con successo (fattore bilanciamento: {right_speed_factor})", "ROBOT")
        except Exception as e:
            raise RobotError(f"Errore inizializzazione robot: {str(e)}") from e

    @property
    def default_speed(self) -> int:
        return self._default_speed

    @default_speed.setter
    def default_speed(self, value: int):
        if not 0 <= value <= 100:
            log(f"Velocità {value}% non valida", "ERROR")
            raise ValueError("La velocità deve essere tra 0 e 100")
        self._default_speed = value
        log(f"Velocità predefinita impostata a {value}%", "DEBUG")

    @property
    def right_speed_factor(self) -> float:
        return self._right_speed_factor
    
    @right_speed_factor.setter
    def right_speed_factor(self, value: float):
        if value <= 0:
            log(f"Fattore di bilanciamento {value} non valido", "ERROR")
            raise ValueError("Il fattore di bilanciamento deve essere maggiore di 0")
        self._right_speed_factor = value
        log(f"Fattore di bilanciamento impostato a {value}", "DEBUG")

    def muovi_avanti(self, speed: Union[int, None] = None):
        actual_speed = speed if speed is not None else self.default_speed
        if not 0 <= actual_speed <= 100:
            log(f"Velocità {actual_speed}% non valida", "ERROR")
            raise ValueError("La velocità dev'essere tra 0 e 100")
        
        # Calcola la velocità bilanciata per il motore destro
        speed_right = min(int(actual_speed * self._right_speed_factor), 100)
        
        log(f"Avvio movimento in avanti a {actual_speed}% (sin) / {speed_right}% (dx)", "ROBOT")
        
        # Attivazione alternata dei motori per un movimento più fluido
        self.motor_destro.start(-speed_right)
        sleep(0.05)  # Piccolo ritardo tra i motori
        self.motor_sinistro.start(-actual_speed)
        
        self._is_moving = True

    def muovi_indietro(self, speed: Union[int, None] = None):
        actual_speed = speed if speed is not None else self.default_speed
        
        # Calcola la velocità bilanciata per il motore destro
        speed_right = min(int(actual_speed * self._right_speed_factor), 100)
        
        log(f"Avvio movimento all'indietro a {actual_speed}% (sin) / {speed_right}% (dx)", "ROBOT")
        
        # Attivazione alternata dei motori per un movimento più fluido
        self.motor_destro.start(speed_right)
        sleep(0.05)  # Piccolo ritardo tra i motori
        self.motor_sinistro.start(actual_speed)
        
        self._is_moving = True

    def stop_movimento(self):
        log("Arresto movimento", "ROBOT")
        self.motor_sinistro.stop()
        self.motor_destro.stop()
        self._is_moving = False

    def muovi_avanti_for(self, unit: str, value: float, speed: Union[int, None] = None):
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Movimento in avanti di {value} {unit} a {actual_speed}%", "DEBUG")
        self._run_movement(value, -actual_speed, unit)

    def muovi_indietro_for(self, unit: str, value: float, speed: Union[int, None] = None):
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Movimento all'indietro di {value} {unit} a {actual_speed}%", "DEBUG")
        self._run_movement(value, actual_speed, unit)

    def gira_destra(self, degrees: int = 90, speed: Union[int, None] = None):
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Rotazione di {degrees}° a destra a {actual_speed}%", "ROBOT")
        self._turn(degrees, actual_speed, right=True)

    def gira_sinistra(self, degrees: int = 90, speed: Union[int, None] = None):
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Rotazione di {degrees}° a sinistra a {actual_speed}%", "ROBOT")
        self._turn(degrees, actual_speed, right=False)

    def _run_movement(self, value: float, speed: int, unit: str):
        valid_units = ['degrees', 'seconds', 'rotations']
        if unit not in valid_units:
            log(f"Unità {unit} non supportata", "ERROR")
            raise RobotError(f"Unità non valida: {unit}. Usare {valid_units}")

        try:
            speed_right = min(int(abs(speed) * self._right_speed_factor), 100)
            if speed < 0:
                speed_right = -speed_right
            
            log(f"Esecuzione movimento con velocità {speed}/{speed_right}", "DEBUG")
            
            # Gestione separata dei metodi dei motori
            method_name = f'run_for_{unit}'
            method_sinistro = getattr(self.motor_sinistro, method_name)
            method_destro = getattr(self.motor_destro, method_name)
            
            # Avvia il motore destro leggermente prima per compensare
            method_destro(value, speed_right)
            sleep(0.05)  # Piccolo ritardo
            method_sinistro(value, speed)
            
            self._is_moving = False
            log(f"Movimento completato: {value} {unit}", "DEBUG")
        except AttributeError:
            error_msg = f"Metodo {method_name} non disponibile"
            log(error_msg, "CRITICAL")
            raise RobotError(error_msg)
        except Exception as e:
            error_msg = f"Errore movimento: {str(e)}"
            log(error_msg, "ERROR")
            raise RobotError(error_msg) from e

    def _turn(self, degrees: int, speed: int, right: bool = True):
        try:
            log(f"Avvio rotazione: {degrees}°", "DEBUG")
            duration = degrees * 0.01  # Regolabile

            if right:
                # Ruota solo il motore destro
                self.motor_destro.start(speed)
                self.motor_sinistro.stop()
            else:
                # Ruota solo il motore sinistro
                self.motor_sinistro.start(speed)
                self.motor_destro.stop()

            sleep(duration)
            self.motor_destro.stop()
            self.motor_sinistro.stop()
            self._is_moving = False
            log("Rotazione completata", "DEBUG")
        except Exception as e:
            error_msg = f"Errore rotazione: {str(e)}"
            log(error_msg, "ERROR")
            raise RobotError(error_msg) from e

    @property
    def is_moving(self) -> bool:
        return self._is_moving

    def __enter__(self):
        log("Avvio contesto robot", "DEBUG")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        log("Uscita contesto robot", "DEBUG")
        self.stop_movimento()

    def calibra_motori(self):
        log("Avvio procedura di calibrazione", "CALIB")
        try:
            for i in range(2):
                log(f"Ciclo calibrazione {i + 1}/2", "CALIB")
                self.muovi_avanti_for('seconds', 1, speed=30)
                self.muovi_indietro_for('seconds', 1, speed=30)
                self.gira_destra(90, speed=30)
                self.gira_sinistra(90, speed=30)
            log("Calibrazione completata con successo", "CALIB")
        except Exception as e:
            error_msg = f"Calibrazione fallita: {str(e)}"
            log(error_msg, "CRITICAL")
            raise RobotError(error_msg) from e

    def aggiusta_bilanciamento(self, nuovo_fattore: float = None):
        """
        Procedura per regolare il fattore di bilanciamento dei motori
        """
        if nuovo_fattore is not None:
            self.right_speed_factor = nuovo_fattore
            log(f"Fattore di bilanciamento aggiornato a {nuovo_fattore}", "CALIB")
            return

        # Procedura interattiva per trovare il fattore di bilanciamento ottimale
        log("Avvio procedura di bilanciamento interattiva", "CALIB")
        current_factor = self.right_speed_factor
        
        # Test con diversi fattori
        for factor in [current_factor - 0.05, current_factor, current_factor + 0.05]:
            self.right_speed_factor = factor
            log(f"Test con fattore {factor}", "CALIB")
            self.muovi_avanti_for('seconds', 2, speed=40)
            sleep(1)
            
        self.right_speed_factor = current_factor
        log("Procedura completata. Selezionare il fattore più appropriato", "CALIB")
        
    def rileva_colore(self) -> Optional[str]:
        """
        Rileva il colore attuale dal sensore
        
        Returns:
            Optional[str]: Colore rilevato o None se sensore non disponibile
        """
        if not self.color_sensor:
            log("Sensore colore non disponibile", "ERROR")
            return None
            
        try:
            color = self.color_sensor.get_color()
            log(f"Colore rilevato: {color}", "DEBUG")
            return color
        except Exception as e:
            log(f"Errore lettura colore: {str(e)}", "ERROR")
            return None
    
    def cerca_blocco_rosso(self, max_tentativi: int = 10, ampiezza_ricerca: int = 30, 
                          durata_movimento: float = 0.5, distanza_avanzamento: float = 10.0):
        """
        Cerca ricorsivamente un blocco rosso con movimenti a zigzag
        
        Args:
            max_tentativi: Numero massimo di tentativi prima di abbandonare
            ampiezza_ricerca: Ampiezza di rotazione in gradi per ogni direzione
            durata_movimento: Durata in secondi di ogni movimento di ricerca
            distanza_avanzamento: Distanza in centimetri da avanzare ad ogni ciclo (i blocchi sono a circa 10cm)
        
        Returns:
            bool: True se trova un blocco rosso, False altrimenti
        """
        if not self.color_sensor:
            log("Impossibile cercare blocchi: sensore colore non disponibile", "ERROR")
            return False
            
        log("Avvio ricerca blocco rosso", "ROBOT")
        
        # Funzione ricorsiva di ricerca
        def esegui_ricerca(tentativi_rimasti: int, direzione_destra: bool = True) -> bool:
            if tentativi_rimasti <= 0:
                log("Ricerca abbandonata: raggiunto numero massimo di tentativi", "WARN")
                return False
                
            # Controlla se vede già rosso
            color = self.rileva_colore()
            if color == "red":
                log("Blocco rosso trovato!", "ROBOT")
                return True
                
            # Esegue rotazione alternata
            log(f"Tentativo {max_tentativi - tentativi_rimasti + 1}/{max_tentativi}: " + 
                f"rotazione {'destra' if direzione_destra else 'sinistra'}", "DEBUG")
                
            if direzione_destra:
                self.gira_destra(ampiezza_ricerca, speed=30)
            else:
                self.gira_sinistra(ampiezza_ricerca, speed=30)
            
            # Breve pausa per stabilizzare il sensore
            sleep(0.2)
            
            # Controlla di nuovo dopo la rotazione
            color = self.rileva_colore()
            if color == "red":
                log("Blocco rosso trovato!", "ROBOT")
                return True
                
            # Se non trova, inverte direzione e riprova
            return esegui_ricerca(tentativi_rimasti - 1, not direzione_destra)
        
        # Eseguire la ricerca iniziale
        trovato = esegui_ricerca(max_tentativi)
        
        # Se non trova nulla, avanza e riprova
        if not trovato:
            log(f"Nessun blocco rosso trovato nelle vicinanze, avanzo di {distanza_avanzamento} cm", "ROBOT")
            # Calcolo approssimativo di rotazioni dai cm (dipende dalle dimensioni delle ruote)
            rotazioni = distanza_avanzamento / 17.0  # Assumendo circonferenza ruota ~ 17cm
            self.muovi_avanti_for('rotations', rotazioni, speed=30)
            
            # Ricorsione per provare di nuovo
            return self.cerca_blocco_rosso(max_tentativi, ampiezza_ricerca, durata_movimento, distanza_avanzamento)
        
        return trovato
        
    def prendi_blocco_rosso(self):
        """
        Cerca e prende un blocco rosso
        
        Returns:
            bool: True se il blocco è stato trovato e preso, False altrimenti
        """
        log("Avvio procedura recupero blocco rosso", "ROBOT")
        
        # Prima cerca il blocco rosso
        if not self.cerca_blocco_rosso():
            log("Impossibile trovare un blocco rosso", "ERROR")
            return False
            
        # Una volta trovato, avvicina il robot al blocco
        log("Avvicinamento al blocco rosso", "ROBOT")
        
        # Avanza lentamente fino a quando il blocco è a portata
        self.muovi_avanti_for('seconds', 1.5, speed=20)
        
        # Qui in un robot reale si attiverebbe un meccanismo di presa
        log("Blocco rosso afferrato", "ROBOT")
        
        return True