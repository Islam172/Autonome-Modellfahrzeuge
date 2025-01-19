import cv2 as cv
import math
import numpy as np
import serial
import time

import FD_teensyComm2_core

def get_angle(corners):
    '''
        Berechnet aus den Eckpunkten des Aruco-Markers sein Heading

        corners: Array mit den vier Eckpunkten

        Gibt zurück:    Heading, 0°...360°
    '''

    # Mittelpunkt der oberen Seite des Markers
    a = (corners[0] + corners[1]) / 2
    # Mittelpunkt des markers
    m = (corners[1] + corners[3]) / 2
    # Richtungsvektor
    r = a - m

    angle = math.asin(r[1] / (math.sqrt(r[0]**2 + r[1]**2)))

    # rad -> deg
    angle = angle / (2*math.pi / 360)
    
    # Winkel von [-90°, +90°] zu [0°, 360°] umwandeln
    # 90° - 180°
    if r[0] > 0 and r[1] > 0:
        angle = 180 - angle
    # 180° - 270°
    elif r[0] > 0 and r[1] <= 0:
        angle = 180 - angle
    # 270° - 360°
    elif r[0] <= 0 and r[1] <= 0:
        angle = 360 + angle
    
    #print(f"corners: {corners}\na: {a}\nMittelpunkt: {m}\nRichtungsvektor: {r}\nWinkel: {angle}")

    return angle

def open_serial(port: int, br: int) -> serial.Serial:
    """
        Erstellt ein Serial-Objekt, mit dem über Serial Daten gesendet werden können.

        port:   Port über den kommuniziert werden soll
        br:     Baudrate
    """

    try:
        ser = serial.Serial(port)
        ser.baudrate = br
        ser.bytesize = 8
        ser.parity = 'N'
        ser.stopbits = 1
    except serial.SerialException:
        print("\nERROR:\tSerial-Objekt konnte nicht erstellt werden.")
        exit(1)

    return ser

def send_data(angle_minutes: int, ser: serial.Serial) -> None:
    '''
        Sendet den Winkel an über Serial.

        angle_minutes:  zu versendender Winkel in Minuten
        ser:            Serial-Objekt über das versendet werden soll
    '''

    # Liste mit Bytes der zu versendenden Daten
    data_list = list()
    data_list.append(angle_minutes >> 8)
    data_list.append(angle_minutes & 255)
    # Nachricht gemäß Protokoll aufbauen und per Serial verschicken
    msg = FD_teensyComm2_core.composeMsg(0, 1, data_list)
    ser.write(msg)


'''
    -------------------------------------------
    Lageerkennung von Aruco-Markern über Webcam
    -------------------------------------------    
    Erkennt Heading von Aruco-Markern, schickt den Winkel über Serial an einen LoRa-Sender und schreibt sie in eine Log-Datei.
    
    benötigte Bibliotheken -> siehe requirements.txt, "pip install -r requirements.txt" lädt benötigte Bibliotheken herunter
'''
if __name__ == "__main__":
    # Serial-Objekt erstellen
    port = input("Port?\t")
    br = input("Baudrate?\t")
    # br = 115200
    ser = open_serial(port, br)

    # Öffnet eine Webcam. Funktioniert auf meinem PC, für andere PCs muss cam_path ggf. abgeändert werden.
    cam_path = "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_CD90D4BF-video-index0"
    cam = cv.VideoCapture(cam_path)
    if not cam.isOpened:
        print("Kamera konnte nicht geöffnet werden")
        exit(1)

    #aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)    # Markerformat auf den Autos
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)     # zum Testen genutztes Markerformat
    detector = cv.aruco.ArucoDetector(aruco_dict, cv.aruco.DetectorParameters())

    # Datei, in die die Daten geschrieben werden
    file_path = f"cam_log_mit_filter_drehung{int(time.time())}.csv"
    log_file = open(file_path, 'w')

    i = 500 # nur jeder i-te Wert kommt in die Log-Datei
    j = 0

    marker_id = 3

    angle_deg: float = 0

    # wird im Log von Zeit abgezogen, um Anfangszeitpunkt auf 0 zu setzen
    start_time = time.time()
    # Spaltennamen
    log_file.write("Zeit,Winkel\n")

    while 1:  
        result, image = cam.read()

        if result:
            image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

            (corners, ids, rejected) = detector.detectMarkers(image)

            if ids is None:
                print("Kein Marker erkannt")
                continue

            print(f"Erkannte Marker: {ids}")

            # gibt Winkel aller detektierten Codes aus
            for i in range(0,len(ids)):
                if ids[i] != marker_id:
                    print("falsche ID")
                    continue

                angle_deg = get_angle(corners[i][0])
                print(f"Winkel des Markers {ids[i]}: {angle_deg}")

                '''
                    Umrechnung deg -> Minuten. Ermöglicht es, uint16 anstatt float an den Pico zu senden
                    und dabei trotzdem noch eine ausreichende Genauigkeit zu erhalten.
                '''
                angle_minutes = int(angle_deg * 60)
                send_data(angle_minutes, ser)

                # Schreibt nur jeden i-ten Winkelwert in die Log-Datei, damit sie bei langen Messungen nicht zu riesig wird.
                # if j >= i:
                #     log_file.write(f"{time.time() - start_time},{angle_deg}\n")
                #     j = 0
                # j += 1

        #time.sleep(1)
