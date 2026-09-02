#!/usr/bin/env python3
"""
ACC - utrzymywanie dystansu (regulator P, bez skrecania)
Raspberry Pi 4B + AQMH2407ND + kamera USB

Zasada:
  - kamera wykrywa kolorowa opaske na pojezdzie z przodu,
  - "pole" opaski w kadrze = miara odleglosci (duze pole = blisko),
  - regulator P: im dalej poprzednik (male pole), tym szybciej jedziemy,
  - gdy dystans docelowy osiagniety -> stop,
  - gdy opaski nie widac -> stop (bezpieczenstwo).

Sterowanie: 'q' w oknie konczy program.
!!! Pierwsze testy z podniesionymi kolami albo w bezpiecznym miejscu.
"""

import numpy as np
import cv2
from gpiozero import Motor

# ======================= SILNIKI =======================
# Piny BCM wg ustalonej mapy (kanal A = lewa, B = prawa)
left  = Motor(forward=5,  backward=6,  enable=12, pwm=True)   # IN1, IN2, ENA
right = Motor(forward=16, backward=26, enable=13, pwm=True)   # IN3, IN4, ENB

# ======================= KAMERA ========================
CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# ===================== WYKRYWANIE ======================
# zakres koloru opaski (zolto-zielony) - dostrojony pod odczyty HSV ~[28,145,165]
lower = np.array([20,  60,  80])
upper = np.array([45, 255, 255])

MIN_AREA = 60     # ponizej tego uznajemy, ze opaski nie widac
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# ROI liczony wzgledem rozmiaru klatki (odporny na zmiane rozdzielczosci)
def get_roi_bounds(frame, roi_w=160, roi_h=180):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    return (cy - roi_h // 2, cy + roi_h // 2,
            cx - roi_w // 2, cx + roi_w // 2)

# ==================== REGULATOR P ======================
# TARGET_AREA: docelowe "pole" opaski = zadany dystans.
#   Ustaw poprzednika w wymaganej odleglosci, odczytaj wypisywane 'pole'
#   i wpisz te wartosc tutaj.
TARGET_AREA = 1000

KP        = 0.0005  # wzmocnienie regulatora (dostroic)
TOLERANCE = 300       # martwa strefa wokol celu -> stoi, nie drga
MAX_SPEED = 0.50      # gorny limit predkosci (0..1)
MIN_MOVE  = 0.15      # ponizej tej wartosci silniki nie ruszaja - podbijamy do niej

def drive(speed):
    speed = max(0.0, min(MAX_SPEED, speed))
    if speed > 0:
        speed = MIN_MOVE + (MAX_SPEED - MIN_MOVE) * (speed / MAX_SPEED)
    left.forward(speed)
    right.forward(speed)
    return speed          # <-- zwraca rzeczywista predkosc

def stop():
    left.stop()
    right.stop()

# ======================== PETLA ========================
try:
    print("ACC start. 'q' konczy. Ctrl+C awaryjnie.")
    while True:
        ok, frame = cap.read()
        if not ok:
            stop()
            break

        Y1, Y2, X1, X2 = get_roi_bounds(frame)
        roi = frame[Y1:Y2, X1:X2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        area = 0
        if contours:
            biggest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(biggest)
            if w * h > MIN_AREA:
                area = w * h
                cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # --- REGULATOR P ---
        if area == 0:
            # nie widac opaski -> zatrzymaj (bezpieczenstwo)
            stop()
            speed = 0.0
            status = "BRAK CELU -> STOP"
        else:
            error = TARGET_AREA - area      # >0: za daleko (jedz), <0: za blisko
            if abs(error) < TOLERANCE:
                stop()
                speed = 0.0
                status = "DYSTANS OK -> STOP"
            elif error > 0:
                speed = drive(KP * error)         # daleko -> jedz do przodu
                drive(speed)
                status = "JADE"
            else:
                stop()                      # za blisko -> stoj
                speed = 0.0
                status = "ZA BLISKO -> STOP"

        print(f"pole={area:5d}  predkosc={speed:.2f}  {status}")

        cv2.imshow("ACC (q = koniec)", roi)
        cv2.imshow("Maska", mask)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

except KeyboardInterrupt:
    print("\nPrzerwano.")
finally:
    stop()
    cap.release()
    cv2.destroyAllWindows()
    print("Zatrzymano, kamera zwolniona.")