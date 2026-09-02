#!/usr/bin/env python3
"""
ACC - utrzymywanie dystansu 30 cm (regulator P, w milimetrach)
Raspberry Pi 4B + AQMH2407ND + kamera Creative Live! Cam Sync 1080p V2

Zasada:
  - kamera mierzy szerokosc 'w' kolorowej tasmy w pikselach,
  - dystans_mm = DIST_CONST / w   (kalibracja: 100 mm -> 132 px => 13200),
  - regulator P trzyma zadany dystans TARGET_MM,
  - za daleko -> jedzie; dystans OK lub za blisko -> stoi; brak celu -> stoi.

Sterowanie: 'q' w oknie konczy.
!!! Pierwsze testy z podniesionymi kolami.
"""

import numpy as np
import cv2
from gpiozero import Motor

# ======================= SILNIKI =======================
left  = Motor(forward=6,  backward=5,  enable=12, pwm=True)   # kanal A = lewa
right = Motor(forward=26, backward=16, enable=13, pwm=True)   # kanal B = prawa

# ======================= KAMERA ========================
CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# ===================== WYKRYWANIE ======================
lower = np.array([20,  60,  80])     # zolto-zielona tasma (poluzowane pod swiatlo)
upper = np.array([45, 255, 255])

MIN_AREA = 80
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

def get_roi_bounds(frame, roi_w=260, roi_h=180):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    return (cy - roi_h // 2, cy + roi_h // 2,
            cx - roi_w // 2, cx + roi_w // 2)

# ================== ODLEGLOSC (mm) =====================
DIST_CONST = 13200.0     # z kalibracji: 100 mm * 132 px

# ==================== REGULATOR P ======================
TARGET_MM = 200.0     # docelowy dystans = 30 cm
KP        = 0.0012    # wzmocnienie (dostroic)
TOLERANCE = 20.0      # mm - martwa strefa wokol celu (stoi, nie drga)
MAX_SPEED = 0.50      # gorny limit predkosci
MIN_MOVE  = 0.15      # minimalna predkosc, przy ktorej kola ruszaja

def drive(raw):
    """Skaluje zadanie regulatora na (MIN_MOVE..MAX_SPEED) i jedzie. Zwraca realna predkosc."""
    raw = max(0.0, min(MAX_SPEED, raw))
    speed = 0.0
    if raw > 0:
        speed = MIN_MOVE + (MAX_SPEED - MIN_MOVE) * (raw / MAX_SPEED)
    left.forward(speed)
    right.forward(speed)
    return speed

def stop():
    left.stop()
    right.stop()

# ======================== PETLA ========================
try:
    print(f"ACC start. Cel = {TARGET_MM:.0f} mm. 'q' konczy, Ctrl+C awaryjnie.")
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

        w_px = 0
        if contours:
            biggest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(biggest)
            if w * h > MIN_AREA:
                w_px = w
                cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # --- REGULATOR P (w mm) ---
        if w_px == 0:
            stop()
            speed, dist = 0.0, 0.0
            status = "BRAK CELU -> STOP"
        else:
            dist = DIST_CONST / w_px             # aktualny dystans w mm
            error = dist - TARGET_MM             # >0: za daleko, <0: za blisko
            if abs(error) < TOLERANCE:
                stop()
                speed = 0.0
                status = "DYSTANS OK -> STOP"
            elif error > 0:
                speed = drive(KP * error)        # za daleko -> jedz
                status = "JADE"
            else:
                stop()                           # za blisko -> stoj
                speed = 0.0
                status = "ZA BLISKO -> STOP"

        print(f"w={w_px:3d}px  dystans={dist:6.0f}mm  v={speed:.2f}  {status}")

        #cv2.imshow("ACC (q = koniec)", roi)
        #cv2.imshow("Maska", mask)
        #if (cv2.waitKey(1) & 0xFF) == ord("q"):
            #break

except KeyboardInterrupt:
    print("\nPrzerwano.")
finally:
    stop()
    cap.release()
    cv2.destroyAllWindows()
    print("Zatrzymano, kamera zwolniona.")