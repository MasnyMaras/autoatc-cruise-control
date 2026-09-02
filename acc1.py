#!/usr/bin/env python3
"""
ACC - utrzymywanie dystansu 30 cm, z pomiarem predkosci z enkoderow
Raspberry Pi 4B + AQMH2407ND + kamera Creative Live! Cam Sync 1080p V2

Integracja pomiarow:
  - dystans:  dystans_mm = 13200 / w   (kalibracja 100 mm -> 132 px)
  - predkosc: regulator zadaje predkosc w m/s (nie surowe PWM),
              wykorzystujac zmierzona charakterystyke (max ~0.80 m/s),
  - enkodery: licza RZECZYWISTA predkosc robota (do podgladu/weryfikacji).

Sterowanie: 'q' w oknie konczy (gdy SHOW_PREVIEW=True).
!!! Pierwsze testy z podniesionymi kolami.
"""

import numpy as np
import cv2
import math
from time import monotonic
from gpiozero import Motor, RotaryEncoder

SHOW_PREVIEW = True      # False do autostartu (usluga systemd, bez ekranu)

# ======================= SILNIKI =======================
left  = Motor(forward=6,  backward=5,  enable=12, pwm=True)   # kanal A = lewa
right = Motor(forward=26, backward=16, enable=13, pwm=True)   # kanal B = prawa

# ======================= ENKODERY ======================
enc_left  = RotaryEncoder(17, 27, max_steps=0)
enc_right = RotaryEncoder(23, 22, max_steps=0)

# ==================== KOLO / PREDKOSC ==================
COUNTS_PER_WHEEL_REV = 223                  # skalibrowane
WHEEL_DIAMETER_M     = 0.067
CIRCUMFERENCE_M      = math.pi * WHEEL_DIAMETER_M   # ~0.2105 m
V_MAX_MS             = 0.80                 # zmierzona predkosc max (100% mocy)

def steps_to_speed(dsteps, dt):
    """Przelicza przyrost impulsow na predkosc [m/s]."""
    if dt <= 0:
        return 0.0
    return (abs(dsteps) / COUNTS_PER_WHEEL_REV) * CIRCUMFERENCE_M / dt

# ======================= KAMERA ========================
CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# ===================== WYKRYWANIE ======================
lower = np.array([20,  60,  80])
upper = np.array([45, 255, 255])
MIN_AREA = 80
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

def get_roi_bounds(frame, roi_w=260, roi_h=180):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    return (cy - roi_h // 2, cy + roi_h // 2,
            cx - roi_w // 2, cx + roi_w // 2)

# ================== ODLEGLOSC (mm) =====================
DIST_CONST = 13200.0

# ==================== REGULATOR P ======================
TARGET_MM   = 300.0     # docelowy dystans = 30 cm
KP_V        = 0.002     # wzmocnienie: m/s na kazdy mm bledu (dostroic)
TOLERANCE   = 20.0      # mm - martwa strefa (stoi, nie drga)
MIN_MOVE_MS = 0.08      # m/s - najmniejsza predkosc, przy ktorej kola jada

def drive_speed(v_target):
    """Zadaje predkosc w m/s -> PWM wg zmierzonej charakterystyki. Zwraca (v_target, pwm)."""
    v_target = max(0.0, min(V_MAX_MS, v_target))
    if 0 < v_target < MIN_MOVE_MS:
        v_target = MIN_MOVE_MS
    pwm = v_target / V_MAX_MS               # liniowa charakterystyka moc->predkosc
    pwm = max(0.0, min(1.0, pwm))
    left.forward(pwm)
    right.forward(pwm)
    return v_target, pwm

def stop():
    left.stop()
    right.stop()

# ======================== PETLA ========================
prev_l = enc_left.steps
prev_r = enc_right.steps
t_prev = monotonic()

try:
    print(f"ACC start. Cel = {TARGET_MM:.0f} mm, v_max = {V_MAX_MS} m/s. 'q'/Ctrl+C konczy.")
    while True:
        ok, frame = cap.read()
        if not ok:
            stop()
            break

        # --- POMIAR RZECZYWISTEJ PREDKOSCI (enkodery) ---
        now = monotonic()
        dt = now - t_prev
        l, r = enc_left.steps, enc_right.steps
        v_real = (steps_to_speed(l - prev_l, dt) + steps_to_speed(r - prev_r, dt)) / 2.0
        prev_l, prev_r, t_prev = l, r, now

        # --- WYKRYWANIE OPASKI ---
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
                if SHOW_PREVIEW:
                    cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # --- REGULATOR P (dystans -> zadana predkosc m/s) ---
        if w_px == 0:
            stop()
            v_set, dist = 0.0, 0.0
            status = "BRAK CELU -> STOP"
        else:
            dist = DIST_CONST / w_px
            error = dist - TARGET_MM              # >0: za daleko, <0: za blisko
            if abs(error) < TOLERANCE:
                stop()
                v_set = 0.0
                status = "DYSTANS OK -> STOP"
            elif error > 0:
                v_set, _ = drive_speed(KP_V * error)
                status = "JADE"
            else:
                stop()
                v_set = 0.0
                status = "ZA BLISKO -> STOP"

        print(f"dystans={dist:6.0f}mm  v_zadana={v_set:4.2f}  "
              f"v_rzecz={v_real:4.2f} m/s  {status}")

        if SHOW_PREVIEW:
            cv2.imshow("ACC (q = koniec)", roi)
            cv2.imshow("Maska", mask)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break

except KeyboardInterrupt:
    print("\nPrzerwano.")
finally:
    stop()
    cap.release()
    if SHOW_PREVIEW:
        cv2.destroyAllWindows()
    print("Zatrzymano, kamera zwolniona.")