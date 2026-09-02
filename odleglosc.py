#!/usr/bin/env python3
"""
KALIBRACJA ODLEGLOSCI - pomiar na 10 cm (100 mm)
Ustaw tasme DOKLADNIE 100 mm od obiektywu kamery, skieruj ja rownolegle
(bok 55 mm poziomo) i uruchom ten skrypt.

Skrypt:
  - wykrywa kolorowa tasme (te same progi HSV co ACC),
  - mierzy szerokosc 'w' bounding boxa,
  - usrednia pomiar z wielu klatek (stabilniejszy wynik),
  - po nacisnieciu 'z' zapisuje srednia i wypisuje gotowa stala DIST_CONST.

Sterowanie:
  z  -> zapisz usredniony pomiar i policz stala
  q  -> wyjscie
"""

import numpy as np
import cv2

# --- ODLEGLOSC KALIBRACJI ---
D_KAL = 100.0        # mm (10 cm)

# --- KAMERA (te same ustawienia co w ACC!) ---
CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# --- PROGI KOLORU (takie same jak w ACC) ---
lower = np.array([20,  60,  80])
upper = np.array([45, 255, 255])

MIN_AREA = 80
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

def get_roi_bounds(frame, roi_w=260, roi_h=180):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    return (cy - roi_h // 2, cy + roi_h // 2,
            cx - roi_w // 2, cx + roi_w // 2)

# srednia krocząca z ostatnich pomiarow
samples = []
MAX_SAMPLES = 30     # ile ostatnich klatek usredniamy

print("Ustaw tasme 100 mm od kamery. 'z' = zapisz pomiar, 'q' = wyjscie.")

try:
    while True:
        ok, frame = cap.read()
        if not ok:
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

                samples.append(w_px)
                if len(samples) > MAX_SAMPLES:
                    samples.pop(0)

        avg = sum(samples) / len(samples) if samples else 0
        print(f"w={w_px:3d} px   srednia({len(samples)})={avg:6.1f} px")

        cv2.imshow("Kalibracja 10cm (z=zapisz, q=wyjscie)", roi)
        cv2.imshow("Maska", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("z"):
            if avg > 0:
                dist_const = D_KAL * avg
                print("\n" + "=" * 50)
                print(f"  Srednia szerokosc na {D_KAL:.0f} mm : {avg:.1f} px")
                print(f"  STALA DIST_CONST = {dist_const:.0f}")
                print(f"  Wzor w ACC:  dystans_mm = {dist_const:.0f} / w")
                print("=" * 50 + "\n")
            else:
                print("Brak wykrytej tasmy - nie zapisano.")
finally:
    cap.release()
    cv2.destroyAllWindows()