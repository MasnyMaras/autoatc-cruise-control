#!/usr/bin/env python3
"""
POMIAR PREDKOSCI ROBOTA z enkoderow
Kolo: srednica 67 mm -> obwod ~0.2105 m

DWA TRYBY:
  python3 predkosc.py kalibracja   -> wyznacza impulsy na obrot kola
  python3 predkosc.py              -> jedzie i mierzy predkosc (m/s, km/h)

Kolejnosc:
  1) najpierw uruchom tryb 'kalibracja' i wpisz wynik do COUNTS_PER_WHEEL_REV,
  2) potem uruchamiaj bez argumentu, zeby mierzyc predkosc.

!!! Tryb pomiaru URUCHAMIA SILNIKI. Postaw robota bezpiecznie.
"""

import sys
import math
from time import sleep, monotonic
from gpiozero import RotaryEncoder, Motor

# ===================== KOLO =====================
WHEEL_DIAMETER_M = 0.067
CIRCUMFERENCE_M  = math.pi * WHEEL_DIAMETER_M      # ~0.2105 m

# ============= KALIBRACJA ENKODERA ==============
# Ile "krokow" enkodera na 1 pelny obrot KOLA.
# Wyznacz w trybie 'kalibracja' i wpisz tutaj.
COUNTS_PER_WHEEL_REV = 1496        # <-- DO WYZNACZENIA

# ================== ENKODERY ====================
# piny BCM (konwencja z ACC): lewy A=17,B=27 ; prawy z zamiana B,A dla znaku
enc_left  = RotaryEncoder(17, 27, max_steps=0)
enc_right = RotaryEncoder(23, 22, max_steps=0)

# ================== SILNIKI =====================
left  = Motor(forward=6,  backward=5,  enable=12, pwm=True)   # po korekcie kierunku
right = Motor(forward=26, backward=16, enable=13, pwm=True)

TEST_PWM = 0.40      # z jaka moca jechac podczas pomiaru
SAMPLE_DT = 0.5      # co ile sekund liczymy predkosc


def kalibracja():
    """Wyznacza impulsy na obrot kola - recznie."""
    print("\n=== KALIBRACJA impulsow na obrot kola ===")
    print("Zaznacz punkt na LEWYM kole (np. mazakiem).")
    input("Ustaw kolo w pozycji startowej i nacisnij Enter...")
    start = enc_left.steps
    input("Obroc kolo DOKLADNIE o 1 pelny obrot (360 st) i nacisnij Enter...")
    end = enc_left.steps
    counts = abs(end - start)
    print("\n" + "=" * 45)
    print(f"  Impulsow na obrot kola = {counts}")
    print(f"  Wpisz:  COUNTS_PER_WHEEL_REV = {counts}")
    print("=" * 45 + "\n")
    print("Wskazowka: powtorz 2-3 razy, wynik powinien byc podobny.")


def pomiar():
    """Jedzie z TEST_PWM i liczy predkosc z enkoderow."""
    print(f"\n=== POMIAR PREDKOSCI (PWM={TEST_PWM}) ===")
    print("Ctrl+C konczy i zatrzymuje silniki.\n")

    left.forward(TEST_PWM)
    right.forward(TEST_PWM)

    prev_l = enc_left.steps
    prev_r = enc_right.steps
    t_prev = monotonic()

    try:
        while True:
            sleep(SAMPLE_DT)
            now = monotonic()
            dt = now - t_prev

            l = enc_left.steps
            r = enc_right.steps
            dl = abs(l - prev_l)
            dr = abs(r - prev_r)

            # obroty kola -> metry -> m/s
            v_l = (dl / COUNTS_PER_WHEEL_REV) * CIRCUMFERENCE_M / dt
            v_r = (dr / COUNTS_PER_WHEEL_REV) * CIRCUMFERENCE_M / dt
            v = (v_l + v_r) / 2.0

            print(f"lewy={v_l:4.2f}  prawy={v_r:4.2f}  "
                  f"SREDNIA={v:4.2f} m/s  ({v*3.6:4.1f} km/h)")

            prev_l, prev_r, t_prev = l, r, now
    finally:
        left.stop()
        right.stop()
        print("Silniki zatrzymane.")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "kalibracja":
        kalibracja()
    else:
        try:
            pomiar()
        except KeyboardInterrupt:
            left.stop()
            right.stop()
            print("\nPrzerwano.")