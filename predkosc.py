#!/usr/bin/env python3
"""
TEST PRZYSPIESZANIA - predkosc vs moc (PWM)
Robot zwieksza moc od 10% do 100% i mierzy predkosc na kazdym poziomie.

Kolo: srednica 67 mm -> obwod ~0.2105 m
Zatrzymanie: DOWOLNY KLAWISZ (bez Entera).

!!! Najlepiej z kolami W POWIETRZU - przy 100% na podlodze robot ucieknie.
"""

import sys
import math
import termios
import tty
import select
from time import sleep, monotonic
from gpiozero import RotaryEncoder, Motor

# ===================== KOLO =====================
WHEEL_DIAMETER_M = 0.067
CIRCUMFERENCE_M  = math.pi * WHEEL_DIAMETER_M      # ~0.2105 m

# ============= KALIBRACJA ENKODERA ==============
COUNTS_PER_WHEEL_REV = 223         # skalibrowane (srednia z 219/225/224)

# ================== ENKODERY ====================
enc_left  = RotaryEncoder(17, 27, max_steps=0)
enc_right = RotaryEncoder(23, 22, max_steps=0)

# ================== SILNIKI =====================
left  = Motor(forward=6,  backward=5,  enable=12, pwm=True)
right = Motor(forward=26, backward=16, enable=13, pwm=True)

# =================== USTAWIENIA =================
POWER_START = 0.10     # 10%
POWER_STEP  = 0.10     # co 10%
SETTLE      = 1.5      # s - czas na ustabilizowanie predkosci po zmianie mocy
MEASURE     = 1.0      # s - okno pomiaru predkosci


def key_pressed():
    """True, jesli wcisnieto dowolny klawisz (bez blokowania)."""
    return select.select([sys.stdin], [], [], 0)[0] != []


def measure_speed(dt_measure):
    """Mierzy srednia predkosc obu kol w oknie czasowym [m/s]."""
    l0, r0 = enc_left.steps, enc_right.steps
    t0 = monotonic()
    sleep(dt_measure)
    dt = monotonic() - t0
    dl = abs(enc_left.steps - l0)
    dr = abs(enc_right.steps - r0)
    v_l = (dl / COUNTS_PER_WHEEL_REV) * CIRCUMFERENCE_M / dt
    v_r = (dr / COUNTS_PER_WHEEL_REV) * CIRCUMFERENCE_M / dt
    return (v_l + v_r) / 2.0


def stop():
    left.stop()
    right.stop()


def wait_settle(seconds):
    """Czeka podany czas, ale przerywa, jesli wcisnieto klawisz."""
    t_end = monotonic() + seconds
    while monotonic() < t_end:
        if key_pressed():
            raise KeyboardInterrupt
        sleep(0.05)


def main():
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        print("Test przyspieszania 10% -> 100%. DOWOLNY KLAWISZ = STOP.")
        print("-" * 40)
        power = POWER_START
        while power <= 1.0001:
            p = min(power, 1.0)
            left.forward(p)
            right.forward(p)

            wait_settle(SETTLE)          # daj sie rozpedzic
            v = measure_speed(MEASURE)   # zmierz predkosc

            print(f"moc={p*100:3.0f}%   v={v:4.2f} m/s   ({v*3.6:4.1f} km/h)")
            power += POWER_STEP

        print("-" * 40)
        print("Koniec testu.")
    except KeyboardInterrupt:
        print("\nZatrzymano klawiszem.")
    finally:
        stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print("Silniki zatrzymane.")


if __name__ == "__main__":
    main()