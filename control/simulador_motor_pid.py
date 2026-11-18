"""MVP: simulación del PID de velocidad de una rueda del robot VSS (lazo cerrado).

El controlador es un port línea por línea de PID::compute / PID::reset
(pid.cpp), así que lo que probamos aquí es literal el mismo algoritmo que corre
en el Arduino cada 25 ms. La planta es un motor de primer orden con cosas
opcionales para hacerlo más real: zona muerta, fricción de Coulomb, ruido y
cuantización del encoder.

Output: tabla + métricas en consola y respuesta_pid.png al lado de este archivo.
"""

import math
import os

import matplotlib

matplotlib.use("Agg")  # solo guardamos el PNG, no abre ventana
import matplotlib.pyplot as plt
import numpy as np

# Parámetros del sistema (lo básico)
SAMPLE_TIME_MS = 25      # cada cuánto corre el lazo en el firmware
MAX_PWM = 255.0          # tope de la salida del PID (max_output)

# --- Datos del motor real (hoja de datos) ---------------------------------
# E-S Motor 1220WG-N20VA-42-EN 6VB45 ("Micro DC Worm Gear Motor w/ Encoder
# - 6V 530RPM"). Los valores de abajo son los del fabricante; lo marcado como
# SUPUESTO hay que confirmarlo con los mentores o medirlo.
MOTOR_RATED_V = 6.0          # voltaje nominal del motor (V)
MOTOR_NO_LOAD_RPM = 530.0    # RPM sin carga en la rueda, a MOTOR_RATED_V
GEAR_RATIO = 42              # reducción de la caja (1:42)
ENCODER_PPR_MOTOR = 7        # pulsos por vuelta del EJE DEL MOTOR (antes de la caja)
SUPPLY_V = 6.0               # SUPUESTO: voltaje que recibe el motor con PWM = 255
ENCODER_EDGES = 1            # SUPUESTO: 1 = un canal, flanco de subida;
                             #           2 = un canal, ambos flancos; 4 = cuadratura

# --- Paso 1: K (ganancia de la planta) ------------------------------------
# Velocidad máxima de la rueda en RPS, corregida por el voltaje real:
#   RPS_max = (RPM sin carga / 60) * (V_supply / V_nominal) = 530/60 * 1 = 8.83
# K = cuántas RPS da cada unidad de PWM:
#   K = RPS_max / 255 = 8.83 / 255 = 0.0346
MAX_WHEEL_RPS = MOTOR_NO_LOAD_RPM / 60.0 * (SUPPLY_V / MOTOR_RATED_V)
PLANT_GAIN = MAX_WHEEL_RPS / MAX_PWM

# --- Paso 2: tau (qué tan "lento" es el motor) ----------------------------
# NO viene en la hoja de datos. 0.25 s es un SUPUESTO razonable para un N20
# con rueda y robot; pruebas_pid.py lo varía de 0.1 a 0.4 s. El valor real
# sale de la prueba de escalón (63 % de la velocidad final = tau).
PLANT_TAU = 0.25

# --- Paso 3: resolución del encoder en la rueda ---------------------------
#   pulsos por vuelta de la rueda = 7 * 42 * ENCODER_EDGES = 294 (con x1)
WHEEL_PULSES_PER_REV = ENCODER_PPR_MOTOR * GEAR_RATIO * ENCODER_EDGES

SETTLING_BAND = 0.02     # banda de establecimiento: ±2 % del escalón
STEADY_WINDOW_S = 0.5    # ventana del final que usamos para el error en estado estable
REQ_SETTLING_S = 2.0     # lo que tiene que cumplir el escenario 1
REQ_OVERSHOOT_PCT = 10.0  # requisito del escenario 1

# --- Paso 4: Sintonía: PI con cancelación polo-cero (la parte de mate) ----

# Planta:       P(s) = K / (tau*s + 1)            K = 0.0346 RPS/PWM, tau = 0.25 s
# Controlador:  C(s) = Kp + Ki/s = Kp*(s + Ki/Kp)/s
#
# La idea: ponemos el cero del PI justo encima del polo de la planta
# (Ki/Kp = 1/tau) y se cancelan. Lo que queda es L(s) = Kp*K/(tau*s), y el lazo
# cerrado se vuelve de primer orden, tipo súper simple:
#
#     T(s) = 1 / (tau_cl*s + 1),   tau_cl = tau / (Kp*K)
#
# Un primer orden no tiene sobrepaso (no se pasa del target, cero drama);
# t_subida(10-90 %) = 2.2*tau_cl y t_establecimiento(2 %) = 3.9*tau_cl.
# Elegimos tau_cl = 0.25 s y sale:
#
#     Kp = tau / (K*tau_cl) = 0.25 / (0.0346*0.25) = 28.9
#     Ki = Kp / tau         = 28.9 / 0.25          = 115.5
#     Kd = 0   (con una planta de primer orden el D solo amplifica el ruido del encoder)
#
# Predicción: t_subida ~ 0.55 s, t_establecimiento ~ 1.0 s, sobrepaso 0 %.
# Como Kp*K = 1, el PWM ideal queda constante (target/K = 173 para 6 RPS) y no
# satura. Y no conviene hacerlo más rápido: el golpe inicial es Kp*target y ya
# satura si Kp > 255/6 = 42.5 (para 6 RPS); además cada pulso del encoder mueve
# el PWM Kp/(N*Ts). Más agresivo = más ruido en el PWM, no vale la pena.
# (Ojo: los números de este comentario son para los datos de arriba; si cambias
# SUPPLY_V o PLANT_TAU, el programa recalcula e imprime los valores nuevos.)
DESIRED_TAU_CL = 0.25


def tune_pi_pole_cancellation(gain, tau, tau_cl):
    """Te da (kp, ki, kd) del PI cuyo cero cancela el polo de K/(tau*s+1).

    kp = tau/(gain*tau_cl), ki = kp/tau, kd = 0. tau_cl es la constante de
    tiempo que quieres en lazo cerrado (modelo continuo, no corrige por el
    muestreo de 25 ms).
    """
    kp = tau / (gain * tau_cl)
    return kp, kp / tau, 0.0



# Controlador (port de pid.h / pid.cpp, tal cual)
PID_ACCEPT_POSITIVES_ONLY = True
PID_ACCEPT_NEGATIVES_ONLY = False


def _clamp(v, lo, hi):
    """Igual que el clamp de pid.cpp, mismo orden de comparaciones."""
    return lo if v < lo else (hi if hi < v else v)


class PIDController:
    """Port de PID::PidParameters + PID::compute + PID::reset (pid.cpp).

    Los atributos que se llaman igual que en el firmware (kp, ki, kd, max_output,
    min_output, error_threshold, integral_error, previous_error,
    last_iteration_ms, first_run, target, error, output, one_direction_only,
    accept_type, reset_within_threshold) hacen lo mismo que en el struct de C++.

    Lo copié tal cual, con todo y sus efectos raros:
      * first_run: no integra; la salida es kp*error recortada a
        [-max_output, min_output] (sí, el límite de arriba es min_output, así
        está en pid.cpp; con min_output = 0 un error positivo sale como 0).
      * Integral: no acumula si |error| <= error_threshold, ni si la salida del
        ciclo ANTERIOR estaba saturada y el error empuja hacia ese mismo lado.
        Lo que ya se acumuló nunca se revierte.
      * La derivada va sobre el error.
      * min_output > 0: si |salida| < min_output, se fuerza a 0.
      * reset_within_threshold: si |error| < error_threshold en UN solo ciclo
        (no hay contador de ciclos), llama a reset(), que borra integral,
        previous_error, error, target y también output (ese ciclo sale 0), y
        deja first_run = True para el siguiente ciclo.

    Hay dos banderas que NO existen en el firmware. Vienen apagadas para no
    cambiar el algoritmo que estamos validando:
      * derivative_on_measurement: D = -kd * d(medición)/dt en vez de
        kd * d(error)/dt (así no hay patada derivativa cuando cambia el target).
      * fix_first_run_clamp: usa max_output como límite de arriba en first_run.

    p_term, i_term, d_term, was_reset y reset_count son solo para diagnóstico.
    """

    def __init__(self, kp, ki, kd, max_output=0.0, min_output=0.0,
                 error_threshold=0.0, one_direction_only=False,
                 accept_type=PID_ACCEPT_POSITIVES_ONLY,
                 reset_within_threshold=False,
                 derivative_on_measurement=False, fix_first_run_clamp=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.error_threshold = error_threshold
        self.one_direction_only = one_direction_only
        self.accept_type = accept_type
        self.reset_within_threshold = reset_within_threshold
        self.derivative_on_measurement = derivative_on_measurement
        self.fix_first_run_clamp = fix_first_run_clamp

        self.integral_error = 0.0
        self.previous_error = 0.0
        self.last_iteration_ms = 0
        self.first_run = True
        self.target = 0.0
        self.error = 0.0
        self.output = 0.0

        self._previous_measurement = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.was_reset = False
        self.reset_count = 0

    def reset(self):
        """Igual que PID::reset: deja el estado en cero y rearma first_run.

        No toca ganancias ni límites. Ojo que también borra target y output,
        igual que el firmware.
        """
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.output = 0.0
        self.error = 0.0
        self.target = 0.0
        self.last_iteration_ms = 0
        self.first_run = True

    def compute(self, current_time_ms):
        """Igual que PID::compute. Lee self.error (lo pone quien llama) y
        actualiza self.output. current_time_ms hace de millis().
        Regresa self.output."""
        self.was_reset = False
        measurement = self.target - self.error

        if self.first_run:
            self.last_iteration_ms = current_time_ms
            self.previous_error = self.error
            self._previous_measurement = measurement
            self.first_run = False

            initial_output = self.kp * self.error
            self.p_term, self.i_term, self.d_term = initial_output, 0.0, 0.0

            if self.min_output > 0.0 and abs(initial_output) < self.min_output:
                initial_output = 0.0

            if self.max_output > 0.0:
                # pid.cpp: clamp(initial_output, -pid.max_output, pid.min_output)
                upper = self.max_output if self.fix_first_run_clamp else self.min_output
                self.output = _clamp(initial_output, -self.max_output, upper)
            else:
                self.output = initial_output
            return self.output

        delta_time_ms = current_time_ms - self.last_iteration_ms
        if delta_time_ms == 0:
            return self.output
        delta_time_s = delta_time_ms / 1000.0

        p_term = self.kp * self.error
        integrate = True

        if abs(self.error) <= self.error_threshold:
            integrate = False

        if self.max_output > 0.0:
            # Anti-windup: se fija en la salida del ciclo anterior.
            if ((self.output >= self.max_output and self.error > 0)
                    or (self.output <= -self.max_output and self.error < 0)):
                integrate = False

        if integrate:
            self.integral_error += self.error * delta_time_s

        i_term = self.ki * self.integral_error

        if self.derivative_on_measurement:
            d_term = -self.kd * (measurement - self._previous_measurement) / delta_time_s
        else:
            d_term = self.kd * (self.error - self.previous_error) / delta_time_s

        calculate_output = p_term + i_term + d_term
        self.p_term, self.i_term, self.d_term = p_term, i_term, d_term

        if self.min_output > 0.0 and abs(calculate_output) < self.min_output:
            calculate_output = 0.0

        if self.max_output > 0.0:
            self.output = _clamp(calculate_output, -self.max_output, self.max_output)
        else:
            self.output = calculate_output

        if self.one_direction_only:
            if self.accept_type == PID_ACCEPT_POSITIVES_ONLY and self.output < 0:
                self.output = 0.0
            if self.accept_type == PID_ACCEPT_NEGATIVES_ONLY and self.output > 0:
                self.output = 0.0

        if self.reset_within_threshold and abs(self.error) < self.error_threshold:
            self.reset()
            self.was_reset = True
            self.reset_count += 1
        else:
            self.previous_error = self.error
            self._previous_measurement = measurement
            self.last_iteration_ms = current_time_ms

        return self.output

    def update(self, target, measurement, current_time_ms):
        """Lo que asumo que hace el lazo del Arduino en cada ciclo: escribe
        target y error = target - medición, llama a compute() y usa output como
        PWM. (No vi el código que llama a PID::compute; si en el robot el target
        se escribe una sola vez, reset() lo deja en 0, ojo con eso.)
        """
        self.target = target
        self.error = target - measurement
        return self.compute(current_time_ms)


# Planta
class MotorPlant:
    """Motor + rueda como sistema de primer orden: tau * dv/dt = K*pwm - v.

    speed es la velocidad real (RPS) y position el ángulo acumulado (vueltas).
    Los efectos son opcionales y cada uno se apaga con el valor que dice:

      * deadzone_pwm (0 = sin zona muerta): si |pwm| < deadzone_pwm el motor no
        recibe empuje (pwm efectivo = 0). Si ya venía girando, frena por
        inercia, no se para en seco.
      * friction_pwm (0 = sin fricción): fricción de Coulomb en unidades de
        PWM. Siempre se opone al giro con la misma magnitud y, en reposo, se
        queda quieto mientras el empuje no la venza.
      * pulses_per_rev (None = sin cuantización): el encoder cuenta pulsos
        enteros; la medición es (pulsos de la última ventana) / (N * sample_time).
        Sin cuantización mides la velocidad instantánea.
      * noise_std (0 = sin ruido): desviación estándar (RPS) del ruido gaussiano
        que se le suma a la medición.

    NO modela inductancia, juego mecánico, caída de batería ni retardo de
    cómputo. El PWM se queda constante durante cada periodo de control.
    """

    def __init__(self, gain=PLANT_GAIN, tau=PLANT_TAU, deadzone_pwm=0.0,
                 friction_pwm=0.0, noise_std=0.0, pulses_per_rev=None,
                 sample_time=SAMPLE_TIME_MS / 1000.0, physics_dt=0.001, seed=0):
        self.gain = gain
        self.tau = tau
        self.deadzone_pwm = deadzone_pwm
        self.friction_pwm = friction_pwm
        self.noise_std = noise_std
        self.pulses_per_rev = pulses_per_rev
        self.sample_time = sample_time
        self.physics_dt = physics_dt
        self.rng = np.random.default_rng(seed)

        self.speed = 0.0
        self.position = 0.0
        self._last_count = 0

    def max_speed(self, max_pwm):
        """Velocidad en estado estable con |pwm| = max_pwm (RPS), o sea el techo."""
        if max_pwm < self.deadzone_pwm:
            return 0.0
        return max(0.0, self.gain * (max_pwm - self.friction_pwm))

    def min_sustained_speed(self):
        """La velocidad más baja (que no sea 0) que se puede sostener con PWM
        constante: la que da un PWM justo igual a la zona muerta. 0 si no hay
        zona muerta."""
        if self.deadzone_pwm <= 0.0:
            return 0.0
        return max(0.0, self.gain * (self.deadzone_pwm - self.friction_pwm))

    def encoder_resolution(self):
        """Cuántos RPS vale un pulso en una ventana (0 si no hay cuantización)."""
        if not self.pulses_per_rev:
            return 0.0
        return 1.0 / (self.pulses_per_rev * self.sample_time)

    def step(self, pwm, duration):
        """Avanza la física `duration` segundos con el PWM constante (Euler con
        paso physics_dt). Regresa la velocidad real al final."""
        effective = 0.0 if abs(pwm) < self.deadzone_pwm else pwm
        drive = self.gain * effective          # velocidad hacia la que empuja el PWM
        friction = self.gain * self.friction_pwm
        n = max(1, int(round(duration / self.physics_dt)))
        h = duration / n
        for _ in range(n):
            if abs(self.speed) > 1e-9:
                opposing = math.copysign(friction, self.speed)
                new_speed = self.speed + h * (drive - self.speed - opposing) / self.tau
                if new_speed * self.speed < 0 and abs(drive) <= friction:
                    new_speed = 0.0            # la fricción frena, no hace que gire al revés
            elif abs(drive) <= friction:
                new_speed = 0.0                # quieto y sin empuje suficiente
            else:
                new_speed = h * (drive - math.copysign(friction, drive)) / self.tau
            self.position += 0.5 * (self.speed + new_speed) * h
            self.speed = new_speed
        return self.speed

    def read_encoder(self):
        """Medición de velocidad (RPS) tal como la vería el firmware.

        Llámala una vez por periodo de control: con cuantización, cada llamada
        cierra la ventana de conteo y abre la siguiente.
        """
        if self.pulses_per_rev:
            count = math.floor(self.position * self.pulses_per_rev)
            measured = (count - self._last_count) / (self.pulses_per_rev * self.sample_time)
            self._last_count = count
        else:
            measured = self.speed
        if self.noise_std > 0.0:
            measured += self.rng.normal(0.0, self.noise_std)
        return measured


# Simulación y métricas 

def target_at(schedule, t):
    """Qué target toca en el instante t, dada una lista [(t_inicio, target), ...]."""
    current = schedule[0][1]
    for start, value in schedule:
        if t >= start - 1e-9:
            current = value
    return current


def validate_target(target, plant, max_output):
    """Avisa por consola si el target no se puede sostener con esta planta.

    Regresa True si |target| <= velocidad máxima de la planta con max_output.
    """
    vmax = plant.max_speed(max_output)
    vmin = plant.min_sustained_speed()
    if abs(target) > vmax:
        print(f"  ADVERTENCIA: target {target:.2f} RPS > velocidad máxima de la planta "
              f"({vmax:.2f} RPS con PWM {max_output:.0f}). Es inalcanzable.")
        return False
    if 0.0 < abs(target) < vmin:
        print(f"  ADVERTENCIA: target {target:.2f} RPS por debajo de {vmin:.2f} RPS, "
              f"la mínima sostenible con la zona muerta; el motor oscilará.")
    else:
        print(f"  Target {target:.2f} RPS alcanzable (máximo de la planta: {vmax:.2f} RPS).")
    return True


def simulate(pid, plant, schedule, duration_s):
    """Corre el lazo cerrado y regresa un dict de arreglos, uno por ciclo.

    Cada ciclo va en el mismo orden que en el firmware: leer encoder, calcular
    el PID, aplicar el PWM los siguientes 25 ms. `speed` es la velocidad real
    de la planta al muestrear; `measured` es lo que ve el PID.
    """
    n = int(round(duration_s * 1000 / SAMPLE_TIME_MS))
    keys = ("t", "target", "speed", "measured", "error", "pwm", "p", "i", "d", "reset")
    log = {k: np.zeros(n) for k in keys}
    for k in range(n):
        now_ms = k * SAMPLE_TIME_MS            # millis() de mentiritas
        t = now_ms / 1000.0
        target = target_at(schedule, t)
        measured = plant.read_encoder()
        pwm = pid.update(target, measured, now_ms)
        row = (t, target, plant.speed, measured, target - measured, pwm,
               pid.p_term, pid.i_term, pid.d_term, float(pid.was_reset))
        for key, value in zip(keys, row):
            log[key][k] = value
        plant.step(pwm, SAMPLE_TIME_MS / 1000.0)
    return log


def _first_crossing(ts, progress, level):
    """Primer instante (interpolado) en que progress >= level; None si nunca pasa."""
    idx = np.nonzero(progress >= level)[0]
    if idx.size == 0:
        return None
    i = idx[0]
    if i == 0:
        return float(ts[0])
    frac = (level - progress[i - 1]) / (progress[i] - progress[i - 1])
    return float(ts[i - 1] + frac * (ts[i] - ts[i - 1]))


def step_metrics(log, t_start, t_end, target):
    """Métricas de un escalón, calculadas sobre la velocidad REAL de la planta
    (no la que mide el encoder).

    El escalón va de la velocidad en t_start hasta `target`, y los tiempos se
    cuentan desde t_start. La banda de establecimiento es ±2 % de la amplitud
    del escalón. El error en estado estable es el promedio de (target -
    velocidad) en los últimos STEADY_WINDOW_S; el rizado es el pico a pico en
    esa ventana. Si un tiempo no se alcanza dentro del tramo, sale None.
    """
    mask = (log["t"] >= t_start - 1e-9) & (log["t"] < t_end - 1e-9)
    ts = log["t"][mask] - t_start
    ys = log["speed"][mask]
    y0 = ys[0]
    amplitude = target - y0
    band = SETTLING_BAND * abs(amplitude)
    progress = (ys - y0) / amplitude

    t10 = _first_crossing(ts, progress, 0.1)
    t90 = _first_crossing(ts, progress, 0.9)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None

    outside = np.nonzero(np.abs(ys - target) > band)[0]
    if outside.size == 0:
        settling = 0.0
    elif outside[-1] == len(ys) - 1:
        settling = None
    else:
        settling = float(ts[outside[-1] + 1])

    tail = ys[ts >= ts[-1] - STEADY_WINDOW_S + 1e-9]
    ss_error = float(target - tail.mean())
    return {
        "t_start": t_start, "target": target, "y0": float(y0),
        "amplitude": float(amplitude), "band": float(band),
        "overshoot_pct": float(max(0.0, (progress.max() - 1.0) * 100.0)),
        "rise_time": rise, "settling_time": settling, "ss_error": ss_error,
        "ripple_pp": float(tail.max() - tail.min()),
        "pwm_max": float(np.abs(log["pwm"][mask]).max()),
        "converged": settling is not None and abs(ss_error) <= band,
    }


def _fmt_time(value):
    return "no se alcanza" if value is None else f"{value:.3f} s"


def print_table(log, every_s=0.1):
    """Tabla resumida: una fila cada `every_s` segundos, no cada ciclo (si no sería eterna)."""
    stride = max(1, int(round(every_s * 1000 / SAMPLE_TIME_MS)))
    header = (f"{'t (s)':>6} | {'Target':>6} | {'Real':>6} | {'Medida':>6} | "
              f"{'Error':>6} | {'PWM':>7} | {'P':>7} | {'I':>7} | {'D':>6}")
    print(header)
    print("-" * len(header))
    for k in range(0, len(log["t"]), stride):
        print(f"{log['t'][k]:>6.2f} | {log['target'][k]:>6.2f} | {log['speed'][k]:>6.2f} | "
              f"{log['measured'][k]:>6.2f} | {log['error'][k]:>6.2f} | {log['pwm'][k]:>7.1f} | "
              f"{log['p'][k]:>7.1f} | {log['i'][k]:>7.1f} | {log['d'][k] + 0.0:>6.1f}")


def print_metrics(m):
    print(f"  Escalón {m['y0']:.2f} -> {m['target']:.2f} RPS (desde t = {m['t_start']:.2f} s)")
    print(f"    Sobrepaso:                 {m['overshoot_pct']:.2f} %")
    print(f"    Tiempo de subida (10-90%): {_fmt_time(m['rise_time'])}")
    print(f"    Tiempo de establec. (2%):  {_fmt_time(m['settling_time'])}")
    print(f"    Error en estado estable:   {m['ss_error']:+.4f} RPS "
          f"(rizado pico a pico {m['ripple_pp']:.4f} RPS)")
    print(f"    PWM máximo usado:          {m['pwm_max']:.1f} de {MAX_PWM:.0f}")


# Gráfica 
INK, INK_2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"
C_SPEED, C_P, C_I, C_D = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"


def _style_axis(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9, length=0)


def plot_results(results, path):
    """Guarda la figura: una columna por escenario. Fila (a): velocidad real,
    medida y target. Fila (b): PWM aplicado y términos P, I, D con la línea de
    saturación."""
    n = len(results)
    fig, axes = plt.subplots(2, n, figsize=(4.7 * n, 7.6), squeeze=False,
                             sharey="row", facecolor=SURFACE)
    for col, res in enumerate(results):
        log, plant = res["log"], res["plant"]
        t = log["t"]
        noisy = bool(plant.pulses_per_rev) or plant.noise_std > 0.0

        ax = axes[0][col]
        _style_axis(ax)
        if noisy:
            ax.step(t, log["measured"], where="post", color=MUTED, linewidth=0.8,
                    alpha=0.7, label="Medida por el encoder")
        ax.step(t, log["target"], where="post", color=INK, linewidth=1.2,
                linestyle=(0, (4, 3)), label="Target")
        ax.plot(t, log["speed"], color=C_SPEED, linewidth=2.0, label="Velocidad real")
        ax.set_title(f"Escenario {res['key']}: {res['short']}", loc="left",
                     fontsize=10, color=INK, fontweight="bold", pad=22)
        m = res["metrics"][-1]             # si hay varios escalones, el último
        prefix = "" if len(res["metrics"]) == 1 else f"{m['y0']:.0f}→{m['target']:.0f} RPS: "
        ax.text(0.0, 1.03,
                f"{prefix}sobrepaso {m['overshoot_pct']:.1f} %  ·  subida "
                f"{_fmt_time(m['rise_time'])}  ·  establec. {_fmt_time(m['settling_time'])}",
                transform=ax.transAxes, fontsize=8.5, color=INK_2)

        ax = axes[1][col]
        _style_axis(ax)
        ax.axhline(MAX_PWM, color=INK_2, linewidth=1.0, linestyle=(0, (2, 2)))
        ax.axhline(-MAX_PWM, color=INK_2, linewidth=1.0, linestyle=(0, (2, 2)))
        ax.step(t, log["p"], where="post", color=C_P, linewidth=1.4, label="Término P")
        ax.step(t, log["i"], where="post", color=C_I, linewidth=1.4, label="Término I")
        ax.step(t, log["d"], where="post", color=C_D, linewidth=1.4, label="Término D")
        ax.step(t, log["pwm"], where="post", color=INK, linewidth=2.0, label="PWM aplicado")
        ax.set_xlabel("Tiempo (s)", fontsize=9.5, color=INK_2)
        if col == 0:
            ax.text(0.99, MAX_PWM, "saturación +255 ", ha="right", va="bottom",
                    transform=ax.get_yaxis_transform(), fontsize=8.5, color=INK_2)
            ax.text(0.99, -MAX_PWM, "saturación −255 ", ha="right", va="bottom",
                    transform=ax.get_yaxis_transform(), fontsize=8.5, color=INK_2)

    axes[0][0].set_ylabel("(a) Velocidad (RPS)", fontsize=9.5, color=INK_2)
    axes[1][0].set_ylabel("(b) PWM y términos del PID", fontsize=9.5, color=INK_2)
    axes[1][0].set_ylim(-MAX_PWM * 1.15, MAX_PWM * 1.15)
    for row, ref in ((0, 1 if n > 1 else 0), (1, 0)):
        legend = axes[row][ref].legend(loc="lower right" if row == 0 else "center right",
                                       fontsize=8.5, frameon=True, facecolor=SURFACE,
                                       edgecolor=GRID, framealpha=0.95)
        for text in legend.get_texts():
            text.set_color(INK_2)

    pid = results[0]["pid"]
    fig.suptitle(f"PID de velocidad VSS: respuesta simulada  (Kp = {pid.kp:g}, "
                 f"Ki = {pid.ki:g}, Kd = {pid.kd:g}, Ts = {SAMPLE_TIME_MS} ms)",
                 x=0.012, ha="left", fontsize=13, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


# Escenarios (aquí se juntan todos)
REALISTIC_PLANT = dict(
    deadzone_pwm=30.0,      # SUPUESTO: |PWM| < 30 el motor ni se entera (medir; el sinfín puede tener más)
    friction_pwm=10.0,      # SUPUESTO: fricción de Coulomb, en unidades de PWM
    noise_std=0.02,         # SUPUESTO: ruido de la medición, en RPS
    pulses_per_rev=WHEEL_PULSES_PER_REV,   # 7 * 42 * ENCODER_EDGES (de la hoja de datos)
    seed=7,
)
RESET_THRESHOLD_RPS = 0.1   # error_threshold del escenario 4

TARGET_RPS = 6.0            # velocidad de prueba (máximo del motor: ~8.8 RPS a 6 V)
TARGET_2_RPS = 3.0          # segundo target del escenario 3

SCENARIOS = [
    dict(key="1", title="planta ideal",
         plant={}, pid={}, schedule=[(0.0, TARGET_RPS)], duration=3.0),
    dict(key="2", title="zona muerta, fricción, ruido y cuantización",
         short="zona muerta + ruido + cuantización", plant=REALISTIC_PLANT, pid={},
         schedule=[(0.0, TARGET_RPS)], duration=3.0),
    dict(key="3", title=f"cambio de target {TARGET_RPS} a {TARGET_2_RPS} RPS",
         plant=REALISTIC_PLANT, pid={},
         schedule=[(0.0, TARGET_RPS), (2.0, TARGET_2_RPS)], duration=4.0),
    # Extra: la misma planta del 2 pero con la bandera del firmware prendida.
    dict(key="4", title="como el 2, con reset_within_threshold",
         short="el 2 con reset_within_threshold", plant=REALISTIC_PLANT,
         pid=dict(error_threshold=RESET_THRESHOLD_RPS, reset_within_threshold=True),
         schedule=[(0.0, TARGET_RPS)], duration=3.0),
]


def run_scenario(cfg, gains):
    kp, ki, kd = gains
    plant = MotorPlant(**cfg["plant"])
    pid = PIDController(kp, ki, kd, max_output=MAX_PWM, **cfg["pid"])

    print("\n" + "=" * 78)
    print(f"ESCENARIO {cfg['key']}: {cfg['title']}")
    print("=" * 78)
    if plant.pulses_per_rev:
        print(f"  Encoder: {plant.pulses_per_rev} pulsos/vuelta -> "
              f"{plant.encoder_resolution():.3f} RPS por pulso en {SAMPLE_TIME_MS} ms")
    if pid.reset_within_threshold:
        print(f"  reset_within_threshold = True, error_threshold = {pid.error_threshold} RPS")
    for _, target in cfg["schedule"]:
        validate_target(target, plant, pid.max_output)
    print()

    log = simulate(pid, plant, cfg["schedule"], cfg["duration"])
    print_table(log)

    starts = [s for s, _ in cfg["schedule"]] + [cfg["duration"]]
    metrics = [step_metrics(log, starts[i], starts[i + 1], cfg["schedule"][i][1])
               for i in range(len(cfg["schedule"]))]
    print()
    for m in metrics:
        print_metrics(m)
    if log["pwm"][0] != log["p"][0]:
        print(f"  Nota: en el primer ciclo P = {log['p'][0]:.1f} pero el PWM aplicado fue "
              f"{log['pwm'][0]:.1f} (clamp de first_run en pid.cpp).")
    if pid.reset_within_threshold:
        print(f"  reset() se ejecutó {pid.reset_count} veces en {cfg['duration']:.1f} s.")
    return dict(key=cfg["key"], title=cfg["title"], short=cfg.get("short", cfg["title"]),
                log=log, metrics=metrics, plant=plant, pid=pid)


def print_conclusion(results):
    """La conclusión sale de las métricas que dieron, no es texto fijo."""
    print("\n" + "=" * 78)
    print("CONCLUSIÓN")
    print("=" * 78)
    by_key = {r["key"]: r for r in results}
    for res in results:
        for m in res["metrics"]:
            tag = f"Escenario {res['key']} ({m['y0']:.1f} -> {m['target']:.1f} RPS)"
            if m["converged"]:
                print(f"- {tag}: CONVERGE. Entra a ±2 % en {m['settling_time']:.2f} s, "
                      f"sobrepaso {m['overshoot_pct']:.1f} %, error estable "
                      f"{abs(m['ss_error']):.3f} RPS "
                      f"({abs(m['ss_error']) / abs(m['target']) * 100:.2f} % del target).")
            else:
                print(f"- {tag}: NO CONVERGE a ±2 % en la ventana simulada. Error medio "
                      f"final {m['ss_error']:+.3f} RPS, rizado {m['ripple_pp']:.3f} RPS "
                      f"pico a pico.")

    m1 = by_key["1"]["metrics"][0]
    ok = (m1["settling_time"] is not None and m1["settling_time"] < REQ_SETTLING_S
          and m1["overshoot_pct"] <= REQ_OVERSHOOT_PCT)
    print(f"\nRequisito del escenario 1 (±2 % en < {REQ_SETTLING_S:.0f} s y sobrepaso "
          f"<= {REQ_OVERSHOOT_PCT:.0f} %): {'SE CUMPLE' if ok else 'NO SE CUMPLE'} "
          f"({_fmt_time(m1['settling_time'])}, {m1['overshoot_pct']:.1f} %).")

    if "2" in by_key and "4" in by_key:
        m2, m4 = by_key["2"]["metrics"][0], by_key["4"]["metrics"][0]
        resets = by_key["4"]["pid"].reset_count
        if not m4["converged"] and m2["converged"]:
            print(f"\nreset_within_threshold: con la bandera activa el lazo de velocidad NO "
                  f"converge ({resets} resets, rizado {m4['ripple_pp']:.2f} RPS frente a "
                  f"{m2['ripple_pp']:.3f} RPS sin ella). Cada reset pone el PWM en 0 y borra "
                  f"el integral, que es justo lo que sostiene la velocidad.")
        elif m4["ripple_pp"] > 2.0 * m2["ripple_pp"]:
            print(f"\nreset_within_threshold: converge, pero el rizado sube de "
                  f"{m2['ripple_pp']:.3f} a {m4['ripple_pp']:.3f} RPS ({resets} resets).")
        else:
            print(f"\nreset_within_threshold: sin degradación medible ({resets} resets, "
                  f"rizado {m4['ripple_pp']:.3f} RPS frente a {m2['ripple_pp']:.3f} RPS).")

    required = [m for r in results if r["key"] in ("1", "2", "3") for m in r["metrics"]]
    passed = sum(m["converged"] for m in required)
    worst = max(abs(m["ss_error"]) for m in required)
    pid = results[0]["pid"]
    if passed == len(required):
        print(f"\nResultado: con Kp = {pid.kp:g}, Ki = {pid.ki:g}, Kd = {pid.kd:g} el PID "
              f"converge en los {len(required)} escalones de los escenarios 1 a 3, con "
              f"error en estado estable < {worst + 0.0005:.3f} RPS.")
    else:
        print(f"\nResultado: con Kp = {pid.kp:g}, Ki = {pid.ki:g}, Kd = {pid.kd:g} el PID "
              f"converge solo en {passed} de {len(required)} escalones de los escenarios "
              f"1 a 3. Hay que revisar la sintonía antes de pasarla al robot.")


def main():
    gains = tune_pi_pole_cancellation(PLANT_GAIN, PLANT_TAU, DESIRED_TAU_CL)
    kp, ki, kd = gains
    print("=== MVP: SIMULACIÓN DEL PID DE VELOCIDAD (VSS) ===")
    print(f"Motor: {MOTOR_NO_LOAD_RPM:g} RPM sin carga a {MOTOR_RATED_V:g} V, reducción 1:{GEAR_RATIO}, "
          f"encoder {ENCODER_PPR_MOTOR} PPR, alimentado a {SUPPLY_V:g} V.")
    print(f"  Paso 1  K   = ({MOTOR_NO_LOAD_RPM:g}/60) * ({SUPPLY_V:g}/{MOTOR_RATED_V:g}) / {MAX_PWM:.0f}"
          f" = {MAX_WHEEL_RPS:.2f} / {MAX_PWM:.0f} = {PLANT_GAIN:.4f} RPS/PWM")
    print(f"  Paso 2  tau = {PLANT_TAU} s (supuesto)")
    print(f"  Paso 3  encoder en la rueda = {ENCODER_PPR_MOTOR} x {GEAR_RATIO} x {ENCODER_EDGES}"
          f" = {WHEEL_PULSES_PER_REV} pulsos/vuelta")
    print(f"  Paso 4  Kp  = tau/(K*tau_cl) = {PLANT_TAU}/({PLANT_GAIN:.4f}*{DESIRED_TAU_CL}) = {kp:.2f}")
    print(f"          Ki  = Kp/tau = {kp:.2f}/{PLANT_TAU} = {ki:.2f}")
    print(f"          Kd  = {kd:g}")
    kick = kp * TARGET_RPS
    print(f"  Golpe inicial Kp*target = {kp:.1f}*{TARGET_RPS:g} = {kick:.0f} "
          f"({'cabe en' if kick <= MAX_PWM else 'SATURA'} {MAX_PWM:.0f})")
    print(f"Predicción del modelo: subida ~{2.2 * DESIRED_TAU_CL:.2f} s, "
          f"establecimiento ~{3.9 * DESIRED_TAU_CL:.2f} s, sin sobrepaso.")
    print("\nValidación de los targets:")
    validate_target(TARGET_RPS, MotorPlant(), MAX_PWM)
    validate_target(10.0, MotorPlant(), MAX_PWM)

    results = [run_scenario(cfg, gains) for cfg in SCENARIOS]

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "respuesta_pid.png")
    plot_results(results, path)
    print_conclusion(results)
    print(f"\nGráfica guardada en: {path}")


if __name__ == "__main__":
    main()