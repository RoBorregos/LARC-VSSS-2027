# PID dinámico para robots VSSS con ESP32-S3-Mini — Resumen
 
> Todo lo numérico sale de la hoja de datos y de simulación. **Aún no hay robot físico**: ninguna cifra es comportamiento observado.
 
## Idea principal
 
Ruta escalonada:
 
1. Corregir los 2 errores de `pid.cpp`.
2. Pasar a **PI + feedforward + compensación de batería y zona muerta**.
3. Añadir **gain scheduling** (por voltaje y velocidad).
4. Usar **RLS** solo si los datos del robot real demuestran que hace falta.
## Datos del proyecto
 
| Dato | Valor |
|---|---|
| Motor | N20 con sinfín 1:42, autobloqueo, 6 V |
| Velocidad sin carga | 530 RPM (8.83 RPS) |
| Velocidad / torque nominal | 390 RPM / 0.06 kg·cm |
| Torque / corriente de bloqueo | ≤ 0.16 kg·cm / ≤ 0.65 A |
| Encoder | 7 PPR → 294 pulsos/vuelta (×1) o 1176 (×4) |
| Lazo PID actual | cada 25 ms |
| Referencias UDP | 50 Hz (cada 20 ms) desde ROS 2 |
| PI actual | Kp ≈ 28.9, Ki ≈ 115.5, Kd = 0 (τ = 0.25 s supuesto) |
 
## Errores en `pid.cpp`
 
- **Error 1:** en `first_run`, cambiar `clamp(initial_output, -max_output, min_output)` por `clamp(initial_output, -max_output, max_output)`. Cuesta un ciclo al arrancar y tras cada reset.
- **Error 2 (el grave):** no usar `reset_within_threshold` en el lazo de velocidad (o exigir varios ciclos seguidos), para que el integral sostenga el giro contra la fricción del sinfín.
- **Menor:** guardar `last_iteration_ms = current_time_ms` en vez de volver a llamar a `millis()`.
- Acumular el integral **ya multiplicado por Ki** (`I += Ki·Ts·e`) para poder cambiar Ki sin saltos.
- Pasar todo de `double` a `float` (la FPU de la S3 solo acelera `float`).
## "PID dinámico": variantes
 
| Variante | Cómo funciona | Riesgo |
|---|---|---|
| Gain scheduling | Tabla según velocidad/voltaje | No cubre lo que la tabla no prevé |
| Auto-tuning | Calibración (escalón o relé), luego fijo | No corre durante el partido |
| Adaptativo (RLS) | Estima el modelo en línea | Diverge sin excitación suficiente |
| MRAC | Sigue un modelo de referencia | Sensible a retardos |
| Fuzzy | Reglas "si error grande, sube Kp" | Difícil de validar |
 
**Recomendado:** auto-tuning fuera de línea por rueda + gain scheduling; RLS opcional y supervisado.
 
## Ley de control recomendada (en voltios)
 
```
V_cmd = V_s·sign(r) + K_v·r + Kp·e + Ki·∫e dt
PWM   = PWM_max · V_cmd / V_bat
```
 
- `V_s`: voltaje mínimo para vencer la fricción estática (medir).
- `K_v ≈ 6 V / 8.83 RPS ≈ 0.68 V/RPS` (estimación inicial).
- Dividir entre `V_bat` (medido por ADC) compensa la descarga de la batería.
## Cómo sintonizar
 
1. **Prueba de escalón** en la alfombra (PWM fijo, registrar cada 5–10 ms).
2. Ajustar modelo **FOPDT**: ganancia `k`, retardo `θ`, constante `τ` (63.2 %).
3. Elegir `τc` (Skogestad sugiere `τc = θ`).
4. **SIMC:**
```
Kp = (1/k) · τ / (τc + θ)
τI = min(τ, 4(τc + θ))
Ki = Kp / τI
```
5. Discretizar + anti-windup.
6. Validar en simulador y en robot.
**Compromiso (simulación):** ganancias SIMC agresivas llegan ~22 % antes, pero saturan el PWM al arrancar y lo mueven ~5× más en estado estable. El simulador no modela retardo, así que las favorece.
 
**Relé (alternativa):** oscilar alrededor de una velocidad positiva (~200 RPM), sin cruzar 0 RPM por el juego del sinfín. `Ku ≈ 4d/(πa)`; PI de Ziegler–Nichols: `Kp = 0.45·Ku`, `Ti = Tu/1.2`.
 
## Estabilidad (lazo actual)
 
- Con cancelación polo-cero: ωc = 4 rad/s, margen de fase 90°, margen de retardo ≈ 0.39 s.
- 4× más rápido → margen de retardo baja a 98 ms.
- Retardo propio del lazo digital ≈ 1 periodo → bajar de 25 a 10 ms da más margen.
## RLS (si se usa)
 
- Modelo: `y_k = a·y_{k-1} + b·u_{k-1}` → `τ = -Ts/ln(a)`, `k = b/(1-a)`.
- Factor de olvido λ típico: 0.95–0.999.
- **Pausar** con robot quieto, en saturación o en choques (si no, la covarianza crece y diverge).
- Inicializar con valores de la calibración fuera de línea.
- En la ESP32-S3 cuesta microsegundos (estimación propia).
## Seguridad al cambiar ganancias
 
1. Bumpless transfer (integral en unidades de salida).
2. Límites mín/máx de Kp, Ki y de su velocidad de cambio.
3. Congelar adaptación sin excitación, con saturación o error anormal.
4. Detectar divergencia: k > 0, 0 < a < 1, sin NaN/Inf.
5. Fallback gradual a ganancias fijas probadas.
## Encoder
 
| Lectura | Pulsos/vuelta | 1 pulso @25 ms | 1 pulso @10 ms |
|---|---|---|---|
| ×1 | 294 | 0.136 RPS | 0.340 RPS |
| ×4 (PCNT) | 1176 | 0.034 RPS | 0.085 RPS |
 
- Usar **PCNT en cuadratura ×4** (gratis en CPU).
- Evitar medias móviles largas (8 muestras @25 ms = 87.5 ms de retardo).
- Para baja velocidad: método M/T o pasa-bajas ligero.
## Implementación en ESP32-S3
 
- **PCNT** por rueda (driver nuevo `driver/pulse_cnt.h`).
- **MCPWM** (o LEDC) a ~20 kHz.
- Trabajar el PID en salida normalizada (−1 a +1) o en voltios; escalar al final.
- **Núcleo 1:** PID cada 10 ms con `esp_timer`. **Núcleo 0:** Wi-Fi/UDP.
- Bloquear light sleep (`ESP_PM_NO_LIGHT_SLEEP`).
## Arquitectura recomendada (híbrida)
 
- **ESP32-S3:** lazo rápido, gain scheduling (y RLS opcional).
- **PC:** cinemática inversa, telemetría, identificación, análisis.
- No correr el lazo de rueda en la PC: el jitter UDP lo desestabiliza.
## Telemetría (por ciclo, ~50–60 bytes binarios, 50–100 Hz)
 
`timestamp_us`, `target_rps[3]`, `meas_rps[3]` + conteos, `u_out[3]` + P/I/FF, `v_bat`, `flags`. Evitar `printf`/JSON en el lazo.
 
## Perturbaciones VSSS
 
- Sinfín con autobloqueo: alta fricción, juego al invertir, posible frenado brusco al cortar PWM.
- Acoplamiento entre las 3 ruedas omni.
- Choques y pérdida de tracción → riesgo de windup.
- Diferencias entre motores → **calibrar V_s, K_v, k, τ por rueda**, limitar aceleración.
## Watchdogs y fallas
 
1. TWDT/IWDT del sistema (`esp_task_wdt_add/reset`).
2. Watchdog UDP: >150–250 ms sin paquete → referencias a 0 y limpiar integrales.
3. Stall: PWM alto >300 ms sin pulsos → PWM a 0 y reportar falla.
4. Brownout (`CONFIG_ESP_BROWNOUT_DET`).
## Evaluación
 
- Índices: **IAE**, **ISE**, **ITAE** y **TV** (variación total del PWM).
- Comparar PI corregido + feedforward vs. PID dinámico, mismas trayectorias y batería.
- Extender el simulador con: retardo, descarga de batería, choques, IAE/ITAE/TV y RLS.
## Plan por fases
 
1. **Corrección y simulador:** arreglar `pid.cpp`, `float`, integral en unidades de salida; simular con retardo, batería y choques.
2. **Migración y telemetría:** PCNT ×4, PWM 20 kHz, lazo 10 ms en núcleo 1, UDP binario, watchdogs.
3. **Caracterización en cancha:** escalón por rueda (k, τ, θ, V_s), batería bajo carga, RTT/jitter; implementar feedforward + PI.
4. **Capa dinámica:** gain scheduling; RLS solo si hace falta, con congelamiento, límites y fallback.
## Preguntas para los mentores
 
1. ¿El puente H va directo a batería o por regulador de 6 V? ¿Hay pin ADC para la batería?
2. ¿Se necesita el autobloqueo del sinfín? ¿Podemos hacer escalones con el peso final?
3. ¿Bajamos el PID a 10 ms y fijamos un formato UDP único?
4. ¿ESP-IDF (o PlatformIO) o seguimos con Arduino-ESP32?
5. ¿Qué tiempo de respuesta/error/sobrepaso bastan para decidir si se justifica RLS?
6. ¿Cómo cuenta pulsos `Motor.hpp` y está activo `reset_within_threshold`?
## Pendiente de medir
 
K, τ, θ reales por rueda · V_s real · voltaje de batería bajo carga · retardo real del lazo · RTT/jitter Wi-Fi · si la rueda derrapa al cortar PWM · tiempo real de RLS · si los 7 PPR son por canal.