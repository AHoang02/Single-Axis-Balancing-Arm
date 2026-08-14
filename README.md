# Embedded C++ PID Motor Control System — Single-Axis Balancing Arm

A bare-metal C++ closed-loop motor controller built on an STM32 Nucleo-F446RE (Cortex-M4), driving a brushless motor via ESC/PWM to stabilize a single-axis balancing arm using real-time potentiometer feedback.

🎥 **Demo video:** 

## Overview

This project was built to develop genuine embedded C++ fluency deliberately implemented at the register level (direct CMSIS access, no HAL/CubeMX) rather than relying on generated peripheral drivers. It covers the full stack from bare-metal peripheral bring-up through a tuned PID control loop running on real hardware.

**Hardware:**
- STM32 Nucleo-F446RE (Cortex-M4)
- Brushless motor (BR2212) + 30A ESC, driven via hardware PWM
- Potentiometer position feedback (ADC)
- Bench power supply for motor power, isolated from the Nucleo's logic supply

## Architecture

Three C++ classes, each wrapping one peripheral and hiding its register-level configuration behind a small public interface:

- **`esc`** — owns a `TIM_TypeDef*`, configures the timer for 50Hz PWM generation on construction, exposes `setThrottle(percent)` which clamps input, converts percent → pulse width → compare-register value, and writes it
- **`adc`** — owns an `ADC_TypeDef*`, configures GPIO analog mode, channel selection, and sample timing on construction, exposes `readRaw()` for a single blocking conversion
- **`PID`** — holds gains and running state (integral accumulator, previous error), exposes `compute(setpoint, measured, dt)` returning a bounded correction term

A `main()` superloop ties them together: read ADC → compute PID correction → apply to a baseline throttle → command the ESC, using a SysTick-driven millisecond timebase for a real, measured `dt` rather than an estimated loop period.

## Notable engineering / debugging work

- **HardFault root-caused via register-level fault decoding**, not trial and error: traced a Cortex-M4 HardFault to an unenabled FPU coprocessor by reading and manually decoding the Cortex-M4 Configurable Fault Status Register (CFSR) bit-by-bit against the ARM architecture reference, isolating the exact fault type (`NOCP` — no coprocessor access) before applying the fix (`SCB->CPACR` + barrier instructions).
- **Anti-windup and output-bounding on the PID controller**: the integral term is clamped to prevent windup, and total controller output is bounded relative to a baseline throttle so no combination of sensor noise or transient error can drive the actuator to full authority.
- **A safety-clamp interaction bug**: an ADC input-range clamp — intended only to guard against sensor glitches — was initially set too narrow, silently capping the controller's view of real position error during a large excursion and preventing an adequate corrective response. Widening the sensor clamp to a true safety-only bound (while keeping a separate, tighter clamp on controller *output*) resolved it — a concrete lesson in why sensor-input clamps and actuator-output clamps serve different purposes and need independently-chosen bounds.

## Control approach

Because the motor/ESC can only produce positive (unidirectional) thrust, PID output is applied as a correction on top of an empirically-determined baseline throttle (rather than as the full throttle command from scratch), letting the controller make small bidirectional adjustments around a point that's already close to equilibrium. Gains were tuned iteratively in the standard order — P first, then D to damp oscillation, then a small I to eliminate residual steady-state offset — with the controller validated against manual disturbances (nudging the arm and observing recovery).

## Build

Built in STM32CubeIDE. CMSIS device/core headers (from ST's `STM32CubeF4` firmware package) required in the include path; no HAL is used.

## Skills demonstrated

C++ (OOP, RAII-adjacent hardware ownership patterns), bare-metal ARM Cortex-M register programming, PID control theory and tuning, real-time embedded debugging (SFR inspection, fault register decoding, live variable watching via GDB/STM32CubeIDE).
