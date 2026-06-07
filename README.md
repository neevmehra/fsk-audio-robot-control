# FSK Audio-Based Robot Control System

Real-time embedded communication system that transmits joystick commands over an acoustic channel using frequency shift keying (FSK), decodes received audio with ADC sampling and DFT-based frequency detection, and controls an RSLK robot.

## Overview

This project implements a wireless audio command system using two TM4C123 microcontrollers. The transmitter reads joystick input, maps each direction to a command, encodes the command into an audio signal, and outputs the waveform through a DAC and speaker. The receiver captures the transmitted signal through a microphone, samples it using an ADC, detects the dominant frequency, and drives an RSLK robot based on the decoded command.

The system was designed to operate in real time while remaining robust to noise and signal distortion in the acoustic channel.

## Features

* Joystick-controlled robot movement
* Acoustic command transmission using FSK tones
* DAC-generated sine wave output through speaker
* Microphone and ADC-based audio capture
* DFT-based frequency detection for command decoding
* FIFO buffering for real-time sample processing
* Interrupt-driven ADC sampling
* Stability filtering and thresholding for noise-tolerant decoding
* RSLK robot control for forward, backward, left, right, and stop commands

## System Architecture

1. Joystick input is read by the transmitter microcontroller.
2. The joystick direction is mapped to a command.
3. The command is encoded as an FSK audio tone.
4. The DAC generates the waveform and outputs it through a speaker.
5. The receiver microphone captures the transmitted audio.
6. The ADC samples the received signal.
7. A DFT-based decoder identifies the dominant frequency.
8. The decoded command is sent to the RSLK robot motor-control logic.

## Communication Protocol

The system uses frequency shift keying to represent command data as distinct audio tones. Each command is transmitted over the air through a speaker and decoded by identifying the dominant received frequency.

The receiver performs frequency-domain analysis using DFT magnitude calculations across expected command frequencies. Thresholding and stability checks help reject transient noise and prevent incorrect command execution.

## Technical Implementation

### Transmitter

* Reads joystick position using ADC inputs
* Applies directional mapping and deadband logic
* Converts commands into encoded audio tones
* Generates sine wave samples using a lookup table
* Outputs audio through a DAC and speaker circuit

### Receiver

* Samples microphone input using interrupt-driven ADC logic
* Removes DC offset from sampled audio
* Buffers samples using FIFO structures
* Computes DFT magnitudes for target frequencies
* Selects the dominant frequency based on magnitude and threshold checks
* Uses stability filtering before executing robot commands

### Robot Control

* Decoded commands control RSLK robot movement
* Supported commands include forward, backward, left, right, and stop
* The system runs within real-time constraints with sufficient CPU timing margin

## Hardware

* TM4C123 microcontrollers
* RSLK robot platform
* Joystick input module
* DAC output circuit
* Speaker
* Microphone receiver circuit
* Breadboarded analog interface circuitry
* Oscilloscope for waveform validation

## Software

* Embedded C
* Interrupt-driven ADC sampling
* DAC waveform generation
* FIFO buffering
* DFT-based frequency detection
* Command decoding and robot-control logic

## Testing

The system was tested at both the module and integration levels.

### Unit Testing

* Verified DAC sine wave output using oscilloscope traces
* Tested joystick ADC readings and command mapping
* Validated microphone ADC sampling
* Tested DFT frequency detection using known input tones
* Checked frame parsing and command decoding logic

### Integration Testing

* Verified end-to-end joystick-to-robot command transmission
* Tested real-time robot response to live joystick input
* Evaluated noise tolerance under background audio interference
* Measured communication reliability over increasing speaker-to-microphone distance
* Confirmed timing performance under real-time sampling constraints

## Results

* Successfully transmitted joystick commands over an acoustic channel
* Controlled an RSLK robot using decoded audio commands
* Maintained reliable command detection using DFT-based decoding
* Demonstrated real-time operation with interrupt-driven sampling and FIFO buffering
* Operated with tolerance to moderate environmental noise

## Repository Structure

```text
.
├── README.md
├── sw/                  # Embedded software
├── hw/                  # Hardware design files
├── Resources/           # Supporting resources
└── docs/                # Lab report and documentation
```

## Project Context

This project was completed for ECE445L and focused on real-time embedded systems, digital signal processing, hardware/software integration, and robotics control.
