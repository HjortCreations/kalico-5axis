; 5-axis full motion dry run for Kalico corexy_3z_tilt
; Target machine: modified RatRig V-Core 3.1, 200 mm build volume
; Purpose: verify XYZAB motion without heaters or extrusion.
;
; WARNING: USE ENTIRELY AT YOUR OWN RISK.
; This is not a universal RatRig test and is not endorsed by RatRig.
; It assumes a matching 5-axis Kalico configuration and the limits below.
; Review every move against your printer.cfg before running it.
; Home all axes first, clear the build volume, and keep a hand near the
; emergency stop. This file moves all motion axes.
;
; Assumed limits from printer.cfg:
;   X 0..200, Y 0..200, Z -5..200
;   A/B +/-6 deg, combined tilt max 6 deg
; This dry run uses conservative positions:
;   XY within 40..160, Z >= 28, A/B individual +/-6 deg
;   Combined A/B moves use +/-4 deg to remain under 6 deg total tilt.
; IMPORTANT: This file assumes you have already homed the machine.

M400
M104 S0
M140 S0
M107
G21
G90
M83
SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=3000 SQUARE_CORNER_VELOCITY=5
RESPOND TYPE=echo MSG="WARNING: own-risk RatRig V-Core 3.1 200 mm 5-axis dry run starting; assumes all axes are homed"

; Start in pivot mode. Homing is intentionally not performed in this file.
G49
M400

; Move to a safe center position.
RESPOND TYPE=echo MSG="Moving to safe center Z"
G1 Z30 F5400
G1 X100 Y100 F12000
G1 A0 B0 F1080
M400

; XYZ individual axis tests.
RESPOND TYPE=echo MSG="Testing X axis"
G1 X40 F12000
G1 X160 F12000
G1 X100 F12000
M400

RESPOND TYPE=echo MSG="Testing Y axis"
G1 Y40 F12000
G1 Y160 F12000
G1 Y100 F12000
M400

RESPOND TYPE=echo MSG="Testing Z axis"
G1 Z45 F5400
G1 Z28 F5400
G1 Z30 F5400
M400

; Pivot-mode A/B tests. XY stays centered so support motion is simple.
RESPOND TYPE=echo MSG="Testing A tilt in pivot mode"
G1 A6.0 B0.0 F1080
G1 A-6.0 B0.0 F1080
G1 A0.0 B0.0 F1080
M400

RESPOND TYPE=echo MSG="Testing B tilt in pivot mode"
G1 A0.0 B6.0 F1080
G1 A0.0 B-6.0 F1080
G1 A0.0 B0.0 F1080
M400

RESPOND TYPE=echo MSG="Testing combined A/B tilt in pivot mode"
G1 A4.0 B4.0 F1080
G1 A-4.0 B4.0 F1080
G1 A-4.0 B-4.0 F1080
G1 A4.0 B-4.0 F1080
G1 A0.0 B0.0 F1080
M400

; Combined XYZ motion without tilt.
RESPOND TYPE=echo MSG="Testing coordinated XYZ moves"
G1 X40 Y40 Z35 F9000
G1 X160 Y40 Z30 F9000
G1 X160 Y160 Z35 F9000
G1 X40 Y160 Z30 F9000
G1 X100 Y100 Z30 F9000
M400

; TCP mode tests. At nonzero tilt, XY motion should drive the Z supports.
RESPOND TYPE=echo MSG="Enabling TCP mode"
G43.4
M400

RESPOND TYPE=echo MSG="Testing TCP XY motion at A=0 B=0"
G1 A0 B0 F1080
G1 X50 Y50 Z30 F12000
G1 X150 Y50 Z30 F12000
G1 X150 Y150 Z30 F12000
G1 X50 Y150 Z30 F12000
G1 X100 Y100 Z30 F12000
M400

RESPOND TYPE=echo MSG="Testing TCP XY motion with A tilt"
G1 A4.0 B0.0 F1080
G1 X70 Y70 Z30 F6000
G1 X130 Y70 Z30 F6000
G1 X130 Y130 Z30 F6000
G1 X70 Y130 Z30 F6000
G1 X100 Y100 Z30 F6000
G1 A0.0 B0.0 F1080
M400

RESPOND TYPE=echo MSG="Testing TCP XY motion with B tilt"
G1 A0.0 B4.0 F1080
G1 X70 Y70 Z30 F6000
G1 X130 Y70 Z30 F6000
G1 X130 Y130 Z30 F6000
G1 X70 Y130 Z30 F6000
G1 X100 Y100 Z30 F6000
G1 A0.0 B0.0 F1080
M400

RESPOND TYPE=echo MSG="Testing TCP combined XYZAB"
G1 X80 Y80 Z34 A4.0 B4.0 F5400
G1 X120 Y80 Z28 A-4.0 B4.0 F5400
G1 X120 Y120 Z34 A-4.0 B-4.0 F5400
G1 X80 Y120 Z28 A4.0 B-4.0 F5400
G1 X100 Y100 Z30 A0.0 B0.0 F5400
M400

; Return to a neutral parked state.
RESPOND TYPE=echo MSG="Returning to center and neutral tilt"
G1 X100 Y100 Z40 A0 B0 F6000
G49
SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=3000 SQUARE_CORNER_VELOCITY=5
M400
RESPOND TYPE=echo MSG="5-axis dry run complete"
