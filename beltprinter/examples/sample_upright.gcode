; Sample upright-sliced G-code (a tiny 2-layer square) for testing Nelox Belt.
; Slice this kind of file in OrcaSlicer with a flat-bed profile, then run it
; through nelox_belt.py to produce belt-printer G-code.
G90
M82
; nelox:begin
G1 Z0.2 F600
G1 X0 Y0 F3000
G1 X20 Y0 E0.8 F1200
G1 X20 Y20 E1.6
G1 X0 Y20 E2.4
G1 X0 Y0 E3.2
G1 Z0.4 F600
G1 X20 Y0 E4.0 F1200
G1 X20 Y20 E4.8
G1 X0 Y20 E5.6
G1 X0 Y0 E6.4
; nelox:end
G1 E4.4 F2400
M104 S0
