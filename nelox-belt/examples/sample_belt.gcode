; Processed by Nelox Belt v0.1.0
;   gantry angle = 45.0 deg, scale_z = True, scale_feedrate = True
;   transform: X'=x  Y'=y+z*cot(a)  Z'=z/sin(a)
; Sample upright-sliced G-code (a tiny 2-layer square) for testing Nelox Belt.
; Slice this kind of file in OrcaSlicer with a flat-bed profile, then run it
; through nelox_belt.py to produce belt-printer G-code.
G90
M82
; nelox:begin
G1 Y0.2 Z0.2828 F1039.2
G1 X0 Y0.2 F3000
G1 X20 Y0.2 E0.8 F1200
G1 X20 Y20.2 E1.6
G1 X0 Y20.2 E2.4
G1 X0 Y0.2 E3.2
G1 Y0.4 Z0.5657 F1039.2
G1 X20 Y0.4 E4.0 F1200
G1 X20 Y20.4 E4.8
G1 X0 Y20.4 E5.6
G1 X0 Y0.4 E6.4
; nelox:end
G1 E4.4 F2400
M104 S0
