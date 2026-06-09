; Processed by Nelox Belt v0.4.0
;   gantry angle = 45.0 deg, belt_axis = z, scale_z = True, scale_feedrate = True, scale_extrusion = True
;   transform: X'=x  Y'=z/sin(a)  Z'=y+z*cot(a)
; Sample upright-sliced G-code (a tiny 2-layer square) for testing Nelox Belt.
; Slice this kind of file in OrcaSlicer with a flat-bed profile, then run it
; through nelox_belt.py to produce belt-printer G-code.
G90
M82
; nelox:begin
G1 Y0.2828 Z0.2 F1039.2
G1 X0 Z0.2 F3000
G1 X20 Z0.2 E0.8 F1200
G1 X20 Z20.2 E1.6
G1 X0 Z20.2 E2.4
G1 X0 Z0.2 E3.2
G1 Y0.5657 Z0.4 F1039.2
G1 X20 Z0.4 E4.0 F1200
G1 X20 Z20.4 E4.8
G1 X0 Z20.4 E5.6
G1 X0 Z0.4 E6.4
; nelox:end
G1 E4.4 F2400
M104 S0
