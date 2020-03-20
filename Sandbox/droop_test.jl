using Revise
using LITS
import PowerSystems
using Sundials
const PSY = PowerSystems

## System
nodes = [
    PSY.Bus(
        1, #number
        "Bus 1", #Name
        "REF", #BusType (REF, PV, PQ)
        0, #Angle in radians
        1.05, #Voltage in pu
        (min = 0.94, max = 1.06), #Voltage limits in pu
        69, #Base voltage in kV
        nothing,
        nothing,
    ),
    PSY.Bus(2, "Bus 2", "PV", 0, 1.0, (min = 0.94, max = 1.06), 69, nothing, nothing),
]

branches = [PSY.Line(
    "Line1", #name
    true, #available
    0.0, #active power flow initial condition (from-to)
    0.0, #reactive power flow initial condition (from-to)
    PSY.Arc(from = nodes[1], to = nodes[2]), #Connection between buses
    0.01, #resistance in pu
    0.05, #reactance in pu
    (from = 0.0, to = 0.0), #susceptance in pu
    18.046, #rate in MW
    1.04,
)]  #angle limits (-min and max)

branches_fault = [PSY.Line(
    "Line1", #name
    true, #available
    0.0, #active power flow initial condition (from-to)
    0.0, #reactive power flow initial condition (from-to)
    PSY.Arc(from = nodes[1], to = nodes[2]), #Connection between buses
    0.02, #resistance in pu
    0.1, #reactance in pu
    (from = 0.0, to = 0.0), #susceptance in pu
    18.046, #rate in MW
    1.04,
)]

inf_source = PSY.Source(
    "InfBus", #name
    true, #availability
    nodes[1], #bus
    1.05, #VR
    0.0, #VI
    0.000005,
) #Xth

## Inverter components
nopll  = NoPLL(1.0)
innerloop = PI_VoltageCurrent(
    0.59, #kpv
    232.0, #kiv
    0.0, #kffi
    0.73, #kpc
    0.0059, #kic
    0.0) #kffv
pdroop = PowerDroop(2.0) #d_ω
virtual_avr = Virtual_PI_AVR(
    0.001, #kp_avr
    0.05) #ki_avr
outerloop = PSY.VirtualInertiaQdroop(pdroop, virtual_avr)
filter_test = PSY.LCLFilter(
    0.08, #Series inductance lf in pu
    0.003, #Series resitance rf in pu
    0.074, #Shunt capacitance cf in pu
    0.2, #Series ractance rg to grid connection (#Step up transformer or similar)
    0.01,
) #Series resistance lg to grid connection (#Step up transformer or similar)
dc_source = PSY.FixedDCSource(2440.0)
conv_droop = PSY.AvgCnvFixedDC(
    2440.0, #Rated Voltage
    100.0,
) #Rated MVA

inv_droop = PSY.DynamicInverter(
        1, #Number
        "inv_droop", #name
        nodes[2], #bus
        1.0, # ω_ref,
        1.00, #V_ref
        0.5, #P_ref
        0.0, #Q_ref
        100.0, #MVABase
        conv_droop, #converter
        outerloop, #outer control
        innerloop, #inner control voltage source
        dc_source, #dc source
        nopll, #pll
        filter_test,
    ) #filter

sys = PSY.System(100.0, frequency = 50.0)
for bus in nodes
    PSY.add_component!(sys, bus)
end
for lines in branches
    PSY.add_component!(sys, lines)
end
PSY.add_component!(sys, inf_source)
PSY.add_component!(sys, inv_droop)

Ybus_fault = PSY.Ybus(branches_fault, nodes)[:,:]
Ybus_change = ThreePhaseFault(
    1.0, #change at t = 1.0
    Ybus_fault,
) #New YBus

tspan = (0.0, 30.0)
x0_guess = [
    1.0, #V1_R
    1.0, #V2_R
    0.0, #V1_I
    0.0, #V2_I
    0.1, #θ
    0.0, #x_avr
    0.0015, #ξ_d
    -0.08, #ξ_q
    0.05, #γ_d
    -0.001, #γ_q
    0.5, #id_cnv
    0.0, #iq_cnv
    0.95, #Vd_o
    -0.1, #Vq_o
    0.49, #Id_o
    -0.1, #Iq_o
    ]

sim = LITS.Simulation(sys, tspan, Ybus_change, initial_guess = x0_guess)
sim.x0_init
LITS.print_init_states(sim)
small_sig = small_signal_analysis(sim)
