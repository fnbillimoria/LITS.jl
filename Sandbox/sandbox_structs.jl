using LITS
using PowerSystems
using Sundials
using Plots
const PSY = PowerSystems

"""
Parameters of an inner loop controller using a PI voltage controller and a PI current controller.

#Arguments
- `ω0::Float64`: Reference frequency in per unit.
"""

mutable struct NoPLL <: PSY.FrequencyEstimator
    ω0::Float64
    n_states::Int64
    states::Vector{Symbol}
    ext::Dict{String, Any}

    function NoPLL(ω0, ext=Dict{String, Any}(), )
        new(ω0, 0, Vector{Symbol}(), ext)
    end

end

get_ω0(value::NoPLL) = value.ω0
PSY.get_states(value::NoPLL) = value.states
PSY.get_n_states(value::NoPLL) = value.n_states
PSY.get_ext(value::NoPLL) = value.ext;


"""
Parameters of an inner loop controller using a PI voltage controller and a PI current controller.

#Arguments
- `kpv::Float64`: voltage controller proportional gain
- `kiv::Float64`: voltage controller integral gain
- `kffi::Float64`: Binary variable to enable feed-forward gain of current
- `kpc::Float64`: current controller proportional gain, validation range
- `kic::Float64`: current controller integral gain, validation range
- `kffv::Float64`: Binary variable to enable feed-forward gain of voltage
"""

mutable struct PI_VoltageCurrent <: PSY.VSControl
    kpv::Float64
    kiv::Float64
    kffi::Float64
    kpc::Float64
    kic::Float64
    kffv::Float64
    n_states::Int64
    states::Vector{Symbol}
    ext::Dict{String, Any}

    function PI_VoltageCurrent(kpv, kiv, kffi, kpc, kic, kffv, ext=Dict{String, Any}(), )

        states = [:ξ_d, :ξ_q, :γ_d, :γ_q]
        n_states = 4

        new(kpv, kiv, kffi, kpc, kic, kffv, n_states, states, ext)
    end
end

PSY.get_kpv(value::PI_VoltageCurrent) = value.kpv
PSY.get_kiv(value::PI_VoltageCurrent) = value.kiv
PSY.get_kffi(value::PI_VoltageCurrent) = value.kffi
PSY.get_kpc(value::PI_VoltageCurrent) = value.kpc
PSY.get_kic(value::PI_VoltageCurrent) = value.kic
PSY.get_kffv(value::PI_VoltageCurrent) = value.kffv
PSY.get_states(value::PI_VoltageCurrent) = value.states
PSY.get_n_states(value::PI_VoltageCurrent) = value.n_states
PSY.get_ext(value::PI_VoltageCurrent) = value.ext;


"""
Parameters of an Active Power Droop Controller

# Arguments
- `d_ω::Float64`: Frequency-Power droop gain
"""

mutable struct PowerDroop <: PSY.ActivePowerControl
    d_ω::Float64
    n_states::Int64
    states::Vector{Symbol}
    ext::Dict{String, Any}

    function PowerDroop(d_ω, ext=Dict{String, Any}(),)

        new(d_ω, 1, [:δθ_vsm], ext) #TODO: I Used δθ_vsm for port issues. Replace with proper symbol.
    end
end

get_d_ω(value::PowerDroop) = value.d_ω
PSY.get_states(value::PowerDroop) = value.states
PSY.get_n_states(value::PowerDroop) = value.n_states
PSY.get_ext(value::PowerDroop) = value.ext;

"""
Parameters of a Virtual PI AVR.

# Arguments
- `kp_avr::Float64`: Proportional gain for AVR
- `ki_avr::Float64`: Integral gain for AVR
"""

mutable struct Virtual_PI_AVR <: PSY.ReactivePowerControl
    kp_avr::Float64
    ki_avr::Float64
    n_states::Int64
    states::Vector{Symbol}
    ext::Dict{String, Any}

    function Virtual_PI_AVR(kp_avr, ki_avr, ext=Dict{String, Any}(), )

        new(kp_avr, ki_avr, 1, [:x_avr], ext)
    end
end

get_kp_avr(value::Virtual_PI_AVR) = value.kp_avr
get_ki_avr(value::Virtual_PI_AVR) = value.ki_avr
PSY.get_states(value::Virtual_PI_AVR) = value.states
PSY.get_n_states(value::Virtual_PI_AVR) = value.n_states
PSY.get_ext(value::Virtual_PI_AVR) = value.ext;
