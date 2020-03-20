##NoPLL
function LITS.mdl_freq_estimator_ode!(
    device_states,
    output_ode,
    f0,
    ω_sys,
    device::PSY.DynamicInverter{C, O, VC, DC, NoPLL, F},
) where {
    C <: PSY.Converter,
    O <: PSY.OuterControl,
    VC <: PSY.VSControl,
    DC <: PSY.DCSource,
    F <: PSY.Filter,
}
    pll_control = PSY.get_freq_estimator(device)
    ω0 = get_ω0(pll_control)

end

## Inner Control
function LITS.mdl_VScontrol_ode!(
    device_states,
    output_ode,
    device::PSY.DynamicInverter{C, O, PI_VoltageCurrent, DC, P, F},
) where {
    C <: PSY.Converter,
    O <: PSY.OuterControl,
    DC <: PSY.DCSource,
    P <: PSY.FrequencyEstimator,
    F <: PSY.Filter,
}
    #Obtain external states for component
    external_ix = LITS.get_input_port_ix(device, PI_VoltageCurrent)
    Id_o = device_states[external_ix[1]]
    Iq_o = device_states[external_ix[2]]
    Id_cnv = device_states[external_ix[3]]
    Iq_cnv = device_states[external_ix[4]]
    Vd_o = device_states[external_ix[5]]
    Vq_o = device_states[external_ix[6]]

    #Obtain inner variables for component
    ω_olc = LITS.get_inner_vars(device)[LITS.ω_control_var]
    V_olc_ref = LITS.get_inner_vars(device)[LITS.v_control_var]
    Vdc = LITS.get_inner_vars(device)[LITS.Vdc_var]

    #Get Voltage Controller parameters
    vscontrol = PSY.get_vscontrol(device)
    filter = PSY.get_filter(device)
    kpv = PSY.get_kpv(vscontrol)
    kiv = PSY.get_kiv(vscontrol)
    kffv = PSY.get_kffv(vscontrol)
    cf = PSY.get_cf(filter)

    #Get Current Controller Parameters
    kpc = PSY.get_kpc(vscontrol)
    kic = PSY.get_kic(vscontrol)
    kffi = PSY.get_kffi(vscontrol)
    lf = PSY.get_lf(filter)
    #rf = PSY.get_rf(filter) We can generalize if the filter have R or not.

    #Obtain indices for component w/r to device
    local_ix = LITS.get_local_state_ix(device, PI_VoltageCurrent)

    #Define internal states for inner control
    internal_states = @view device_states[local_ix]
    ξ_d = internal_states[1]
    ξ_q = internal_states[2]
    γ_d = internal_states[3]
    γ_q = internal_states[4]

    #Inputs to Voltage Controller (control signals)
    Vd_ref = V_olc_ref #Create Inner var for references in both d and q.
    Vq_ref = 0 #TODO: Create Inner var for references in both d and q.

    #Voltage Control ODES
    #PI Integrator (internal state)
    output_ode[local_ix[1]] = (Vd_ref - Vd_o)
    output_ode[local_ix[2]] = (Vq_ref - Vq_o)

    #Inputs to Current Controller (control signals)
    Id_vc_ref = kpv * (Vd_ref - Vd_o) + kiv * ξ_d + kffi * (Id_o - cf * ω_olc * Vq_o)
    Iq_vc_ref = kpv * (Vq_ref - Vq_o) + kiv * ξ_q + kffi * (Iq_o + cf * ω_olc * Vd_o)

    #Current Control ODEs
    #PI Integrator (internal state)
    output_ode[local_ix[3]] = Id_vc_ref - Id_cnv
    output_ode[local_ix[4]] = Iq_vc_ref - Iq_cnv

    #Voltage reference to converter
    Vd_cc_ref = kpc * (Id_vc_ref - Id_cnv) + kic * γ_d + kffv * (Vd_o - lf * ω_olc * Iq_cnv)
    Vq_cc_ref = kpc * (Iq_vc_ref - Iq_cnv) + kic * γ_q + kffv * (Vq_o + lf * ω_olc * Id_cnv)

    #Update inner_vars
    LITS.get_inner_vars(device)[LITS.md_var] = Vd_cc_ref / Vdc
    LITS.get_inner_vars(device)[LITS.mq_var] = Vq_cc_ref / Vdc
end

## Outer Loop
function LITS.mdl_outer_ode!(
    device_states,
    output_ode,
    f0,
    ω_sys,
    device::PSY.DynamicInverter{
        C,
        PSY.VirtualInertiaQdroop{PowerDroop, Virtual_PI_AVR},
        VC,
        DC,
        P,
        F,
    },
) where {
    C <: PSY.Converter,
    VC <: PSY.VSControl,
    DC <: PSY.DCSource,
    P <: PSY.FrequencyEstimator,
    F <: PSY.Filter,
}

    #Obtain external states inputs for component
    external_ix = LITS.get_input_port_ix(
        device,
        PSY.VirtualInertiaQdroop{PowerDroop, Virtual_PI_AVR},
    )
    Vd_o = device_states[external_ix[1]]
    Vq_o = device_states[external_ix[2]]
    Id_o = device_states[external_ix[3]]
    Iq_o = device_states[external_ix[4]]
    #Problem when some states are not defined? Since Outer has a lot of state_ports that may not exist.

    #Get Active Power Controller parameters
    outer_control = PSY.get_outercontrol(device)
    active_power_control = PSY.get_active_power(outer_control)
    d_ω = get_d_ω(active_power_control)

    #Get Reactive Power Controller parameters
    reactive_power_control = PSY.get_reactive_power(outer_control)
    kp_avr = get_kp_avr(reactive_power_control)
    ki_avr = get_ki_avr(reactive_power_control)

    #Obtain external parameters
    p_ref = PSY.get_ext(device)[LITS.CONTROL_REFS][LITS.P_ref_index]
    ω_ref = PSY.get_ext(device)[LITS.CONTROL_REFS][LITS.ω_ref_index]
    V_ref = PSY.get_ext(device)[LITS.CONTROL_REFS][LITS.V_ref_index]

    #Obtain inner vars
    V_tR = LITS.get_inner_vars(device)[LITS.VR_inv_var]
    V_tI = LITS.get_inner_vars(device)[LITS.VI_inv_var]

    #Compute electric variables
    Pe = (Vd_o * Id_o + Vq_o * Iq_o) #No Resistance in output.
    #Vo_mag = sqrt(V_tR^2 + V_tI^2)
    Vo_mag = sqrt(Vd_o^2 + Vq_o^2)
    ω_olc = ω_ref + d_ω * (p_ref - Pe)

    #Obtain indices for component w/r to device
    local_ix = LITS.get_local_state_ix(
        device,
        PSY.VirtualInertiaQdroop{PowerDroop, Virtual_PI_AVR},
    )

    #Define internal states for outer loop control
    internal_states = @view device_states[local_ix]
    θ_olc = internal_states[1]
    x_avr = internal_states[2]

    #Compute ODEs for Active Power Droop
    output_ode[local_ix[1]] = 2 * π * f0 * (ω_olc - ω_sys)

    #Compute ODEs for Virtual AVR
    output_ode[local_ix[2]] = V_ref - Vo_mag

    #Update Inner Vars
    LITS.get_inner_vars(device)[LITS.ω_control_var] = ω_olc
    LITS.get_inner_vars(device)[LITS.v_control_var] = kp_avr * (V_ref - Vo_mag) + ki_avr * x_avr
end
