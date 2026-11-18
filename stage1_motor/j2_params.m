function p = j2_params()
% J2_PARAMS  Joint-2 actuator parameters (stage 1: motor + current loop).
%   Source of every value: 관절2_기준사양_결정.md (2026-11-16)
%   Motor : maxon EC-i 52 180 W, 24 V winding #574741 (catalog, block commutation)
%   Gear  : Harmonic Drive CSG-17-100-2A-R-BB-LW (product page)
%   Drive : maxon EPOS4 50/15 class, 24 V bus (assumption)
%   [M] = manufacturer value, [C] = converted/calculated, [A] = modelling assumption

% ---------------- motor (dq, amplitude-invariant Park) ----------------
kM_B   = 0.0481;                      % [M] torque constant, block commutation, N*m/A
p.pp   = 8;                           % [M] pole pairs
p.Rs   = 0.0935/2;                    % [C] phase resistance = R(ph-ph)/2, wye [A], 25 C
p.Ld   = 0.123e-3/2;                  % [C] = L(ph-ph)/2, non-salient [A]
p.Lq   = p.Ld;
p.Kt   = pi/(2*sqrt(3))*kM_B;         % [C] sinusoidal torque const (phase-current amplitude)
p.psi  = p.Kt/(1.5*p.pp);             % [C] PM flux linkage, Wb
p.Jm   = 170e-7;                      % [M] rotor inertia, kg*m^2
p.IN   = 2/sqrt(3)*9.03;              % [C] continuous current, amplitude (10.43 A)
p.nmax_rpm = 6000;                    % [M]

% ---------------- drive ----------------
p.Vdc   = 24;                         % [A] Gen3 nominal supply (18 V for sensitivity case)
p.Vmax  = 0.9*p.Vdc/sqrt(3);          % [C] |v_dq| limit, EPOS4 max output 0.9*Vcc, SVPWM linear
p.Imax  = 20.9;                       % [A] current COMMAND limit, case A (2 x IN). Case B: 10.4
wc      = 2*pi*1000;                  % [A] current-loop bandwidth 1 kHz
p.Kp    = wc*p.Lq;                    % [C] V/A   (pole-zero cancellation design)
p.Ki    = wc*p.Rs;                    % [C] V/(A*s)
p.Kaw   = p.Ki/p.Kp;                  % [C] back-calculation anti-windup gain, 1/s

% ---------------- gear (used from stage 2) ----------------
p.N     = 100;                        % [M]
p.JWG   = 0.079e-4;                   % [M] wave-generator inertia (brochure, re-check)
p.s     = -1;                         % [M] CS fixed, FS output -> output opposes input
p.Kg    = 1.4e4;                      % [M] torsional stiffness K2, N*m/rad (K1 1.0e4, K3 1.6e4)
p.zeta_g= 0.05;                       % [A] gear-mode damping ratio (no maker data)
p.cg    = 2*p.zeta_g*sqrt(p.Kg*p.N^2*(p.Jm+p.JWG));   % [C] N*m*s/rad, joint side

% ---------------- stage-1 test fixture ----------------
p.J_test = p.Jm + p.JWG + 0.5/p.N^2;  % [A] motor + WG + 0.5 kg*m^2 joint-side load
end
