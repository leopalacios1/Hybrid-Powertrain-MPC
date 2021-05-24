global capacity

% BATTERY
bat_nom_V  = 40    ;   % Nominal voltage.                [V]
capacity   = 2.2   ;   % Rated capacity.                 [Ah]
bat_tau    = 30    ;   % Response time.                  [s]

% FUEL CELL
v_0A       = 18    ;   % Voltage at 0A.                  [V]
v_1A       = 15    ;   % Voltage at 1A.                  [V]
Inom       = 4     ;   % Nominal operating current.      [A]
Vnom       = 12.5  ;   % Nominal operating voltage.      [V]
Iend       = 5     ;   % Maximum operating current.      [A]
Vend       = 12    ;   % Maximum operating voltage.      [V]
nCells     = 20    ;   % Number of cells.                [#]
stackEff   = 46    ;   % Nominal Stack efficiency.       [%]
opTemp     = 25    ;   % Operating tempreature.          [C]
nomAir     = 20    ;   % Nominal air flow rate.          [lpm]
nomFuelP   = 0.35  ;   % Nominal fuel supply pressure.   [bar]
nomAirP    = 0.3   ;   % Nominal air supply pressure.    [bar]
nomH2comp  = 99.95 ;   % Nominal composition H2 in fuel. [%]
nomO2comp  = 21    ;   % Nominal composition O2 in air.  [%]
nomH20comp = 1     ;   % Nominal composition H20 in air. [%] 
