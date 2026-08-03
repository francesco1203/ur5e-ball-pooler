import numpy as np


def parallel_axis_theorem(
    I_com: np.ndarray, m: float, R: np.ndarray
) -> np.ndarray:
    """Applica il teorema degli assi paralleli (Steiner) per traslare il tensore d'inerzia.

    Formula: J_ij = I_ij + m * (|R|^2 * delta_ij - R_i * R_j)
    """
    R_sq = np.dot(R, R)
    return I_com + m * (R_sq * np.eye(3) - np.outer(R, R))


# =====================================================================
# DATI SCALATI NEL SISTEMA INTERNAZIONALE (m, kg, kg*m^2)
# =====================================================================

# 1. HOLDER (CAD)
m_holder = 0.395622  # kg
CoM_holder = np.array([-0.000040, -0.002910, 0.047880])  # m
I_holder = np.array(
    [
        [0.0007072540, -0.0000001274, -0.0000001817],
        [-0.0000001274, 0.0006607535, -0.0000727837],
        [-0.0000001817, -0.0000727837, 0.0002350857],
    ]
)

# 2. CUT ROD
m_rod = 0.015991  # kg
CoM_rod = np.array([-0.041200, 0.000000, 0.106920])  # m
I_rod = np.array(
    [
        [0.0000003074, 0.0000000000, 0.0000000307],
        [0.0000000000, 0.0000487030, 0.0000000000],
        [0.0000000307, 0.0000000000, 0.0000487052],
    ]
)


# =====================================================================
# CALCOLO PROPRIETÀ COMPOSITE (HOLDER + CUT ROD)
# =====================================================================

# 1. Massa Totale
m_totale = m_holder + m_rod

# 2. Centro di Massa Globale (in metri)
CoM_globale = (m_holder * CoM_holder + m_rod * CoM_rod) / m_totale

# 3. Vettori Spostamento (in metri) dal nuovo CoM ai CoM locali
R_holder = CoM_holder - CoM_globale
R_rod = CoM_rod - CoM_globale

# 4. Traslazione dei Tensori d'Inerzia al Centro di Massa Globale
J_holder_traslato = parallel_axis_theorem(I_holder, m_holder, R_holder)
J_rod_traslato = parallel_axis_theorem(I_rod, m_rod, R_rod)

# 5. Somma dei Tensori (Inerzia Totale Rispetto al CoM Globale)
I_totale = J_holder_traslato + J_rod_traslato


# =====================================================================
# STAMPA DEI RISULTATI
# =====================================================================

np.set_printoptions(suppress=True, formatter={"float_kind": "{:.10f}".format})

print("=" * 70)
print("PROPRIETÀ FISICHE CALCOLATE PER 'HOLDER WITH CUT ROD'")
print("=" * 70)
print(f"Massa Totale:           {m_totale:.6f} kg  ({m_totale * 1000:.2f} g)")
print("-" * 70)
print(
    "Centro di Massa Globale (m): ",
    np.array2string(CoM_globale, formatter={"float_kind": "{:.6f}".format}),
)
print("=" * 70)
print("\nTANSORE D'INERZIA TOTALE rispetto al CoM Globale (kg*m^2):")
print(I_totale)
