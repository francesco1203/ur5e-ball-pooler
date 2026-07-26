import numpy as np


def convert_meshlab_data(
    name: str, V_mm3: float, CoM_mm: list, I_meshlab: list, rho_kg_m3: float
):
    """Converte le misurazioni grezze di MeshLab nelle unità del Sistema Internazionale (SI).

    Parameters:
    -----------
    name : str
        Nome del componente
    V_mm3 : float
        Volume restituito da MeshLab (mm^3)
    CoM_mm : list or np.ndarray
        Centro di massa da MeshLab [x, y, z] (mm)
    I_meshlab : list or np.ndarray (3x3)
        Tensore d'inerzia geometrico da MeshLab (mm^5)
    rho_kg_m3 : float
        Densità del materiale (kg/m^3)
    """
    # Conversione array numpy
    CoM_mm_arr = np.array(CoM_mm, dtype=float)
    I_meshlab_arr = np.array(I_meshlab, dtype=float)

    # 1. Volume [m^3]
    V_m3 = V_mm3 * 1e-9

    # 2. Massa [kg]
    m_kg = rho_kg_m3 * V_m3

    # 3. Centro di Massa [m]
    CoM_m = CoM_mm_arr * 1e-3

    # 4. Tensore d'Inerzia [kg * m^2]
    # In Meshlab il tensore d'inerzia unitario ha unità mm^5.
    # Scalatura: 1e-15 * rho converte mm^5 * (kg/m^3) in kg*m^2.
    I_kg_m2 = 1e-15 * rho_kg_m3 * I_meshlab_arr

    return {
        "name": name,
        "volume_m3": V_m3,
        "massa_kg": m_kg,
        "CoM_m": CoM_m,
        "I_kg_m2": I_kg_m2,
    }


def print_scaled_component(data: dict):
    """Stampa formattata dei dati scalati"""
    np.set_printoptions(suppress=True, formatter={"float_kind": "{:.10f}".format})

    print("=" * 65)
    print(f"COMPONENTE: {data['name'].upper()}")
    print("=" * 65)
    print(f"Volume:           {data['volume_m3']:.8e} m^3")
    print(
        f"Massa:            {data['massa_kg']:.6f} kg  ({data['massa_kg']*1000:.2f} g)"
    )
    print(
        f"Centro di Massa:  [{data['CoM_m'][0]:.6f}, {data['CoM_m'][1]:.6f}, {data['CoM_m'][2]:.6f}] m"
    )
    print("\nTANSORE D'INERZIA SCALATO (kg*m^2):")
    print(data["I_kg_m2"])
    print("\n")


# =====================================================================
# DATI ESTRATTI DA INERTIAL.TXT E APPLICAZIONE DELLE DENSITÀ
# =====================================================================

# Densità
RHO_PLA = 1240.0  # kg/m^3                                          (1.24 g/cm^3 da Gemini!!!!)
RHO_ROD = 720.0  # kg/m^3                                           (calcolata da peso e volume su asta intera)
RHO_BILLIARD = 1.370 / (9517831.000000 * 1e-9)  # ~143.9 kg/m^3     (calcolata da massa e volume)
RHO_WHITE_BALL = 0.010 / (8176.56 * 1e-9) # 1223.0 kg/m^3           (calcolata da massa e volume)
RHO_COLOURED_BALL = 0.015 / (8176.56 * 1e-9) # 1834.5 kg/m^3         (calcolata da massa e volume)

# 1. HOLDER (CAD senza asta)
holder = convert_meshlab_data(
    name="Holder (CAD)",
    V_mm3=319049.69,
    CoM_mm=[-0.04, -2.91, 47.88],
    I_meshlab=[
        [570366144.000000, -102764.695312, -146555.562500],
        [-102764.695312, 532865760.000000, -58696496.000000],
        [-146555.562500, -58696496.000000, 189585216.000000],
    ],
    rho_kg_m3=RHO_PLA,
)

# 2. CUT ROD (asta tagliata)
cut_rod = convert_meshlab_data(
    name="Cut Rod",
    V_mm3=22210.00,
    CoM_mm=[-41.20, 0.0, 106.92],
    I_meshlab=[
        [426887.562500, 54.580643, 42580.738281],
        [54.580643, 67643048.000000, 6.285519],
        [42580.738281, 6.285519, 67646144.000000],
    ],
    rho_kg_m3=RHO_ROD,
)

# 3. BILLIARD (biliardino)
billiard = convert_meshlab_data(
    name="Billiard",
    V_mm3=9517831.000000,
    CoM_mm=[0.0, 0.0, 31.672869],
    I_meshlab=[
        [78749474816.000000, -307.027863, 0.498290],
        [-307.027863, 208509730816.000000, 0.605656],
        [0.498290, 0.605656, 280543821824.000000],
    ],
    rho_kg_m3=RHO_BILLIARD,
)

# 4. WHITE BALL (pallina bianca)
white_ball = convert_meshlab_data(
    name="white_ball",
    V_mm3= 8176.56,
    CoM_mm=[0.0, 0.0, 0.0],
    I_meshlab=[
        [510860, 0.0000, 0.0000],
        [0.0000, 510860, 0.0000],
        [0.0000, 0.0000, 510860],
    ],
    rho_kg_m3=RHO_WHITE_BALL,
)

# 5. COLORED BALL (palline colorate)
coloured_ball = convert_meshlab_data(
    name="coloured_ball",
    V_mm3= 8176.56,
    CoM_mm=[0.0, 0.0, 0.0],
    I_meshlab=[
        [510860, 0.0000, 0.0000],
        [0.0000, 510860, 0.0000],
        [0.0000, 0.0000, 510860],
    ],
    rho_kg_m3=RHO_COLOURED_BALL,
)

# =====================================================================
# ESCUZIONE E STAMPA
# =====================================================================
if __name__ == "__main__":
    print_scaled_component(holder)
    print_scaled_component(cut_rod)
    print_scaled_component(billiard)
    print_scaled_component(white_ball)
    print_scaled_component(coloured_ball)
