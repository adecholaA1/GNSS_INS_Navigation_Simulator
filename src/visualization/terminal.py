"""
terminal.py

Affichage terminal professionnel pour le simulateur GNSS/INS.
"""


def print_banner():
    print()
    print("╔" + "═" * 68 + "╗")
    print("║" + " " * 68 + "║")
    print("║        GNSS / INS Navigation Simulator v1.0                  ║")
    print("║" + " " * 68 + "║")
    print("║        Multi-GNSS • RAIM/FDE • Kalman • INS • Fusion         ║")
    print("║" + " " * 68 + "║")
    print("╚" + "═" * 68 + "╝")
    print()


def print_step(message):
    print(f"[✓] {message}")


def print_section(title):
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def print_final_summary(
    gnss,
    raim,
    kalman,
    ins,
    fusion,
):
    print_section("Résumé final")

    print(f"Configuration GNSS      : {gnss['configuration']}")
    print(f"Nombre de satellites    : {gnss['satellites'].shape[0]}")
    print(f"PDOP moyen              : {gnss['dop_mean']['PDOP']:.3f}")
    print(f"HDOP moyen              : {gnss['dop_mean']['HDOP']:.3f}")
    print(f"VDOP moyen              : {gnss['dop_mean']['VDOP']:.3f}")

    print()
    print("GNSS")
    print("-" * 72)
    print(f"Gauss-Newton nominal    : {gnss['rmse']:.3f} m")
    print(f"Sans RAIM               : {raim['rmse_raw']:.3f} m")
    print(f"Avec RAIM/FDE           : {raim['rmse_raim']:.3f} m")
    print(f"Kalman GNSS             : {kalman['rmse']:.3f} m")

    print()
    print("RAIM/FDE")
    print("-" * 72)
    print(f"Détections              : {raim['n_detections']}")
    print(f"Satellite fautif        : {raim['fault_satellite'] + 1}")

    if raim["most_common_excluded"] is not None:
        print(f"Satellite exclu         : {raim['most_common_excluded'] + 1}")
    else:
        print("Satellite exclu         : Aucun")

    print()
    print("INS / Fusion")
    print("-" * 72)

    for scenario_name in ins:
        print()
        print(f"Scénario                : {ins[scenario_name]['label']}")
        print(f"INS Strapdown           : {ins[scenario_name]['rmse']:.3f} m")
        print(f"Fusion GNSS/INS         : {fusion[scenario_name]['rmse']:.3f} m")