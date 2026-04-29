#pragma once

/**
 * Phases de la machine d'état d'approche qui s'intercale avant l'appel
 * à l'action /dock du Create 3. Elle étend la portée utile du dock
 * bien au-delà des ~30-50cm des IR natifs.
 *
 *   - IDLE         : pas d'approche en cours.
 *   - COARSE       : on se dirige vers la pose odométrique sauvegardée au dernier
 *                  undock/dock, en open-loop sur l'odométrie.
 *   - FINE         : marqueur ArUco visible, asservissement visuel en cap + distance.
 *   - SEARCH       : rotation sur place à la recherche du marqueur.
 *   - DOCK_PENDING : l'action /dock native est en cours, les IR prennent la main.
 */
enum class ApproachPhase : uint8_t
{
    IDLE,
    COARSE,
    FINE,
    SEARCH,
    DOCK_PENDING,
};

inline const char* approachPhaseToString(const ApproachPhase p) noexcept
{
    switch (p) {
        case ApproachPhase::IDLE:         return "IDLE";
        case ApproachPhase::COARSE:       return "COARSE";
        case ApproachPhase::FINE:         return "FINE";
        case ApproachPhase::SEARCH:       return "SEARCH";
        case ApproachPhase::DOCK_PENDING: return "DOCK_PENDING";
    }
    return "UNKNOWN";
}