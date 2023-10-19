#pragma once

#include <vector>

#include "ais_ship.h"

class AisTransceiver
{
public:
    virtual std::vector<AisShip> ships() = 0;
    void                         updatePolarisPos(Vec2DFloat lat_lon){/* Intentionally left blank */};
};
