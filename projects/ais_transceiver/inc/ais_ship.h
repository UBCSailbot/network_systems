#pragma once

#include <array>
#include <cstdint>

using Vec2DFloat = std::array<float, 2>;

struct AisShip
{
    Vec2DFloat lat_lon_;
    float      speed_;
    float      heading_;
    uint32_t   id_;
};
