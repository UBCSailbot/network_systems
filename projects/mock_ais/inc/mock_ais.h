#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <cstdint>
#include <mutex>
#include <random>

constexpr float MAX_HEADING_CHANGE = 5.0;
constexpr float MAX_SPEED_CHANGE   = 1.0;
constexpr float MIN_AIS_SHIP_DIST  = 0.001;  // 111m at equator
constexpr float MAX_AIS_SHIP_DIST  = 0.1;    // 11.1km at equator

using Vec2DFloat = std::array<float, 2>;

struct AisShip
{
    Vec2DFloat lat_lon_;
    float      speed_;
    float      heading_;
    uint32_t   id_;
};

class MockAisShip : public AisShip
{
public:
    MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat polaris_lat_lon);
    void tick(const Vec2DFloat & polaris_lat_lon);

private:
    std::mt19937 mt_rng_;
};

class MockAis
{
public:
    MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon);
    std::vector<AisShip> ships();
    void                 updatePolarisPos(const Vec2DFloat & lat_lon);
    void                 tick();

private:
    Vec2DFloat               polaris_lat_lon_;
    std::vector<MockAisShip> ships_;
};
