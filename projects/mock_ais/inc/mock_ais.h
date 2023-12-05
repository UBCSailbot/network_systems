#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <cstdint>
#include <mutex>
#include <random>

using Vec2DFloat = std::array<float, 2>;
namespace defaults
{
constexpr float             MAX_HEADING_CHANGE = 5.0;
constexpr float             MAX_SPEED_CHANGE   = 1.0;
constexpr float             MIN_AIS_SHIP_DIST  = 0.001;  // 111m at equator
constexpr float             MAX_AIS_SHIP_DIST  = 0.1;    // 11.1km at equator
static constexpr int        UPDATE_RATE_MS     = 500;
static constexpr int        SEED               = 123456;
static constexpr int        NUM_SIM_SHIPS      = 20;
static constexpr Vec2DFloat POLARIS_START_POS{
  49.28397458822112, -123.6525841364974};  // some point in the Strait of Georgia;
}  // namespace defaults

struct AisShip
{
    Vec2DFloat lat_lon_;
    float      speed_;
    float      heading_;
    uint32_t   id_;
};

struct SimShipConfig
{
    float max_heading_change_;
    float max_speed_change_;
    float max_ship_dist_;
    float min_ship_dist_;
};

class MockAisShip : public AisShip
{
public:
    MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat polaris_lat_lon, SimShipConfig config);
    void tick(const Vec2DFloat & polaris_lat_lon);

private:
    std::mt19937  mt_rng_;
    SimShipConfig config_;
};

class MockAis
{
public:
    MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon);
    MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon, SimShipConfig config);
    std::vector<AisShip> ships();
    void                 updatePolarisPos(const Vec2DFloat & lat_lon);
    void                 tick();

private:
    Vec2DFloat               polaris_lat_lon_;
    std::vector<MockAisShip> ships_;
};
