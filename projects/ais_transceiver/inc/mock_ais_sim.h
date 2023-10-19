#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <cstdint>
#include <mutex>
#include <random>

#include "ais_ship.h"
#include "ais_transceiver.h"

constexpr float MAX_HEADING_CHANGE = 5.0;
constexpr float MAX_SPEED_CHANGE   = 1.0;
constexpr float MIN_AIS_SHIP_DIST  = 0.001;  // 111m at equator
constexpr float MAX_AIS_SHIP_DIST  = 0.1;    // 11.1km at equator

class MockAisShip : public AisShip
{
public:
    MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat curr_lat_lon);
    void tick(const Vec2DFloat & curr_lat_lon);

private:
    std::mt19937 mt_rng_;
};

class MockAisSim : public AisTransceiver
{
public:
    MockAisSim(
      uint32_t seed, uint32_t num_ships, uint32_t tick_rate_ms, Vec2DFloat polaris_lat_lon,
      boost::asio::io_service & io);
    std::vector<AisShip> ships();
    void                 updatePolarisPos(const Vec2DFloat & lat_lon);

private:
    Vec2DFloat                  polaris_lat_lon_;
    std::vector<MockAisShip>    ships_;
    uint32_t                    tick_rate_ms_;
    boost::asio::deadline_timer timer_;
    std::mutex                  mtx_;

    void simTick();
};
