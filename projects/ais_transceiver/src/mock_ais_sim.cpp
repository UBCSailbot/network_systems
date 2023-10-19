#include "mock_ais_sim.h"

#include <array>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/system/error_code.hpp>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <random>

#include "ais_ship.h"
#include "shared_constants.h"

static float degToRad(const float & degrees)
{
    return static_cast<float>(degrees * M_PI / 180.0);  // NOLINT(readability-magic-numbers)
}

static float boundHeading(const float & heading)
{
    if (heading < HEADING_LBND) {
        return heading + HEADING_UBND;
    }
    if (heading >= HEADING_UBND) {
        return heading - HEADING_UBND;
    }
    return heading;
}

static Vec2DFloat headingToVec2D(const float & heading)
{
    float angle = degToRad(heading);
    // Since 0 is north (y-axis), use sin for x and cos for y
    return {std::sin(angle), std::cos(angle)};
}

MockAisShip::MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat curr_lat_lon) : mt_rng_(seed)
{
    static const std::array<float, 2>       lat_lon_or_neg = {-1.0, 1.0};
    std::uniform_real_distribution<float>   lat_dist(MIN_AIS_SHIP_DIST, MAX_AIS_SHIP_DIST);
    std::uniform_real_distribution<float>   lon_dist(MIN_AIS_SHIP_DIST, MAX_AIS_SHIP_DIST);
    std::uniform_real_distribution<float>   speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float>   heading_dist(HEADING_LBND, HEADING_UBND);
    std::uniform_int_distribution<uint32_t> lat_lon_or_neg_dist(0, 1);

    id_      = id;
    lat_lon_ = {
      curr_lat_lon[0] + lat_lon_or_neg[lat_lon_or_neg_dist(mt_rng_)] * lat_dist(mt_rng_),
      curr_lat_lon[1] + lat_lon_or_neg[lat_lon_or_neg_dist(mt_rng_)] * lon_dist(mt_rng_)};
    speed_   = speed_dist(mt_rng_);
    heading_ = heading_dist(mt_rng_);
}

void MockAisShip::tick(const Vec2DFloat & curr_lat_lon)
{
    std::uniform_real_distribution<float> heading_dist(heading_ - MAX_HEADING_CHANGE, heading_ + MAX_HEADING_CHANGE);
    std::uniform_real_distribution<float> speed_dist(speed_ - MAX_SPEED_CHANGE, speed_ + MAX_SPEED_CHANGE);

    float speed = speed_dist(mt_rng_);
    if (speed > SPEED_UBND) {
        speed = SPEED_UBND;
    } else if (speed < SPEED_LBND) {
        speed = SPEED_LBND;
    }

    heading_           = boundHeading(heading_dist(mt_rng_));
    speed_             = speed;
    Vec2DFloat dir_vec = headingToVec2D(heading_);
    if (
      (std::abs(std::abs(lat_lon_[0] + dir_vec[0] * speed_) - std::abs(curr_lat_lon[0])) < MAX_AIS_SHIP_DIST) &&
      (std::abs(std::abs(lat_lon_[0] + dir_vec[0] * speed_) - std::abs(curr_lat_lon[0])) > MIN_AIS_SHIP_DIST)) {
        lat_lon_[0] += dir_vec[0] * speed_;
    }
    if (
      (std::abs(std::abs(lat_lon_[1] + dir_vec[1] * speed_) - std::abs(curr_lat_lon[1])) < MAX_AIS_SHIP_DIST) &&
      (std::abs(std::abs(lat_lon_[1] + dir_vec[1] * speed_) - std::abs(curr_lat_lon[1])) > MIN_AIS_SHIP_DIST)) {
        lat_lon_[1] += dir_vec[1] * speed_;
    }
}

MockAisSim::MockAisSim(
  uint32_t seed, uint32_t num_ships, uint32_t tick_rate_ms, Vec2DFloat polaris_lat_lon, boost::asio::io_service & io)
: polaris_lat_lon_(polaris_lat_lon), tick_rate_ms_(tick_rate_ms), timer_(boost::asio::deadline_timer(io))
{
    for (uint32_t i = 0; i < num_ships; i++) {
        ships_.push_back(MockAisShip(seed + i, i, polaris_lat_lon));
    }
    simTick();
}

std::vector<AisShip> MockAisSim::ships()
{
    std::lock_guard<std::mutex> guard(mtx_);
    std::vector<AisShip>        ships;
    for (const MockAisShip & ship : ships_) {
        ships.push_back({.lat_lon_ = ship.lat_lon_, .speed_ = ship.speed_, .heading_ = ship.heading_, .id_ = ship.id_});
    }
    return ships;
}

void MockAisSim::updatePolarisPos(const Vec2DFloat & lat_lon)
{
    std::lock_guard<std::mutex> guard(mtx_);
    polaris_lat_lon_ = lat_lon;
}

void MockAisSim::simTick()
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (MockAisShip & ship : ships_) {
        ship.tick(polaris_lat_lon_);
    }
    timer_.expires_from_now(boost::posix_time::milliseconds(tick_rate_ms_));
    timer_.async_wait([&](const boost::system::error_code & /* e */) { simTick(); });
}
