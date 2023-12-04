#include "mock_ais.h"

#include <array>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <random>

#include "cmn_hdrs/shared_constants.h"

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

MockAisShip::MockAisShip(uint32_t seed, uint32_t id, Vec2DFloat polaris_lat_lon) : mt_rng_(seed)
{
    static const std::array<float, 2>       pos_or_neg = {-1.0, 1.0};
    std::uniform_real_distribution<float>   lat_dist(MIN_AIS_SHIP_DIST, MAX_AIS_SHIP_DIST);
    std::uniform_real_distribution<float>   lon_dist(MIN_AIS_SHIP_DIST, MAX_AIS_SHIP_DIST);
    std::uniform_real_distribution<float>   speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float>   heading_dist(HEADING_LBND, HEADING_UBND);
    std::uniform_int_distribution<uint32_t> pos_or_neg_dist(0, 1);

    id_      = id;
    lat_lon_ = {
      polaris_lat_lon[0] + pos_or_neg[pos_or_neg_dist(mt_rng_)] * lat_dist(mt_rng_),
      polaris_lat_lon[1] + pos_or_neg[pos_or_neg_dist(mt_rng_)] * lon_dist(mt_rng_)};
    speed_   = speed_dist(mt_rng_);
    heading_ = heading_dist(mt_rng_);
}

void MockAisShip::tick(const Vec2DFloat & polaris_lat_lon)
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
      (std::abs(std::abs(lat_lon_[0] + dir_vec[0] * speed_ - polaris_lat_lon[0])) < MAX_AIS_SHIP_DIST) &&
      (std::abs(std::abs(lat_lon_[0] + dir_vec[0] * speed_ - polaris_lat_lon[0])) > MIN_AIS_SHIP_DIST)) {
        lat_lon_[0] += dir_vec[0] * speed_;
    } else if (std::abs(lat_lon_[0] - polaris_lat_lon[0]) >= MAX_AIS_SHIP_DIST) {
        lat_lon_[0] = polaris_lat_lon[0] + MAX_AIS_SHIP_DIST - MIN_AIS_SHIP_DIST;
    } else if (std::abs(lat_lon_[0] - polaris_lat_lon[0]) < MIN_AIS_SHIP_DIST) {
        lat_lon_[0] = polaris_lat_lon[0] + 2 * MIN_AIS_SHIP_DIST;
    }
    if (
      (std::abs(std::abs(lat_lon_[1] + dir_vec[1] * speed_ - polaris_lat_lon[1])) < MAX_AIS_SHIP_DIST) &&
      (std::abs(std::abs(lat_lon_[1] + dir_vec[1] * speed_ - polaris_lat_lon[1])) > MIN_AIS_SHIP_DIST)) {
        lat_lon_[1] += dir_vec[1] * speed_;
    } else if (std::abs(lat_lon_[1] - polaris_lat_lon[1]) >= MAX_AIS_SHIP_DIST) {
        lat_lon_[1] = polaris_lat_lon[1] + MAX_AIS_SHIP_DIST - MIN_AIS_SHIP_DIST;
    } else if (std::abs(lat_lon_[1] - polaris_lat_lon[1]) < MIN_AIS_SHIP_DIST) {
        lat_lon_[1] = polaris_lat_lon[1] + 2 * MIN_AIS_SHIP_DIST;
    }
}

MockAis::MockAis(uint32_t seed, uint32_t num_ships, Vec2DFloat polaris_lat_lon) : polaris_lat_lon_(polaris_lat_lon)
{
    for (uint32_t i = 0; i < num_ships; i++) {
        ships_.push_back(MockAisShip(seed + i, i, polaris_lat_lon));
    }
}

std::vector<AisShip> MockAis::ships()
{
    std::vector<AisShip> ships;
    for (const MockAisShip & ship : ships_) {
        ships.push_back({.lat_lon_ = ship.lat_lon_, .speed_ = ship.speed_, .heading_ = ship.heading_, .id_ = ship.id_});
    }
    return ships;
}

void MockAis::updatePolarisPos(const Vec2DFloat & lat_lon) { polaris_lat_lon_ = lat_lon; }

void MockAis::tick()
{
    for (MockAisShip & ship : ships_) {
        ship.tick(polaris_lat_lon_);
    }
}
