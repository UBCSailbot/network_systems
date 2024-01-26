#include <gtest/gtest.h>

#include <cstring>

#include "can_frame_parser.h"
#include "cmn_hdrs/shared_constants.h"

/**
 * @brief Test CAN<->ROS translations
 *
 */
class TestCanFrameParser : public ::testing::Test
{
protected:
    TestCanFrameParser() {}
    ~TestCanFrameParser() override {}
};

TEST_F(TestCanFrameParser, BatteryTestValid)
{
    constexpr std::array<float, NUM_BATTERIES> expected_volts{12.5, 10.6};
    constexpr std::array<float, NUM_BATTERIES> expected_expected_currs{
      2.5, -1.0};  // negative expected_currents are valid
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_volts{1250, 1060};
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_expected_currs{250, -100};

    for (size_t i = 0; i < NUM_BATTERIES; i++) {
        CAN::CanId                            id            = CAN::Battery::BATTERY_IDS[i];
        float                                 expected_volt = expected_volts[i];
        float                                 expected_curr = expected_expected_currs[i];
        custom_interfaces::msg::HelperBattery msg;
        msg.set__voltage(expected_volt);
        msg.set__current(expected_curr);
        CAN::Battery  bat_from_ros = CAN::Battery(msg, i);
        CAN::CanFrame cf           = bat_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN::Battery::CAN_BYTE_DLEN_);

        int16_t raw_volt;
        int16_t raw_curr;
        std::memcpy(&raw_volt, cf.data, sizeof(int16_t));
        std::memcpy(&raw_curr, cf.data + 2, sizeof(int16_t));

        EXPECT_EQ(raw_volt, expected_raw_volts[i]);
        EXPECT_EQ(raw_curr, expected_raw_expected_currs[i]);

        CAN::Battery bat_from_can = CAN::Battery(cf);

        EXPECT_EQ(bat_from_can.id_, id);

        custom_interfaces::msg::HelperBattery msg_from_bat = bat_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_bat.voltage, expected_volt);
        EXPECT_DOUBLE_EQ(msg_from_bat.current, expected_curr);
    }
}

TEST_F(TestCanFrameParser, TestBatteryInvalid)
{
    custom_interfaces::msg::HelperBattery msg;

    EXPECT_THROW(CAN::Battery tmp(msg, NUM_BATTERIES), std::length_error);

    CAN::CanId    invalid_id = CAN::CanId::RESERVED;
    CAN::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN::Battery tmp(cf), CAN::CanIdMismatchException);
}
