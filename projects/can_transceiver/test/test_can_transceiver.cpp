#include <gtest/gtest.h>

#include <cstring>

#include "can_frame_parser.h"
#include "cmn_hdrs/shared_constants.h"

namespace msg = custom_interfaces::msg;

constexpr CAN::RawDataBuf GARBAGE_DATA = []() constexpr
{
    CAN::RawDataBuf buf;
    for (uint8_t & entry : buf) {
        entry = 0xFF;  // NOLINT(readability-magic-numbers)
    }
    return buf;
}
();

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

/**
 * @brief Test CAN<->ROS Battery translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, BatteryTestValid)
{
    constexpr std::array<float, NUM_BATTERIES>   expected_volts{12.5, 10.6};
    constexpr std::array<float, NUM_BATTERIES>   expected_currs{2.5, -1.0};  // negative currents are valid
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_volts{1250, 1060};
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_expected_currs{250, -100};

    for (size_t i = 0; i < NUM_BATTERIES; i++) {
        auto optId = CAN::Battery::rosIdxToCanId(i);

        ASSERT_TRUE(optId.has_value());

        CAN::CanId         id            = optId.value();
        float              expected_volt = expected_volts[i];
        float              expected_curr = expected_currs[i];
        msg::HelperBattery msg;
        msg.set__voltage(expected_volt);
        msg.set__current(expected_curr);
        CAN::Battery  bat_from_ros = CAN::Battery(msg, id);
        CAN::CanFrame cf           = bat_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN::Battery::CAN_BYTE_DLEN_);

        int16_t raw_volt;
        int16_t raw_curr;
        std::memcpy(&raw_volt, cf.data + CAN::Battery::BYTE_OFF_VOLT, sizeof(int16_t));
        std::memcpy(&raw_curr, cf.data + CAN::Battery::BYTE_OFF_CURR, sizeof(int16_t));

        EXPECT_EQ(raw_volt, expected_raw_volts[i]);
        EXPECT_EQ(raw_curr, expected_raw_expected_currs[i]);

        CAN::Battery bat_from_can = CAN::Battery(cf);

        EXPECT_EQ(bat_from_can.id_, id);

        msg::HelperBattery msg_from_bat = bat_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_bat.voltage, expected_volt);
        EXPECT_DOUBLE_EQ(msg_from_bat.current, expected_curr);
    }
}

/**
 * @brief Test the behavior of the Battery class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestBatteryInvalid)
{
    auto optId = CAN::Battery::rosIdxToCanId(NUM_BATTERIES);
    EXPECT_FALSE(optId.has_value());

    CAN::CanId invalid_id = CAN::CanId::RESERVED;

    CAN::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN::Battery tmp(cf), CAN::CanIdMismatchException);

    std::vector<float> invalid_volts{BATT_VOLT_LBND - 1, BATT_VOLT_UBND + 1};
    std::vector<float> invalid_currs{BATT_CURR_LBND - 1, BATT_CURR_UBND + 1};

    optId = CAN::Battery::rosIdxToCanId(0);
    ASSERT_TRUE(optId.has_value());

    CAN::CanId         valid_id = optId.value();
    msg::HelperBattery msg;

    // Set a valid current for this portion
    for (float invalid_volt : invalid_volts) {
        msg.set__voltage(invalid_volt);
        msg.set__current(BATT_CURR_LBND);

        EXPECT_THROW(CAN::Battery tmp(msg, valid_id), std::out_of_range);
    };

    // Set a valid voltage for this portion
    for (float invalid_curr : invalid_currs) {
        msg.set__voltage(BATT_VOLT_LBND);
        msg.set__current(invalid_curr);

        EXPECT_THROW(CAN::Battery tmp(msg, valid_id), std::out_of_range);
    };

    cf.can_id = static_cast<canid_t>(CAN::CanId::BMS_P_DATA_FRAME_1);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN::Battery tmp(cf), std::out_of_range);
}