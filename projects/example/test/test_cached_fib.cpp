#include "cached_fib.h"
#include "gtest/gtest.h"

constexpr int defaultSize = 5;

static CachedFib testFib = CachedFib(defaultSize);

class TestFib : public ::testing::Test
{
protected:
    TestFib()
    {
        // Every time a test is started, testFib is reinitialized with a constructor parameter of 5
        testFib = CachedFib(defaultSize);
    }

    ~TestFib() override
    {
        // Clean up after a test
    }
};

TEST_F(TestFib, TestBasic) { ASSERT_EQ(testFib.getFib(5), 3) << "5th fibonacci number must be 3!"; }

TEST_F(TestFib, TestBasic2)
{
    ASSERT_EQ(testFib.getFib(6), 5);
    ASSERT_EQ(testFib.getFib(7), 8);
}
