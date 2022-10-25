#include "cached_fib.h"
#include "gtest/gtest.h"

CachedFib testFib = CachedFib(5);

class TestFib : public ::testing::Test
{
protected:
    TestFib()
    {
        // Every time a test is started, testFib is reinitialized with a constructor parameter of 5
        testFib = CachedFib(5);
    }

    ~TestFib() override
    {
        // Clean up after a test
    }
};

TEST_F(TestFib, TestBasic) { ASSERT_EQ(testFib.getFib(5), 3) << "5th fibonacci number must be 3!"; }
