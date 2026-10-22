#include <unity.h>
#include "storage/FixedLogBuffer.h"
#include <string.h>
void setUp(void) {}
void tearDown(void) {}
void test_full_rows_and_no_overwrite() {
    FixedLogBuffer<12> b;
    TEST_ASSERT_TRUE(b.append("abc\n", 4, 10));
    TEST_ASSERT_TRUE(b.append("defg\n", 5, 20));
    TEST_ASSERT_EQUAL_UINT(4, b.prefix(8));
    TEST_ASSERT_FALSE(b.append("xyz\n", 4, 30));
    TEST_ASSERT_EQUAL_UINT(9, b.size());
    TEST_ASSERT_EQUAL_MEMORY("abc\ndefg\n", b.data(), 9);
    TEST_ASSERT_FALSE(b.consume(2));
    TEST_ASSERT_TRUE(b.consume(4));
    TEST_ASSERT_EQUAL_MEMORY("defg\n", b.data(), 5);
    TEST_ASSERT_TRUE(b.consume(5));
    TEST_ASSERT_EQUAL_UINT(0, b.size());
}
void test_empty_invalid_and_clock_wrap() {
    FixedLogBuffer<8> b;
    TEST_ASSERT_FALSE(b.append("abc", 3, 0));
    TEST_ASSERT_FALSE(b.append(nullptr, 1, 0));
    TEST_ASSERT_EQUAL_UINT(0, b.prefix(8));
    TEST_ASSERT_TRUE(b.append("a\n", 2, 0xfffffff0u));
    TEST_ASSERT_EQUAL_UINT32(32, b.age(16));
    TEST_ASSERT_TRUE(b.consume(2));
    TEST_ASSERT_TRUE(b.append("b\n", 2, 40));
    TEST_ASSERT_EQUAL_UINT32(10, b.age(50));
}
int main(int, char**) { UNITY_BEGIN(); RUN_TEST(test_full_rows_and_no_overwrite); RUN_TEST(test_empty_invalid_and_clock_wrap); return UNITY_END(); }
