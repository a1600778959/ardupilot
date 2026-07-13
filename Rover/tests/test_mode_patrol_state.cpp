#include <AP_gtest.h>

#include "../mode_patrol_state.h"

TEST(ModePatrolRoute, TargetChangeIsTransactional)
{
    ModePatrolRoute route;

    const ModePatrolRoute::Target first = route.start_target();
    EXPECT_EQ(first.line_index, 1U);
    EXPECT_EQ(first.phase, ModePatrolRoute::Phase::LineStart);
    EXPECT_TRUE(first.starts_new_line);
    EXPECT_TRUE(route.waiting_for_points());
    EXPECT_EQ(route.line_index(), 0U);

    route.commit(first);
    EXPECT_TRUE(route.navigating());
    EXPECT_EQ(route.line_index(), 1U);
    EXPECT_EQ(route.phase(), ModePatrolRoute::Phase::LineStart);

    ModePatrolRoute::Target line_end {};
    ASSERT_TRUE(route.next_target(line_end));
    EXPECT_EQ(line_end.line_index, 1U);
    EXPECT_EQ(line_end.phase, ModePatrolRoute::Phase::LineEnd);
    EXPECT_FALSE(line_end.starts_new_line);
    route.commit(line_end);

    ModePatrolRoute::Target next_line {};
    ASSERT_TRUE(route.next_target(next_line));
    EXPECT_EQ(next_line.line_index, 2U);
    EXPECT_EQ(next_line.phase, ModePatrolRoute::Phase::LineStart);
    EXPECT_TRUE(next_line.starts_new_line);

    // A rejected WPNav target enters Fault without committing the candidate.
    EXPECT_TRUE(route.set_fault());
    EXPECT_TRUE(route.in_fault());
    EXPECT_EQ(route.line_index(), 1U);
    EXPECT_EQ(route.phase(), ModePatrolRoute::Phase::LineEnd);
    EXPECT_FALSE(route.set_fault());
}

TEST(ModePatrolRoute, ResumeKeepsCurrentLegGeometry)
{
    ModePatrolRoute route;
    route.commit(route.start_target());

    ModePatrolRoute::Target resume {};
    ASSERT_TRUE(route.resume_target(resume));
    EXPECT_EQ(resume.line_index, 1U);
    EXPECT_EQ(resume.phase, ModePatrolRoute::Phase::LineStart);
    EXPECT_FALSE(resume.starts_new_line);
}

TEST(ModePatrolRoute, NewGeometryIsRequestedOnlyBetweenLines)
{
    ModePatrolRoute route;
    route.commit(route.start_target());

    ModePatrolRoute::Target candidate {};
    ASSERT_TRUE(route.next_target(candidate));
    EXPECT_FALSE(candidate.starts_new_line);
    route.commit(candidate);

    ASSERT_TRUE(route.next_target(candidate));
    EXPECT_TRUE(candidate.starts_new_line);
}

TEST(ModePatrolRoute, ResetClearsFaultAndCommittedProgress)
{
    ModePatrolRoute route;
    route.commit(route.start_target());
    ASSERT_TRUE(route.set_fault());

    route.reset();

    EXPECT_TRUE(route.waiting_for_points());
    EXPECT_EQ(route.line_index(), 0U);
    EXPECT_EQ(route.phase(), ModePatrolRoute::Phase::LineStart);
}

TEST(ModePatrolRoute, RefusesToOverflowLineIndex)
{
    ModePatrolRoute route;
    route.commit({UINT16_MAX, ModePatrolRoute::Phase::LineEnd, true});

    ModePatrolRoute::Target candidate {};
    EXPECT_FALSE(route.next_target(candidate));
    EXPECT_EQ(route.line_index(), UINT16_MAX);
    EXPECT_EQ(route.phase(), ModePatrolRoute::Phase::LineEnd);
}

TEST(ModePatrolPivotWatchdog, TimesOutAfterNoHeadingProgress)
{
    ModePatrolPivotWatchdog watchdog;

    EXPECT_FALSE(watchdog.update(true, 90.0f, 100U, 1000U));
    EXPECT_FALSE(watchdog.update(true, 90.0f, 1099U, 1000U));
    EXPECT_TRUE(watchdog.update(true, 90.0f, 1100U, 1000U));
}

TEST(ModePatrolPivotWatchdog, RealProgressRestartsDeadline)
{
    ModePatrolPivotWatchdog watchdog;

    EXPECT_FALSE(watchdog.update(true, 90.0f, 100U, 1000U));
    EXPECT_FALSE(watchdog.update(true, 89.4f, 800U, 1000U));
    EXPECT_FALSE(watchdog.update(true, 89.4f, 1799U, 1000U));
    EXPECT_TRUE(watchdog.update(true, 89.4f, 1800U, 1000U));
}

TEST(ModePatrolPivotWatchdog, SensorNoiseDoesNotPostponeTimeout)
{
    ModePatrolPivotWatchdog watchdog;

    EXPECT_FALSE(watchdog.update(true, 90.0f, 100U, 1000U));
    EXPECT_FALSE(watchdog.update(true, 89.6f, 800U, 1000U));
    EXPECT_TRUE(watchdog.update(true, 89.6f, 1100U, 1000U));
}

TEST(ModePatrolPivotWatchdog, DisabledOrInactiveMonitoringResetsDeadline)
{
    ModePatrolPivotWatchdog watchdog;

    EXPECT_FALSE(watchdog.update(true, 90.0f, 100U, 1000U));
    EXPECT_FALSE(watchdog.update(false, 90.0f, 900U, 1000U));
    EXPECT_FALSE(watchdog.update(true, 90.0f, 1000U, 1000U));
    EXPECT_FALSE(watchdog.update(true, 90.0f, 1999U, 0U));
    EXPECT_FALSE(watchdog.update(true, 90.0f, 2000U, 1000U));
}

TEST(ModePatrolPivotWatchdog, HandlesMillisWraparound)
{
    ModePatrolPivotWatchdog watchdog;

    constexpr uint32_t start_ms = UINT32_MAX - 499U;
    EXPECT_FALSE(watchdog.update(true, 90.0f, start_ms, 1000U));
    EXPECT_FALSE(watchdog.update(true, 90.0f, 499U, 1000U));
    EXPECT_TRUE(watchdog.update(true, 90.0f, 500U, 1000U));
}

AP_GTEST_MAIN()
