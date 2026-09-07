/**
 * Copyright (c) 2026 PAL Robotics S.L.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "depth_sensor_model.hpp"

using mujoco_ros2_control_plugins::DepthSensorModel;

namespace
{
/// Focal length of the D435i colour/depth stream at 848x480, as published in camera_info.
constexpr float FX = 617.0f;

/// A model with the noise sources switched off, so range handling can be checked exactly.
DepthSensorModel noiseless()
{
  DepthSensorModel model;
  model.subpixel_error = 0.0f;
  model.dropout_fraction = 0.0;
  return model;
}
}  // namespace

TEST(DepthSensorModel, DefaultsDescribeAD435i)
{
  const DepthSensorModel model;
  EXPECT_FLOAT_EQ(model.min_range, 0.28f);
  EXPECT_FLOAT_EQ(model.max_range, 3.0f);
  EXPECT_FLOAT_EQ(model.stereo_baseline, 0.05f);
  EXPECT_FLOAT_EQ(model.subpixel_error, 0.15f);
  EXPECT_DOUBLE_EQ(model.dropout_fraction, 0.02);
}

TEST(DepthSensorModel, PassesSamplesInsideTheUsableRange)
{
  auto model = noiseless();
  for (const float depth : { 0.28f, 0.5f, 1.5f, 3.0f })
  {
    EXPECT_FLOAT_EQ(model.apply(depth, FX), depth) << "at " << depth << " m";
  }
}

TEST(DepthSensorModel, ReturnsNanBelowMinRange)
{
  auto model = noiseless();
  // The regression this guards: MuJoCo happily reports 0.2477 m, inside the D435i's Min-Z.
  EXPECT_TRUE(std::isnan(model.apply(0.2477f, FX)));
  EXPECT_TRUE(std::isnan(model.apply(0.0f, FX)));
  EXPECT_TRUE(std::isnan(model.apply(std::nextafterf(0.28f, 0.0f), FX)));
}

TEST(DepthSensorModel, ReturnsNanAboveMaxRange)
{
  auto model = noiseless();
  EXPECT_TRUE(std::isnan(model.apply(3.01f, FX)));
  EXPECT_TRUE(std::isnan(model.apply(100.0f, FX)));
  EXPECT_TRUE(std::isnan(model.apply(std::numeric_limits<float>::infinity(), FX)));
}

TEST(DepthSensorModel, PropagatesNanInput)
{
  auto model = noiseless();
  EXPECT_TRUE(std::isnan(model.apply(std::numeric_limits<float>::quiet_NaN(), FX)));
}

TEST(DepthSensorModel, RangeBoundsAreInclusive)
{
  auto model = noiseless();
  EXPECT_FALSE(std::isnan(model.apply(model.min_range, FX)));
  EXPECT_FALSE(std::isnan(model.apply(model.max_range, FX)));
}

TEST(DepthSensorModel, TriangulationErrorGrowsWithTheSquareOfRange)
{
  const DepthSensorModel model;
  // sigma(Z) = Z^2 * subpixel / (fx * baseline): doubling the range quadruples the error.
  const float s1 = model.stddevAt(1.0f, FX);
  const float s2 = model.stddevAt(2.0f, FX);
  EXPECT_NEAR(s2 / s1, 4.0f, 1e-4f);
}

TEST(DepthSensorModel, TriangulationErrorMatchesTheDocumentedTable)
{
  const DepthSensorModel model;
  // The table in the header, in millimetres, for fx 617 and a 50 mm baseline.
  // The table is quoted to 0.1 mm, so allow the rounding in either direction.
  EXPECT_NEAR(1e3f * model.stddevAt(0.5f, FX), 1.2f, 0.1f);
  EXPECT_NEAR(1e3f * model.stddevAt(1.0f, FX), 4.9f, 0.1f);
  EXPECT_NEAR(1e3f * model.stddevAt(2.0f, FX), 19.4f, 0.1f);
  EXPECT_NEAR(1e3f * model.stddevAt(3.0f, FX), 43.7f, 0.1f);
  // About 1% of the range at 2 m, the figure usually quoted for a D435i.
  EXPECT_NEAR(model.stddevAt(2.0f, FX) / 2.0f, 0.01f, 0.001f);
}

TEST(DepthSensorModel, ShorterFocalLengthMeansLargerError)
{
  const DepthSensorModel model;
  // The infra streams have a shorter focal length, so they are noisier at the same range.
  EXPECT_GT(model.stddevAt(2.0f, 433.0f), model.stddevAt(2.0f, FX));
}

TEST(DepthSensorModel, ZeroFocalLengthDoesNotDivideByZero)
{
  const DepthSensorModel model;
  EXPECT_TRUE(std::isfinite(model.stddevAt(1.0f, 0.0f)));
}

TEST(DepthSensorModel, NoiseHasTheModelledMagnitude)
{
  DepthSensorModel model;
  model.dropout_fraction = 0.0;  // isolate the noise from the holes
  constexpr float depth = 2.0f;
  constexpr int samples = 20000;

  double sum = 0.0;
  double sum_sq = 0.0;
  int valid = 0;
  for (int i = 0; i < samples; ++i)
  {
    const float value = model.apply(depth, FX);
    if (!std::isnan(value))
    {
      const double error = value - depth;
      sum += error;
      sum_sq += error * error;
      ++valid;
    }
  }
  ASSERT_GT(valid, samples / 2);
  const double mean = sum / valid;
  const double stddev = std::sqrt(sum_sq / valid - mean * mean);

  // Unbiased, and scattered by the sigma the formula predicts (19.4 mm at 2 m).
  EXPECT_NEAR(mean, 0.0, 1e-3);
  EXPECT_NEAR(stddev, model.stddevAt(depth, FX), 0.1 * model.stddevAt(depth, FX));
}

TEST(DepthSensorModel, DropoutFractionIsRespected)
{
  DepthSensorModel model;
  model.subpixel_error = 0.0f;  // isolate the holes from the noise
  model.dropout_fraction = 0.25;
  constexpr int samples = 20000;

  int dropped = 0;
  for (int i = 0; i < samples; ++i)
  {
    if (std::isnan(model.apply(1.0f, FX)))
    {
      ++dropped;
    }
  }
  EXPECT_NEAR(static_cast<double>(dropped) / samples, 0.25, 0.02);
}

TEST(DepthSensorModel, ZeroDropoutNeverDropsAnInRangeSample)
{
  DepthSensorModel model;
  model.subpixel_error = 0.0f;
  model.dropout_fraction = 0.0;
  for (int i = 0; i < 1000; ++i)
  {
    ASSERT_FALSE(std::isnan(model.apply(1.0f, FX))) << "iteration " << i;
  }
}

TEST(DepthSensorModel, IsDeterministicForAGivenSeed)
{
  DepthSensorModel a(7u);
  DepthSensorModel b(7u);
  for (int i = 0; i < 500; ++i)
  {
    const float lhs = a.apply(1.0f, FX);
    const float rhs = b.apply(1.0f, FX);
    ASSERT_EQ(std::isnan(lhs), std::isnan(rhs)) << "iteration " << i;
    if (!std::isnan(lhs))
    {
      ASSERT_FLOAT_EQ(lhs, rhs) << "iteration " << i;
    }
  }
}

TEST(DepthSensorModel, DifferentSeedsGiveDifferentNoise)
{
  DepthSensorModel a(1u);
  DepthSensorModel b(2u);
  bool differs = false;
  for (int i = 0; i < 100 && !differs; ++i)
  {
    const float lhs = a.apply(1.0f, FX);
    const float rhs = b.apply(1.0f, FX);
    differs = std::isnan(lhs) != std::isnan(rhs) || (!std::isnan(lhs) && std::fabs(lhs - rhs) > 0.0f);
  }
  EXPECT_TRUE(differs);
}

TEST(DepthSensorModel, ReseedRestartsTheSequence)
{
  DepthSensorModel model(3u);
  std::vector<float> first;
  for (int i = 0; i < 50; ++i)
  {
    first.push_back(model.apply(1.0f, FX));
  }
  model.reseed(3u);
  for (std::size_t i = 0; i < first.size(); ++i)
  {
    const float again = model.apply(1.0f, FX);
    ASSERT_EQ(std::isnan(first[i]), std::isnan(again)) << "sample " << i;
    if (!std::isnan(again))
    {
      ASSERT_FLOAT_EQ(first[i], again) << "sample " << i;
    }
  }
}

TEST(DepthSensorModel, NoiseCannotProduceAnOutOfRangeReading)
{
  DepthSensorModel model;
  // Right at the near edge, where the noise is as likely to push the sample below Min-Z as
  // above it. Anything returned must still be a reading the sensor could produce.
  for (int i = 0; i < 5000; ++i)
  {
    const float value = model.apply(model.min_range, FX);
    if (!std::isnan(value))
    {
      ASSERT_GE(value, model.min_range);
      ASSERT_LE(value, model.max_range);
    }
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
