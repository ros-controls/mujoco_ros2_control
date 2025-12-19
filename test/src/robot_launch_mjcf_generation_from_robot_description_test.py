#!/usr/bin/env python3

# Copyright 2025 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import os
import pytest


sys.path.append(os.path.dirname(__file__))  # noqa: E402

from robot_launch_test import generate_test_description_common  # noqa: E402
from robot_launch_pid_test import TestFixtureHardwareInterfacesCheck  # noqa: F401, E402
from robot_launch_test import TestFixture, TestMJCFGenerationFromURDF  # noqa: F401, E402


@pytest.mark.rostest
def generate_test_description():
    return generate_test_description_common(use_pid="true", use_mjcf_from_topic="true")
