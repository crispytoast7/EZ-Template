#!/bin/bash
# host test for the json parser: extracts the parser half of json_auton.cpp
# (everything above the selector/executor, which needs the robot) and runs it
# against planner-style files.
set -e
cd "$(dirname "$0")/../.."
S=tests/host
sed -n '1,/Selector registration/p' src/EZ-Template/json_auton.cpp | sed '$d' | sed '$d' \
  | grep -v '#include "EZ-Template/api.hpp"' \
  | grep -v '#include "EZ-Template/dsr.hpp"' \
  | grep -v '#include "pros/rtos.hpp"' > /tmp/json_parser_extract.cpp
echo "}  // namespace ez" >> /tmp/json_parser_extract.cpp
c++ -std=c++17 -I "$S/stubs" -I include \
  -include /tmp/json_parser_extract.cpp "$S/json_test_main.cpp" -o /tmp/json_test
/tmp/json_test
