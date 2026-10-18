#!/bin/bash
set -euo pipefail
cd /work
mkdir -p build
c++ -O2 -std=c++17 seeded_server.cc -o build/seeded_server $(pkg-config --cflags --libs gz-sim8 gz-math7 gz-common5)
