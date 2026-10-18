#!/bin/bash
set -euo pipefail
cd /work
c++ -O2 -std=c++17 seeded_server.cc -o build/seeded_server $(pkg-config --cflags --libs gz-sim8 gz-math7 gz-common5)
c++ -O2 -std=c++17 -shared -fPIC TruthRecorder.cc -o build/libJepaTruthRecorder.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)
for seed in 0 1 2 3 4; do build/seeded_server --probe fixed "$seed"; done
