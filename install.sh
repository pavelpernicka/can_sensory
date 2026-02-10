#!/usr/bin/env bash
set -e

CUBE_DIR="third_party/STM32CubeL4"

if [ ! -d "$CUBE_DIR" ]; then
    mkdir -p third_party
    echo "Cloning STM32CubeL4..."
    git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/STMicroelectronics/STM32CubeL4.git "$CUBE_DIR"
else
    echo "STM32CubeL4 already present, skipping"
fi

echo "Done"

