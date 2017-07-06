#!/bin/bash

LOG_DIR=$1
if [ -n "$LOG_DIR" ] && ! [[ "$LOG_DIR" = /* ]]; then
    LOG_DIR=$PWD/$LOG_DIR
fi

if [ -n "$LOG_DIR" ]; then
    echo "Logging output to the following directory:"
    echo "$LOG_DIR"
    echo "CPPLINT log from $(date)" > "$LOG_DIR"
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")
CPPLINT_DIR=$REPO_DIR/dependencies/cpplint/cpplint

cd "$CPPLINT_DIR"

LINT_DIRS=(
    "rospackages"
)

FLAGS=(
    "--extensions=cpp,hpp,h"
    "--filter=-legal/copyright"
)

for dir in "${LINT_DIRS[@]}";
do
    if [ -n "$LOG_DIR" ]; then
        echo "Linting the $dir directory ..." >> "$LOG_DIR"
        find "$REPO_DIR/$dir" -type f \
            \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
            -print0 | xargs -0 ./cpplint.py "${FLAGS[@]}" >> "$LOG_DIR" 2>&1
    else
        echo "Linting the $dir directory ..."
        find "$REPO_DIR/$dir" -type f \
            \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
            -print0 | xargs -0 ./cpplint.py "${FLAGS[@]}"
    fi
done
