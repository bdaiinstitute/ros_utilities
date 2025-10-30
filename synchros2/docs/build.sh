#!/bin/bash

SCRIPT_DIRPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PACKAGE_NAME="synchros2"
PACKAGE_PATH="$(cd "$SCRIPT_DIRPATH/.." && pwd)"
GEN_PATH=$PACKAGE_PATH/.generated
mkdir -p $GEN_PATH

# Command to build and open documentation
build() {
    cd $GEN_PATH
    rosdoc2 build --package-path $PACKAGE_PATH 
    rosdoc2 open docs_output/$PACKAGE_NAME
    cd -
}

# Parse options using getopt
OPTIONS=$(getopt -o hw --long help,watch -- "$@")
if [ $? -ne 0 ]; then
    echo "Failed to parse options." >&2
    exit 1
fi
eval set -- "$OPTIONS"

WATCH_MODE=0
while true; do
    case "$1" in
        -w|--watch)
            WATCH_MODE=1
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [-w|--watch] [-h|--help]"
            echo "  -w, --watch   Watch for file changes and rebuild documentation automatically."
            echo "  -h, --help    Show this help message and exit."
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [-w|--watch] [-h|--help]"
            exit 1
            ;;
    esac
done

if ! command -v rosdoc2 &> /dev/null; then
    read -p "rosdoc2 is not installed. Do you want to install it using pipx? [y/N]: " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        if ! command -v pipx &> /dev/null; then
            echo "pipx is required but not installed. Please install pipx first."
            exit 1
        fi
        pipx install --include-deps "git+https://github.com/ros-infrastructure/rosdoc2@main"
        pipx runpip rosdoc2 install --upgrade sphinxcontrib-mermaid
    else
        echo "rosdoc2 is required to build documentation."
        exit 1
    fi
fi

if [ "$WATCH_MODE" -eq 1 ]; then
    if ! command -v inotifywait &> /dev/null; then
        echo "inotifywait is required but not installed. Please install inotify-tools first."
        exit 1
    fi
fi

echo "Building documentation..."
build

if [ "$WATCH_MODE" -ne 1 ]; then
    exit 0
fi

echo "Watching for file changes in $PACKAGE_PATH..."
inotifywait -m -r -e modify,create,delete --exclude "$GEN_PATH/" --format '%w%f' $PACKAGE_PATH | \
while read -r file
do
    echo "Change detected: $file"
    echo "Batching $(timeout 3 cat | wc -l) changes"
    echo "Rebuilding documentation..."
    build
done
