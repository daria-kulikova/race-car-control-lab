#!/usr/bin/env bash

# Check if the script is being run or sourced (must source for idf.py to be exported)
if [ "$0" = "$BASH_SOURCE" ]; then
    echo "Don't run $0, source it: . $0"
    exit 1
fi

# Check that we're in the root folder of the project
if [[ -f docker-compose.yaml ]]; then
    printf "Configuring ESP-IDF...\n"
else
    printf "Cannot run setup script in this context. Switch to root project folder.\n"
    exit 1
fi

# Add environment variables to tell ESP-IDF where it should install things
mkdir -p lib/esp-idf-tools

export IDF_PATH=$(cd lib/esp-idf; pwd)
export IDF_TOOLS_PATH=$(cd lib/esp-idf-tools; pwd)

# Execute ESP-IDF install script, will install to lib/esp-idf/tools subdirectory
./lib/esp-idf/install.sh

# Export the ESP-IDF path to the shell by sourcing
. ./lib/esp-idf/export.sh

pip install pre-commit protobuf grpcio-tools

# Initialize the VS Code Settings
cp .vscode/settings.json.template .vscode/settings.json

printf "Finished setup! Happy coding.\n"
