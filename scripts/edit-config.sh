#!/bin/bash

function write_source_line {
    if [ -e "$1" ]; then
        SOURCE_LINE="source $(readlink -f $2)"
        grep "$SOURCE_LINE" "$1" > /dev/null 2>&1
        if [ $? != 0 ]; then
            echo $SOURCE_LINE >> $1
        fi
    fi
}

# The output directory is the first argument
SCRIMMAGE_LOCAL_CONFIG_DIR=$(readlink -f "$1")
PROJECT_NAME="$2"

touch ${SCRIMMAGE_LOCAL_CONFIG_DIR}/setup.bash
write_source_line "${SCRIMMAGE_LOCAL_CONFIG_DIR}/setup.bash" "${SCRIMMAGE_LOCAL_CONFIG_DIR}/env/${PROJECT_NAME}-setenv"
