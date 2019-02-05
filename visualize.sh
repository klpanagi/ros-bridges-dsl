#!/usr/bin/env bash


if [[ "${1}" == "" ]]; then
  METAMODEL_FILE="ros-amqp-metamodel.tx"
else
  METAMODEL_FILE=${1}
fi

command -v textx >/dev/null 2>&1 || { echo >&2 "TextX cli is required and is not installed.  Aborting."; exit 1; }
command -v dot >/dev/null 2>&1 || { echo >&2 "Dot is required and is not installed.  Aborting."; exit 1; }

textx visualize "${METAMODEL_FILE}"
dot -Tpng -O "${METAMODEL_FILE}.dot"
