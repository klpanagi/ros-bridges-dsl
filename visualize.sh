#!/usr/bin/env bash

command -v textx >/dev/null 2>&1 || { echo >&2 "TextX cli is required and is not installed.  Aborting."; exit 1; }
command -v dot >/dev/null 2>&1 || { echo >&2 "Dot is required and is not installed.  Aborting."; exit 1; }

textx visualize metamodel.tx
dot -Tpng -O metamodel.tx.dot
