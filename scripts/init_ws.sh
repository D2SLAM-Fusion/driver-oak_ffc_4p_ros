#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_DIR=$(cd $SCRIPT_DIR/..; pwd)

cd $BASE_DIR
python3 -m pip install pre-commit
pre-commit install