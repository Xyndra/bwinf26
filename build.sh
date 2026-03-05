#!/bin/bash
# Only build the project. Run local_setup.sh first.
. ./esp/esp-idf/export.sh
idf.py build
