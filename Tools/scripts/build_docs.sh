#!/usr/bin/env bash

set -e

# work from either APM directory or above
[ -d Rover ] || cd APM

export DOCS_OUTPUT_BASE=./docs

(
$DOCS_OUTPUT_BASE/build-libs.sh
$DOCS_OUTPUT_BASE/build-apmrover2.sh
) > $DOCS_OUTPUT_BASE/build_docs.log 2>&1
