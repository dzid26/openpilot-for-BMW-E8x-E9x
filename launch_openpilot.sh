#!/usr/bin/env bash


export FINGERPRINT="BMW_E82"
export SKIP_FW_QUERY="1"
export STARTED="1"
export TESTING_CLOSET="1"
export ALLOW_DEBUG="1"
export ZMQ=1
exec ./launch_chffrplus.sh
