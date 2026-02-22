#!/usr/bin/env bash
set -e
export UA_API_SECRET_INTENTIONALLY_EMPTY=1

bin/package

LIBS=out/xrDriver-libs-x86_64.tar.gz
if [ ! -f "$LIBS" ]; then
    echo "Downloading libs..."
    curl -L https://github.com/wheaney/XRLinuxDriver/releases/latest/download/xrDriver-libs-x86_64.tar.gz \
         -o "$LIBS"
fi

sudo bin/xr_driver_setup "$(pwd)/out"
systemctl --user daemon-reload
systemctl --user restart xr-driver.service
echo "Done. Tailing log..."
tail -20 ~/.local/state/xr_driver/driver.log
