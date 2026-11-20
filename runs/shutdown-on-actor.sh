#!/bin/bash

# Usage: sudo bash /path/to/shutdown-on-actor.sh
if [ "$#" -ne 0 ]; then
    echo "Usage: $0" >&2
    exit 2
fi

echo "Monitoring the current directory and its subdirectories."

while true; do
    if ! file=$(find . -type f -name actor.h -print -quit); then
        echo "Failed to search for actor.h." >&2
        exit 1
    fi
    if [ -n "$file" ]; then
        break
    fi
    sleep 30
done

echo "File detected: $file. Shutting down in 10 seconds (Ctrl+C to cancel)."
sleep 10
if ! systemctl --no-ask-password poweroff; then
    echo "Shutdown failed. Run the script with the required permissions on the Ubuntu host." >&2
    exit 1
fi
