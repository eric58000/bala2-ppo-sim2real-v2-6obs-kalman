#!/bin/bash

# Usage: sudo bash /chemin/vers/shutdown-on-actor.sh
if [ "$#" -ne 0 ]; then
    echo "Usage: $0" >&2
    exit 2
fi

echo "Surveillance du dossier courant et de ses sous-dossiers."

while true; do
    if ! file=$(find . -type f -name actor.h -print -quit); then
        echo "Echec de la recherche de actor.h." >&2
        exit 1
    fi
    if [ -n "$file" ]; then
        break
    fi
    sleep 30
done

echo "Fichier detecte : $file. Extinction dans 10 secondes (Ctrl+C pour annuler)."
sleep 10
if ! systemctl --no-ask-password poweroff; then
    echo "Echec de l'extinction. Lancez le script avec les droits necessaires sur l'hote Ubuntu." >&2
    exit 1
fi
