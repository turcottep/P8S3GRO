#!/bin/bash

# $1: Premier argument est le nom d'utilisateur sur le RPI
# $2: Deuxieme argument est l'adresse du RPI
# $3: Troisieme argument est le chemin vers le fichier compile "firmware.hex"
# $4: Quatrieme argument est le port du Arduino"

# transfert du fichier compile "firmware.hex" vers le RPI
scp $3 $1@$2:/tmp/

# Envoie de la commande d'ecriture Arduino par SSH au RPI
ssh $1@$2 'avrdude -v -patmega2560 -cwiring -P'$4' -b115200 -D -Uflash:w:/tmp/firmware.hex'