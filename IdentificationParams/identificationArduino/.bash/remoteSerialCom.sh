#!/bin/bash

# Doit avoir minicom d'installer sur le RPI (commande: sudo apt install minicom)
# $1: Premier argument est le nom d'utilisateur sur le RPI
# $2: Deuxieme argument est l'adresse du RPI
# $3: Troisieme argument est le port du Arduino
# $4: Quatrieme argument vitesse de communication
# transfert du fichier compile "firmware.hex" vers le RPI
ssh -t $1@$2 'minicom -D '$3' -b '$4