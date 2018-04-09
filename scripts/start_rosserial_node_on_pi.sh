#!/bin/bash

sshpass -p "TheBoatDoctor" ssh -tt -o StrictHostKeyChecking=no theboatdoctor-pi@192.168.1.30 << EOSSH
	chmodmega
	startrosserial
EOSSH