#!/bin/sh
hostname="192.168.199.60"
password="raspberry"

if [ $# = 1 ]; then
    hostname=$1
fi
if [ $# = 2 ]; then
    hostname=$1
    password=$2
fi

echo "hostname: ${hostname}"
echo "password: ${password}"

sshpass -p "${password}" ssh -o StrictHostKeyChecking=no pi@"${hostname}" 'mkdir -p RflyPilot_Project/RflyPilot/'
sshpass -p "${password}" scp -r ./rflypilot ../config/rflypilot.txt ../config/parameter.txt pi@"${hostname}":/home/pi/RflyPilot_Project/RflyPilot/
# sshpass -p "${password}" scp -r ./rflypilot ../config/rflypilot.txt ../config/calibration.txt ../config/parameter.txt pi@"${hostname}":/home/pi/RflyPilot_Project/RflyPilot/