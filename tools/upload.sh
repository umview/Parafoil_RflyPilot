hostname='192.168.199.226'
password='raspberry'
sshpass -p ${password} ssh pi@${hostname} 'mkdir -p RflyPilot_Project/RflyPilot/'
sshpass -p ${password} scp -r ./rflypilot ../config/rflypilot.txt ../config/calibration.txt ../config/parameter.txt pi@${hostname}:/home/pi/RflyPilot_Project/RflyPilot/

